"""Enforces the layer registry in contracts.LAYERS against actual module imports."""

import ast
import sys
import unittest
from pathlib import Path

from tools.trajectory_sim.contracts import LAYERS, ScalarTimeLaw, layer_of
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw

PACKAGE = Path(__file__).resolve().parents[1]
PACKAGE_NAME = "tools.trajectory_sim"


def package_modules() -> list[str]:
    return sorted(path.stem for path in PACKAGE.glob("*.py") if path.stem != "__init__")


def imports_of(source: str) -> tuple[set[str], set[str]]:
    """Return (sibling modules, third-party top-level packages) imported by package module source."""
    tree = ast.parse(source)
    siblings, external = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level == 1:
                if node.module:
                    siblings.add(node.module.split(".")[0])
                else:
                    siblings.update(alias.name for alias in node.names)
            elif node.module and node.module.startswith(PACKAGE_NAME + "."):
                siblings.add(node.module.removeprefix(PACKAGE_NAME + ".").split(".")[0])
            elif node.level == 0 and node.module:
                external.add(node.module.split(".")[0])
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith(PACKAGE_NAME + "."):
                    siblings.add(alias.name.removeprefix(PACKAGE_NAME + ".").split(".")[0])
                else:
                    external.add(alias.name.split(".")[0])
    return siblings, {name for name in external if name not in sys.stdlib_module_names}


class ArchitectureTests(unittest.TestCase):
    def test_every_module_registered_once(self):
        for module in package_modules():
            with self.subTest(module=module):
                layer_of(module)

    def test_implemented_layers_exist_and_planned_do_not(self):
        present = set(package_modules())
        for layer in LAYERS:
            for module in layer.modules:
                with self.subTest(module=module):
                    if layer.implemented:
                        self.assertIn(module, present, f"{layer.name} is marked implemented")

    def test_imports_follow_layer_rules(self):
        for module in package_modules():
            layer = layer_of(module)
            siblings, external = imports_of((PACKAGE / f"{module}.py").read_text(encoding="utf-8"))
            with self.subTest(module=module):
                self.assertLessEqual(siblings, set(layer.may_import), f"{module} imports outside its layer contract")
                self.assertLessEqual(external, set(layer.third_party), f"{module} uses unregistered third-party packages")

    def test_dependencies_point_downward(self):
        for layer in LAYERS:
            for dependency in layer.may_import:
                with self.subTest(layer=layer.name, dependency=dependency):
                    if layer.level == 0:
                        self.assertEqual(layer_of(dependency).level, 0)
                    else:
                        self.assertLess(layer_of(dependency).level, layer.level)

    def test_validation_is_independent_of_production_layers(self):
        self.assertEqual(layer_of("validation").may_import, ("model",))

    def test_analytic_time_law_satisfies_protocol(self):
        self.assertIsInstance(AnalyticTimeLaw(), ScalarTimeLaw)

    def test_detector_catches_forbidden_import(self):
        siblings, external = imports_of("from .lookahead import x\nimport numpy\nfrom . import geometry\n")
        self.assertEqual(siblings, {"lookahead", "geometry"})
        self.assertEqual(external, {"numpy"})


if __name__ == "__main__":
    unittest.main()
