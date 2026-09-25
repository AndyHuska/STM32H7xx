# STM32H7xx Project Log

## 2026-09-22 - Waveshare OpenH743I-C Ethernet/WebUI Bring-up

Summary:
- Firmware builds and flashes successfully with PlatformIO (parallel build enabled with `-j`).
- Native USB console operational.
- Ethernet and WebUI are functional.
- Board now uses DP83848 PHY path for this target.

Observed runtime diagnostics:
- [MAC:02:80:e1:b8:47:56]
- [IP:192.168.1.218]
- [ETHDBG:LINK=up,IPMODE=dhcp,PHYINIT=0,PHYSTATE=2,NETIF=up]

Interpretation:
- LINK=up, NETIF=up: MAC + lwIP interface are healthy.
- PHYINIT=0: PHY init succeeds.
- PHYSTATE=2: negotiated 100M full duplex.
- IP populated: DHCP lease successful.

Implemented during bring-up (development phase):
- Added DP83848 PHY component sources under Drivers/BSP/Components/dp83848.
- Added PHY selection support in LWIP/Target/ethernetif.c (LAN8742 vs DP83848 via build macro).
- Enabled DP83848 for Waveshare build in platformio.ini.
- Added temporary Ethernet diagnostics emitted via $I output (ETHDBG line).
- Added EventOut support and extra auxiliary outputs for LED-based status signaling on Waveshare/reference map setup.

Known workflow notes:
- PlatformIO may cache local libraries under .pio/libdeps; deleting cached Target can be necessary after changing LWIP/Target sources.
- If build/link reports firmware.elf in-use, close active debug session/process and rebuild.

## PR Cleanup Checklist (to do before upstream contribution)

1. Remove or guard development-only diagnostics:
- Decide whether ETHDBG line remains behind a compile-time flag or is removed.

2. Consolidate board support definitions:
- Ensure Waveshare-specific macros and pin mappings are explicit and documented.
- Confirm no accidental impact to existing supported boards.

3. Normalize library/dependency usage:
- Verify no reliance on stale .pio/libdeps copies.
- Confirm clean clone + clean build path works without manual cache deletion.

4. Validate release behavior:
- Test DHCP and static IP modes.
- Test link up/down handling and reconnection behavior.
- Test WebUI, HTTP, WebSocket, Telnet (if enabled), FTP/WebDAV (if enabled).

5. Documentation updates:
- Add board/PHY notes and required build flags to README/platformio comments.
- Add migration/usage notes for Waveshare OpenH743I-C + DP83848.

6. Prepare contribution hygiene:
- Separate functional commits (PHY support, board map, diagnostics, tasks/docs).
- Keep temporary local development conveniences out of final PR unless broadly useful.

## 2026-09-23 - H743 MOSFET PWM Plugin (8 channels)

Summary:
- Added a board-specific plugin for Waveshare OpenH743I-C to control 8 MOSFET outputs as independent PWM channels.
- Implemented using existing STM32H7 timer/PWM infrastructure so each channel can have independent PWM frequency.

Build integration:
- New plugin source: plugins/h743_mosfet_pwm.c
- Plugin init hook added in grbl/plugins_init.h
- Enabled for Waveshare env via `-D H743_MOSFET_PWM_ENABLE=1`

Current M-code interface:
- `M1010 P<channel> Q<duty_percent>` set duty (0-100%).
- `M1010 P<channel>` report one channel.
- `M1011 P<channel> F<freq_hz>` set channel frequency (1-100000 Hz).
- `M1012` report all channels.
- `M1012 P<channel>` report one channel.

Channel mapping:
- Uses AUXOUTPUT0..AUXOUTPUT7 pin definitions from board map.

Notes/caveats:
- Plugin currently claims the mapped digital output ports at startup.
- If these pins are also assigned to spindle/coolant in a given config, claim conflicts can occur; board map cleanup is needed before upstream PR.
