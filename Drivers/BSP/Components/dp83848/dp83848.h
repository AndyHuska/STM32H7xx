/**
  ******************************************************************************
  * @file    dp83848.h
  * @brief   Minimal DP83848 PHY driver interface compatible with ST BSP style.
  ******************************************************************************
  */

#ifndef DP83848_H
#define DP83848_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Register map */
#define DP83848_BCR      ((uint16_t)0x0000U)
#define DP83848_BSR      ((uint16_t)0x0001U)
#define DP83848_SMR      ((uint16_t)0x0019U)
#define DP83848_ISFR     ((uint16_t)0x0012U)
#define DP83848_IMR      ((uint16_t)0x0011U)
#define DP83848_PHYSCSR  ((uint16_t)0x0010U)

/* BCR bits */
#define DP83848_BCR_SOFT_RESET         ((uint16_t)0x8000U)
#define DP83848_BCR_LOOPBACK           ((uint16_t)0x4000U)
#define DP83848_BCR_SPEED_SELECT       ((uint16_t)0x2000U)
#define DP83848_BCR_AUTONEGO_EN        ((uint16_t)0x1000U)
#define DP83848_BCR_POWER_DOWN         ((uint16_t)0x0800U)
#define DP83848_BCR_RESTART_AUTONEGO   ((uint16_t)0x0200U)
#define DP83848_BCR_DUPLEX_MODE        ((uint16_t)0x0100U)

/* BSR bits */
#define DP83848_BSR_AUTONEGO_CPLT      ((uint16_t)0x0020U)
#define DP83848_BSR_LINK_STATUS        ((uint16_t)0x0004U)

/* SMR bits */
#define DP83848_SMR_PHY_ADDR           ((uint16_t)0x001FU)

/* PHYSCSR bits */
#define DP83848_PHYSCSR_AUTONEGO_DONE  ((uint16_t)0x0100U)
#define DP83848_PHYSCSR_HCDSPEEDMASK   ((uint16_t)0x0006U)
#define DP83848_PHYSCSR_10BT_HD        ((uint16_t)0x0002U)
#define DP83848_PHYSCSR_10BT_FD        ((uint16_t)0x0006U)
#define DP83848_PHYSCSR_100BTX_HD      ((uint16_t)0x0000U)
#define DP83848_PHYSCSR_100BTX_FD      ((uint16_t)0x0004U)

/* Status */
#define DP83848_STATUS_READ_ERROR            ((int32_t)-5)
#define DP83848_STATUS_WRITE_ERROR           ((int32_t)-4)
#define DP83848_STATUS_ADDRESS_ERROR         ((int32_t)-3)
#define DP83848_STATUS_RESET_TIMEOUT         ((int32_t)-2)
#define DP83848_STATUS_ERROR                 ((int32_t)-1)
#define DP83848_STATUS_OK                    ((int32_t) 0)
#define DP83848_STATUS_LINK_DOWN             ((int32_t) 1)
#define DP83848_STATUS_100MBITS_FULLDUPLEX  ((int32_t) 2)
#define DP83848_STATUS_100MBITS_HALFDUPLEX  ((int32_t) 3)
#define DP83848_STATUS_10MBITS_FULLDUPLEX   ((int32_t) 4)
#define DP83848_STATUS_10MBITS_HALFDUPLEX   ((int32_t) 5)
#define DP83848_STATUS_AUTONEGO_NOTDONE     ((int32_t) 6)

/* IRQ bits */
#define DP83848_INT_8       ((uint16_t)0x0100U)
#define DP83848_INT_7       ((uint16_t)0x0080U)
#define DP83848_INT_6       ((uint16_t)0x0040U)
#define DP83848_INT_5       ((uint16_t)0x0020U)
#define DP83848_INT_4       ((uint16_t)0x0010U)
#define DP83848_INT_3       ((uint16_t)0x0008U)
#define DP83848_INT_2       ((uint16_t)0x0004U)
#define DP83848_INT_1       ((uint16_t)0x0002U)

#define DP83848_WOL_IT                        DP83848_INT_8
#define DP83848_ENERGYON_IT                   DP83848_INT_7
#define DP83848_AUTONEGO_COMPLETE_IT          DP83848_INT_6
#define DP83848_REMOTE_FAULT_IT               DP83848_INT_5
#define DP83848_LINK_DOWN_IT                  DP83848_INT_4
#define DP83848_AUTONEGO_LP_ACK_IT            DP83848_INT_3
#define DP83848_PARALLEL_DETECTION_FAULT_IT   DP83848_INT_2
#define DP83848_AUTONEGO_PAGE_RECEIVED_IT     DP83848_INT_1

typedef int32_t (*dp83848_Init_Func)(void);
typedef int32_t (*dp83848_DeInit_Func)(void);
typedef int32_t (*dp83848_ReadReg_Func)(uint32_t, uint32_t, uint32_t *);
typedef int32_t (*dp83848_WriteReg_Func)(uint32_t, uint32_t, uint32_t);
typedef int32_t (*dp83848_GetTick_Func)(void);

typedef struct {
    dp83848_Init_Func Init;
    dp83848_DeInit_Func DeInit;
    dp83848_WriteReg_Func WriteReg;
    dp83848_ReadReg_Func ReadReg;
    dp83848_GetTick_Func GetTick;
} dp83848_IOCtx_t;

typedef struct {
    uint32_t DevAddr;
    uint32_t Is_Initialized;
    dp83848_IOCtx_t IO;
    void *pData;
} dp83848_Object_t;

int32_t DP83848_RegisterBusIO(dp83848_Object_t *pObj, dp83848_IOCtx_t *ioctx);
int32_t DP83848_Init(dp83848_Object_t *pObj);
int32_t DP83848_DeInit(dp83848_Object_t *pObj);
int32_t DP83848_DisablePowerDownMode(dp83848_Object_t *pObj);
int32_t DP83848_EnablePowerDownMode(dp83848_Object_t *pObj);
int32_t DP83848_StartAutoNego(dp83848_Object_t *pObj);
int32_t DP83848_GetLinkState(dp83848_Object_t *pObj);
int32_t DP83848_SetLinkState(dp83848_Object_t *pObj, uint32_t LinkState);
int32_t DP83848_EnableLoopbackMode(dp83848_Object_t *pObj);
int32_t DP83848_DisableLoopbackMode(dp83848_Object_t *pObj);
int32_t DP83848_EnableIT(dp83848_Object_t *pObj, uint32_t Interrupt);
int32_t DP83848_DisableIT(dp83848_Object_t *pObj, uint32_t Interrupt);
int32_t DP83848_ClearIT(dp83848_Object_t *pObj, uint32_t Interrupt);
int32_t DP83848_GetITStatus(dp83848_Object_t *pObj, uint32_t Interrupt);

#ifdef __cplusplus
}
#endif

#endif
