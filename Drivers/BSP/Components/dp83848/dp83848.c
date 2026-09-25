/*
  ******************************************************************************
  * @file    dp83848.c
  * @brief   DP83848 PHY driver (ST BSP-compatible implementation style).
  ******************************************************************************
  */

#include "dp83848.h"

#define DP83848_SW_RESET_TO    ((uint32_t)500U)
#define DP83848_INIT_TO        ((uint32_t)2000U)
#define DP83848_MAX_DEV_ADDR   ((uint32_t)31U)

int32_t DP83848_RegisterBusIO(dp83848_Object_t *pObj, dp83848_IOCtx_t *ioctx)
{
    if(!pObj || !ioctx || !ioctx->ReadReg || !ioctx->WriteReg || !ioctx->GetTick)
        return DP83848_STATUS_ERROR;

    pObj->IO.Init = ioctx->Init;
    pObj->IO.DeInit = ioctx->DeInit;
    pObj->IO.ReadReg = ioctx->ReadReg;
    pObj->IO.WriteReg = ioctx->WriteReg;
    pObj->IO.GetTick = ioctx->GetTick;

    return DP83848_STATUS_OK;
}

int32_t DP83848_Init(dp83848_Object_t *pObj)
{
    uint32_t tickstart = 0, regvalue = 0, addr;
    int32_t status = DP83848_STATUS_OK;

    if(pObj->Is_Initialized == 0U) {

        if(pObj->IO.Init != 0)
            pObj->IO.Init();

        pObj->DevAddr = DP83848_MAX_DEV_ADDR + 1U;

        for(addr = 0; addr <= DP83848_MAX_DEV_ADDR; addr++) {

            if(pObj->IO.ReadReg(addr, DP83848_SMR, &regvalue) < 0) {
                status = DP83848_STATUS_READ_ERROR;
                continue;
            }

            if((regvalue & DP83848_SMR_PHY_ADDR) == addr) {
                pObj->DevAddr = addr;
                status = DP83848_STATUS_OK;
                break;
            }
        }

        if(pObj->DevAddr > DP83848_MAX_DEV_ADDR)
            status = DP83848_STATUS_ADDRESS_ERROR;

        if(status == DP83848_STATUS_OK) {

            if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, DP83848_BCR_SOFT_RESET) >= 0) {

                if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &regvalue) >= 0) {

                    tickstart = pObj->IO.GetTick();

                    while((regvalue & DP83848_BCR_SOFT_RESET) != 0U) {

                        if((pObj->IO.GetTick() - tickstart) <= DP83848_SW_RESET_TO) {

                            if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &regvalue) < 0) {
                                status = DP83848_STATUS_READ_ERROR;
                                break;
                            }

                        } else {
                            status = DP83848_STATUS_RESET_TIMEOUT;
                            break;
                        }
                    }

                } else
                    status = DP83848_STATUS_READ_ERROR;

            } else
                status = DP83848_STATUS_WRITE_ERROR;
        }
    }

    if(status == DP83848_STATUS_OK) {
        tickstart = pObj->IO.GetTick();
        while((pObj->IO.GetTick() - tickstart) <= DP83848_INIT_TO) {
        }
        pObj->Is_Initialized = 1U;
    }

    return status;
}

int32_t DP83848_DeInit(dp83848_Object_t *pObj)
{
    if(pObj->Is_Initialized) {
        if(pObj->IO.DeInit != 0 && pObj->IO.DeInit() < 0)
            return DP83848_STATUS_ERROR;
        pObj->Is_Initialized = 0U;
    }

    return DP83848_STATUS_OK;
}

int32_t DP83848_DisablePowerDownMode(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval &= ~DP83848_BCR_POWER_DOWN;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_EnablePowerDownMode(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval |= DP83848_BCR_POWER_DOWN;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_StartAutoNego(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval |= DP83848_BCR_AUTONEGO_EN;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_GetLinkState(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BSR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BSR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    if((readval & DP83848_BSR_LINK_STATUS) == 0U)
        return DP83848_STATUS_LINK_DOWN;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    if((readval & DP83848_BCR_AUTONEGO_EN) != DP83848_BCR_AUTONEGO_EN) {

        if(((readval & DP83848_BCR_SPEED_SELECT) == DP83848_BCR_SPEED_SELECT) && ((readval & DP83848_BCR_DUPLEX_MODE) == DP83848_BCR_DUPLEX_MODE))
            return DP83848_STATUS_100MBITS_FULLDUPLEX;
        else if((readval & DP83848_BCR_SPEED_SELECT) == DP83848_BCR_SPEED_SELECT)
            return DP83848_STATUS_100MBITS_HALFDUPLEX;
        else if((readval & DP83848_BCR_DUPLEX_MODE) == DP83848_BCR_DUPLEX_MODE)
            return DP83848_STATUS_10MBITS_FULLDUPLEX;
        else
            return DP83848_STATUS_10MBITS_HALFDUPLEX;

    } else {

        if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_PHYSCSR, &readval) < 0)
            return DP83848_STATUS_READ_ERROR;

        if((readval & DP83848_PHYSCSR_AUTONEGO_DONE) == 0U)
            return DP83848_STATUS_AUTONEGO_NOTDONE;

        if((readval & DP83848_PHYSCSR_HCDSPEEDMASK) == DP83848_PHYSCSR_100BTX_FD)
            return DP83848_STATUS_100MBITS_FULLDUPLEX;
        else if((readval & DP83848_PHYSCSR_HCDSPEEDMASK) == DP83848_PHYSCSR_100BTX_HD)
            return DP83848_STATUS_100MBITS_HALFDUPLEX;
        else if((readval & DP83848_PHYSCSR_HCDSPEEDMASK) == DP83848_PHYSCSR_10BT_FD)
            return DP83848_STATUS_10MBITS_FULLDUPLEX;
        else
            return DP83848_STATUS_10MBITS_HALFDUPLEX;
    }
}

int32_t DP83848_SetLinkState(dp83848_Object_t *pObj, uint32_t LinkState)
{
    uint32_t bcrvalue = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &bcrvalue) < 0)
        return DP83848_STATUS_READ_ERROR;

    bcrvalue &= ~(DP83848_BCR_AUTONEGO_EN | DP83848_BCR_SPEED_SELECT | DP83848_BCR_DUPLEX_MODE);

    if(LinkState == DP83848_STATUS_100MBITS_FULLDUPLEX)
        bcrvalue |= (DP83848_BCR_SPEED_SELECT | DP83848_BCR_DUPLEX_MODE);
    else if(LinkState == DP83848_STATUS_100MBITS_HALFDUPLEX)
        bcrvalue |= DP83848_BCR_SPEED_SELECT;
    else if(LinkState == DP83848_STATUS_10MBITS_FULLDUPLEX)
        bcrvalue |= DP83848_BCR_DUPLEX_MODE;
    else if(LinkState != DP83848_STATUS_10MBITS_HALFDUPLEX)
        return DP83848_STATUS_ERROR;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, bcrvalue) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_EnableLoopbackMode(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval |= DP83848_BCR_LOOPBACK;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_DisableLoopbackMode(dp83848_Object_t *pObj)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_BCR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval &= ~DP83848_BCR_LOOPBACK;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_BCR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_EnableIT(dp83848_Object_t *pObj, uint32_t Interrupt)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_IMR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval |= Interrupt;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_IMR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_DisableIT(dp83848_Object_t *pObj, uint32_t Interrupt)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_IMR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    readval &= ~Interrupt;

    if(pObj->IO.WriteReg(pObj->DevAddr, DP83848_IMR, readval) < 0)
        return DP83848_STATUS_WRITE_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_ClearIT(dp83848_Object_t *pObj, uint32_t Interrupt)
{
    uint32_t readval = 0;
    (void)Interrupt;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_ISFR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    return DP83848_STATUS_OK;
}

int32_t DP83848_GetITStatus(dp83848_Object_t *pObj, uint32_t Interrupt)
{
    uint32_t readval = 0;

    if(pObj->IO.ReadReg(pObj->DevAddr, DP83848_ISFR, &readval) < 0)
        return DP83848_STATUS_READ_ERROR;

    return ((readval & Interrupt) == Interrupt) ? 1 : 0;
}
