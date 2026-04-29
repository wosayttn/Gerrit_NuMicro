/**************************************************************************//**
 * @file     utcpd.c
 * @version  V1.00
 * @brief    NPD48 series UTCPD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif


uint8_t I2C_ADDR[4] = {0x60, 0x61, 0x70, 0x71};

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UTCPD_Driver UTCPD Driver
  @{
*/

/** @addtogroup UTCPD_EXPORTED_FUNCTIONS UTCPD Exported Functions
  @{
*/

uint16_t u16Dac5VTrim;
uint16_t u16Dac48VTrim;

void UTCPD_GetTrimDACData(int port, uint16_t* pu16Trim5VData, uint16_t* pu16Trim48VData)
{
    if(port == 0)
    {
        *pu16Trim5VData = u16Dac5VTrim;
        *pu16Trim48VData = u16Dac48VTrim;
    }
}


/**
  * @brief      Enable specify UTCPD Controller and set Clock Divider
  *
  * @param[in]  UTCPD         Specify UTCPD port
  * @return     Always 0
  *
  * @details    The function enable the specify I2C Controller and set proper Clock Divider
  *             in I2C CLOCK DIVIDED REGISTER (I2CLK) according to the target I2C Bus clock.
  *             I2C Bus clock = PCLK / (4*(divider+1).
  *
  */
uint32_t UTCPD_Open(int port)
{
    /* Enable Clock */
    /* Set Clock Divider */
    /* Enable PHY */
    uint16_t u16HighNibble;
    int32_t i32Val;


    tcpc_addr_read16(port, I2C_ADDR[port], 0, &i32Val);

#if 0
    __DBG_PRINTF("ID = 0x%x\n", i32Val);
    {
        int32_t mask_clr, mask_set;
        //(int port, int i2c_addr, int reg, int *val)
        tcpc_addr_read16(port, I2C_ADDR[port], NPD48_ALERTML, &i32Val);
        DBG_PRINTF("Original ALERT = 0x%x\n", i32Val);

        mask_clr = 0x100;
        tcpc_update16(port, NPD48_ALERTML, mask_clr, MASK_CLR);

        tcpc_addr_read16(port, I2C_ADDR[port], NPD48_ALERTML, &i32Val);
        __DBG_PRINTF("ALERT = 0x%x after clear\n", i32Val);

        mask_set = 0x100;
        tcpc_update16(port, NPD48_ALERTML, mask_set, MASK_SET);

        tcpc_addr_read16(port, I2C_ADDR[port], NPD48_ALERTML, &i32Val);
        __DBG_PRINTF("ALERT = 0x%x after set\n", i32Val);
    }
#endif

    /* PHY Enable */
    i2c_write8(port, NULL, NPD48_AFEPWRSTE, NPD48_AFEPWRSTE_PHYPWR_Msk);
    i2c_update8(port, NULL, NPD48_AFEPWRSTE, NPD48_AFEPWRSTE_AFEPWR_Msk, MASK_SET);
    /* Charge Pump Enable */
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);

    /* Gate Drive Debug Mode Enable  */
    i2c_update8(port, NULL, NPD48_GDDBGCR, NPD48_PWRCR_GDCPEN_Msk, MASK_SET);

#if 0
{
		int i;
		for(i=0; i<8; i=i+1)
		{
				i2c_read8(port, NULL, NPD48_TRIMDB0+i, &i32Val);
				DBG_PRINTF("TRIM%d = 0x%x\n", i, i32Val);
		}
}	
#endif
#if 1
    i2c_read8(port, NULL, NPD48_TRIMDB3, &i32Val);
    u16Dac5VTrim = i32Val;

    i2c_read8(port, NULL, NPD48_TRIMDB5, &i32Val);
    u16Dac48VTrim = i32Val<<4;

    i2c_read8(port, NULL, NPD48_TRIMDB4, &i32Val);



    u16Dac5VTrim =  ((i32Val&0x0F)<<8) | u16Dac5VTrim;
    u16Dac48VTrim =  ((i32Val&0xF0)>>4) | u16Dac48VTrim;
//    DBG_PRINTF("TRIM 5V  = %d\n", u16Dac5VTrim);
//    DBG_PRINTF("TRIM 48V  = %d\n", u16Dac48VTrim);
#endif

    return 0;
}

/**
  * @brief      Disan;e specify UTCPD Controller
  *
  * @param[in]  UTCPD         Specify UTCPD port
  * @return     Always 0
  *
  * @details    The function enable the specify I2C Controller and set proper Clock Divider
  *             in I2C CLOCK DIVIDED REGISTER (I2CLK) according to the target I2C Bus clock.
  *             I2C Bus clock = PCLK / (4*(divider+1).
  *
  */
uint32_t UTCPD_Close(int port)
{
    /* PHY Disable */
    i2c_update8(port, NULL, NPD48_AFEPWRSTE, NPD48_AFEPWRSTE_PHYPWR_Msk, MASK_CLR);
    i2c_update8(port, NULL, NPD48_AFEPWRSTE, NPD48_AFEPWRSTE_AFEPWR_Msk, MASK_CLR);

    /* Charge Pump Enable */
    i2c_update8(port, NULL, NPD48_PWRCR, NPD48_PWRCR_GDCPEN_Msk, MASK_CLR);

    /* Disable Clock */

    /* Reset UTCPD */

    return 0;
}
/**
  * @brief      Clear UTCPD Alert Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  AlertStClr     Alert status clear
  *             - \ref UTCPD_ALERT_CCSCHIS
  *             - \ref UTCPD_ALERT_PWRSCHIS
  *             - \ref UTCPD_ALERT_RXSOPIS
  *             - \ref UTCPD_ALERT_RXHRSTIS
  *             - \ref UTCPD_ALERT_TXFAILIS
  *             - \ref UTCPD_ALERT_TXDCUIS
  *             - \ref UTCPD_ALERT_TXOKIS
  *             - \ref UTCPD_ALERT_VBAMHIS
  *             - \ref UTCPD_ALERT_VBAMLIS
  *             - \ref UTCPD_ALERT_FUTIS
  *             - \ref UTCPD_ALERT_RXOFIS
  *             - \ref UTCPD_ALERT_SKDCDTIS
  *             - \ref UTCPD_ALERT_VNDIS
  * @return     0: Successful,  1: Fail
  *
  * @details    Write One Clear
  *
  */
int32_t UTCPD_GetAlertStatus(int port, int* i32AlertSts)
{
    return tcpc_read16(port, NPD48_ALERTL, i32AlertSts);
}

/**
  * @brief      Get UTCPD Alert Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  AlertStClr     Alert status clear
  *             - \ref UTCPD_ALERT_CCSCHIS
  *             - \ref UTCPD_ALERT_PWRSCHIS
  *             - \ref UTCPD_ALERT_RXSOPIS
  *             - \ref UTCPD_ALERT_RXHRSTIS
  *             - \ref UTCPD_ALERT_TXFAILIS
  *             - \ref UTCPD_ALERT_TXDCUIS
  *             - \ref UTCPD_ALERT_TXOKIS
  *             - \ref UTCPD_ALERT_VBAMHIS
  *             - \ref UTCPD_ALERT_VBAMLIS
  *             - \ref UTCPD_ALERT_FUTIS
  *             - \ref UTCPD_ALERT_RXOFIS
  *             - \ref UTCPD_ALERT_SKDCDTIS
  *             - \ref UTCPD_ALERT_VNDIS
  * @return     0: Successful,  1: Fail
  *
  * @details    Write One Clear
  *
  */
int32_t UTCPD_ClearAlertStatus(int port, int AlertStClr)
{
    return tcpc_update16(port, NPD48_ALERTL, AlertStClr, MASK_SET);
}

/**
  * @brief      Enable UTCPD Alert Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Alert bit mask
  *             - \ref UTCPD_ALERTM_CCSCHIE
  *             - \ref UTCPD_ALERTM_PWRSCHIE
  *             - \ref UTCPD_ALERTM_RXSOPIE
  *             - \ref UTCPD_ALERTM_RXHRSTIE
  *             - \ref UTCPD_ALERTM_TXFAILIE
  *             - \ref UTCPD_ALERTM_TXDCUIE
  *             - \ref UTCPD_ALERTM_TXOKIE
  *             - \ref UTCPD_ALERTM_VBAMHIE
  *             - \ref UTCPD_ALERTM_VBAMLIE
  *             - \ref UTCPD_ALERTM_FUTIE
  *             - \ref UTCPD_ALERTM_RXOFIE
  *             - \ref UTCPD_ALERTM_SKDCDTIE
  *             - \ref UTCPD_ALERTM_VNDIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableAlertMask(int port, int mask_set)
{
    return tcpc_update16(port, NPD48_ALERTML, mask_set, MASK_SET);
}


/**
  * @brief      Disable UTCPD Alert Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     Alert bit mask
  *             - \ref UTCPD_ALERTM_CCSCHIE
  *             - \ref UTCPD_ALERTM_PWRSCHIE
  *             - \ref UTCPD_ALERTM_RXSOPIE
  *             - \ref UTCPD_ALERTM_RXHRSTIE
  *             - \ref UTCPD_ALERTM_TXFAILIE
  *             - \ref UTCPD_ALERTM_TXDCUIE
  *             - \ref UTCPD_ALERTM_TXOKIE
  *             - \ref UTCPD_ALERTM_VBAMHIE
  *             - \ref UTCPD_ALERTM_VBAMLIE
  *             - \ref UTCPD_ALERTM_FUTIE
  *             - \ref UTCPD_ALERTM_RXOFIE
  *             - \ref UTCPD_ALERTM_SKDCDTIE
  *             - \ref UTCPD_ALERTM_VNDIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableAlertMask(int port, int mask_clr)
{
    return tcpc_update16(port, NPD48_ALERTML, mask_clr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Power Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Power status bit mask
  *             - \ref NPD48_PWRSM_SKVBIE
  *             - \ref NPD48_PWRSM_VCPSIE
  *             - \ref NPD48_PWRSM_VBPSIE
  *             - \ref NPD48_PWRSM_VBDTDGIE
  *             - \ref NPD48_PWRSM_SRVBIE
  *             - \ref NPD48_PWRSM_SRHVIE
  *             - \ref NPD48_PWRSM_DACONIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnablePowerStatusMask(int port, int mask_set)
{
    return tcpc_update8(port, NPD48_PWRSM, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Power Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     Power bit mask
  *             - \ref UTCPD_PWRSM_SKVBIE
  *             - \ref UTCPD_PWRSM_VCPSIE
  *             - \ref UTCPD_PWRSM_VBPSIE
  *             - \ref UTCPD_PWRSM_VBDTDGIE
  *             - \ref UTCPD_PWRSM_SRVBIE
  *             - \ref UTCPD_PWRSM_SRHVIE
  *             - \ref UTCPD_PWRSM_DACONIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
    *
  */
int32_t UTCPD_DisablePowerStatusMask(int port, int mask_clr)
{
    return tcpc_update8(port, NPD48_PWRSM, mask_clr, MASK_CLR);
}


/**
  * @brief      Enable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault Status bit mask
  *             - \ref UTCPD_FAULTSM_VCOCIE
  *             - \ref UTCPD_FAULTSM_VBOVIE
  *             - \ref UTCPD_FAULTSM_VBOCIE
  *             - \ref UTCPD_FAULTSM_FDGFALIE
  *             - \ref UTCPD_FAULTSM_ADGFALIE
  *             - \ref UTCPD_FAULTSM_FOFFVBIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableFaultMask(int port, int mask_set)
{
    return tcpc_update8(port, NPD48_FAULTSM, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     Fault Status bit mask
  *             - \ref UTCPD_FAULTSM_VCOCIE
  *             - \ref UTCPD_FAULTSM_VBOVIE
  *             - \ref UTCPD_FAULTSM_VBOCIE
  *             - \ref UTCPD_FAULTSM_FDGFALIE
  *             - \ref UTCPD_FAULTSM_ADGFALIE
  *             - \ref UTCPD_FAULTSM_FOFFVBIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableFaultMask(int port, int mask_clr)
{
    return tcpc_update8(port, NPD48_FAULTSM, mask_clr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Power Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Power Control bit mask
  *             - \ref UTCPD_PWRCTL_VCEN
  *             - \ref UTCPD_PWRCTL_VCPWR
  *             - \ref UTCPD_PWRCTL_FDGEN
  *             - \ref UTCPD_PWRCTL_BDGEN
  *             - \ref UTCPD_PWRCTL_ADGDC
  *             - \ref UTCPD_PWRCTL_DSVBAM_DIS
  *             - \ref UTCPD_PWRCTL_VBMONI_DIS
  *
  * @return     None
  *
  * @details    Remember that VBUS Monitor and VBUS Alarm are "0" enabled. And both are "1" disabled
  *
  */
int32_t UTCPD_EnablePowerCtrl(int port, uint32_t mask_set)
{
    return tcpc_update8(port, NPD48_PWRCTL, mask_set, MASK_SET);	/* VBUS Monitor and VBUS Alarm are 0 to enable */
}

/**
  * @brief      Disable UTCPD Power Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Power Control bit mask
  *             - \ref UTCPD_PWRCTL_VCEN
  *             - \ref UTCPD_PWRCTL_VCPWR
  *             - \ref UTCPD_PWRCTL_FDGEN
  *             - \ref UTCPD_PWRCTL_BDGEN
  *             - \ref UTCPD_PWRCTL_ADGDC
  *             - \ref UTCPD_PWRCTL_DSVBAM_DIS
  *             - \ref UTCPD_PWRCTL_VBMONI_DIS
  * @return     None
  *
  * @details    Remember that VBUS Monitor and VBUS Alarm are "0" enabled. And both are "1" disabled
  *
  */
int32_t UTCPD_DisablePowerCtrl(int port, uint32_t maskclr)
{
    return tcpc_update8(port, NPD48_PWRCTL, maskclr, MASK_CLR);	/* VBUS Monitor and VBUS Alarm are 1 to disable */
}


/**
  * @brief      Enable UTCPD Fault Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault bit mask
  *             - \ref UTCPD_FAULTCTL_VCOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBOVDTDS
  *             - \ref UTCPD_FAULTCTL_VBOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBDGTMDS
  *             - \ref UTCPD_FAULTCTL_FOFFVBDS
  *
  * @return     None
  *
  * @details    When Time-out flag will be set, use this function to clear I2C Bus Time-out counter flag .
  *
  */
int32_t UTCPD_EnableFaultCtrl(int port, uint32_t mask_set)
{
    return tcpc_update8(port, NPD48_FAULTCTL, mask_set, MASK_CLR);	/* 0 to enable */
}

/**
  * @brief      Disable UTCPD Fault Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault bit mask
  *
  * @return     None
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableFaultCtrl(int port, uint32_t maskclr)
{
    return tcpc_update8(port, NPD48_FAULTCTL, maskclr, MASK_SET);	/* 1 to disable */
}



/**
  * @brief      Clear UTCPD Power Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  PowerStClr   Power status clear
  *             - \ref UTCPD_PWRSTS_SKVB
  *             - \ref UTCPD_PWRSTS_VCPS
  *             - \ref UTCPD_PWRSTS_VBPS
  *             - \ref UTCPD_PWRSTS_VBPSDTEN
  *             - \ref UTCPD_PWRSTS_SRVB
  *             - \ref UTCPD_PWRSTS_SRHV
  *             - \ref UTCPD_PWRSTS_DACON
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_ClearPowerStatus(int port, int PowerStClr)
{
    return tcpc_update16(port, NPD48_PWRSTS, PowerStClr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault status bit mask
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableFaultStatusMask(int port, int mask_set)
{
    return tcpc_update8(port, NPD48_FAULTSM, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     Fault bit mask
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_DisableFaultStatusMask(int port, int mask_clr)
{
    return tcpc_update8(port, NPD48_FAULTSM, mask_clr, MASK_CLR);
}


int32_t UTCPD_GetFaultStatus(int port, int32_t* pi32RegData)
{
    return tcpc_addr_read(0, NULL, NPD48_FAULTSTS, pi32RegData);
}

/**
  * @brief      Clear UTCPD Fault Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  FaultStClr   Fault status clear
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_ClearFaultStatus(int port, int FaultStClr)
{
    return tcpc_update16(port, NPD48_FAULTSTS, FaultStClr, MASK_CLR);
}

/**
  * @brief      Set Role Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32DrpToggle DRP toggling
  *             - \ref UTCPD_ROLECTL_DRP
  *             - \ref NULL,
  * @param[in]  u32RpValue   Rp pull up value
  *             - \ref UTCPD_ROLECTL_RPVALUE_DEF
  *             - \ref UTCPD_ROLECTL_RPVALUE_1P5A
  *             - \ref UTCPD_ROLECTL_RPVALUE_3A
  * @param[in]  u32CC2       Force CC2 state
  *             - \ref UTCPD_ROLECTL_CC2_RA
  *             - \ref UTCPD_ROLECTL_CC2_RP
  *             - \ref UTCPD_ROLECTL_CC2_RD
  *             - \ref UTCPD_ROLECTL_CC2_OPEN
  * @param[in]  u32CC1       Force CC1 state
  *             - \ref UTCPD_ROLECTL_CC1_RA
  *             - \ref UTCPD_ROLECTL_CC1_RP
  *             - \ref UTCPD_ROLECTL_CC1_RD
  *             - \ref UTCPD_ROLECTL_CC1_OPEN
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetRoleCtrl(int port, uint32_t u32DrpToggle, uint32_t u32Rpvalue, uint32_t u32CC2, uint32_t u32CC1)
{
    uint32_t reg_set = u32DrpToggle | u32Rpvalue | u32CC2 | u32CC1;
    return tcpc_addr_write(port, NULL, NPD48_ROLECTL, reg_set);
}

/**
  * @brief      Get Role Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32DrpToggle DRP toggling
  *             - \ref UTCPD_ROLECTL_DRP
  *             - \ref NULL,
  * @param[in]  u32RpValue   Rp pull up value
  *             - \ref UTCPD_ROLECTL_RPVALUE_DEF
  *             - \ref UTCPD_ROLECTL_RPVALUE_1P5A
  *             - \ref UTCPD_ROLECTL_RPVALUE_3A
  * @param[in]  u32CC2       Force CC2 state
  *             - \ref UTCPD_ROLECTL_CC2_RA
  *             - \ref UTCPD_ROLECTL_CC2_RP
  *             - \ref UTCPD_ROLECTL_CC2_RD
  *             - \ref UTCPD_ROLECTL_CC2_OPEN
  * @param[in]  u32CC1       Force CC1 state
  *             - \ref UTCPD_ROLECTL_CC1_RA
  *             - \ref UTCPD_ROLECTL_CC1_RP
  *             - \ref UTCPD_ROLECTL_CC1_RD
  *             - \ref UTCPD_ROLECTL_CC1_OPEN
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetRoleCtrl(int port, uint32_t* pu32DrpToggle, uint32_t* pu32CC1, uint32_t* pu32CC2, uint32_t* pu32Rpvalue)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_ROLECTL, &i32Reg);
    *pu32DrpToggle = i32Reg & NPD48_ROLECTL_DRP_Msk;
    *pu32Rpvalue = i32Reg & NPD48_ROLECTL_RPVALUE_Msk;
    *pu32CC2 = i32Reg & NPD48_ROLECTL_CC2_Msk;
    *pu32CC1 = i32Reg & NPD48_ROLECTL_CC1_Msk;
    return rv;
}

/**
  * @brief      Set Type-C Port Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32BistMode  Enable or Disable BIST Mode
  *             - \ref UTCPD_TCPCCTL_BISTEN or 0
  * @param[in]  u32Orient    Plug Oritentation
  *             - \ref UTCPD_TCPCCTL_ORIENT or 0
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetTypeCPortCtrl(int port, uint32_t u32BistMode, uint32_t u32Orient)
{
    uint8_t reg_set = u32BistMode | u32Orient;
    return tcpc_addr_write(port, NULL, NPD48_TCPCCTL, reg_set);
}


/**
  * @brief      Get Role Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32BistMode  Enable or Disable BIST Mode
  *             - \ref UTCPD_TCPCCTL_BISTEN or 0
  * @param[in]  u32Orient    Plug Oritentation
  *             - \ref UTCPD_TCPCCTL_ORIENT or 0
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetTypeCPortCtrl(int port, uint32_t* pu32BistMode, uint32_t* pu32Orient)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_TCPCCTL, &i32Reg);
    *pu32BistMode = i32Reg & NPD48_TCPCCTL_BISTEN_Msk;
    *pu32Orient = i32Reg & NPD48_TCPCCTL_ORIENT_Msk;
    return rv;
}
/**
  * @brief      UTCPD Issue Command
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u8TcpcCtl    Type-C Port Control
  *             - \ref UTCPD_CMD_DISABLE_VBUS_DETECT
  *             - \ref UTCPD_CMD_ENABLE_VBUS_DETECT
  *             - \ref UTCPD_CMD_DISABLE_SINK_VBUS
  *             - \ref UTCPD_CMD_SINK_VBUS
  *             - \ref UTCPD_CMD_SRC_VBUS_DEFAULT
  *             - \ref UTCPD_CMD_SRC_VBUS_NONDEFAULT
  *             - \ref UTCPD_CMD_LOOK4CONNECTION
  * @return     0: Successful,  1: Fail
  *
  * @details    The command can't be read back.
  *
  */
int32_t UTCPD_IssueCmd(int port, uint32_t cmd)
{

    return tcpc_addr_write(port, NULL, NPD48_CMD, cmd);
}



/**
  * @brief      Get CC Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32Look4Con.             Under Look for connection or done
  *             - \ref UTCPD_LK4CONN_DONE
  *             - \ref UTCPD_UNDER_LK4CONN
  * @param[in]  pu32ConRlt.               Connection result
  *             - \ref UTCPD_CONN_RESULT_RP
  *             - \ref UTCPD_CONN_RESULT_RD
  * @param[in]  pu32CC2Sts is CC2_State.  CC2 connection state depends on connection result
  * |\ref             |\ref UTCPD_CCSTS_CC2STATE_SRC_OPEN    | \ref UTCPD_CCSTS_CC2STATE_SNK_OPEN                       |
  * |\ref             |\ref UTCPD_CCSTS_CC2STATE_SRC_RA      | \ref UTCPD_CCSTS_CC2STATE_SNK_DEF                        |
  * |\ref             |\ref UTCPD_CCSTS_CC2STATE_SRC_Rd      | \ref UTCPD_CCSTS_CC2STATE_SNK_1P5A                       |
  * |\ref             |\ref x                                | \ref UTCPD_CCSTS_CC2STATE_SNK_3A                         |
  * @param[in]  pu32CC1Sts is CC1_State.   CC1 connection state depends on connection result
  * |\ref             |\ref UTCPD_CCSTS_CC1STATE_SRC_OPEN    | \ref UTCPD_CCSTS_CC1STATE_SNK_OPEN                       |
  * |\ref             |\ref UTCPD_CCSTS_CC1STATE_SRC_RA      | \ref UTCPD_CCSTS_CC1STATE_SNK_DEF                        |
  * |\ref             |\ref UTCPD_CCSTS_CC1STATE_SRC_Rd      | \ref UTCPD_CCSTS_CC1STATE_SNK_1P5A                       |
  * |\ref             |\ref x                                | \ref UTCPD_CCSTS_CC1STATE_SNK_3A                         |
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetCCSts(int port, uint32_t* pu32Look4Con, uint32_t* pu32ConRlt, uint32_t* pu32CC2Sts, uint32_t* pu32CC1Sts)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_CCSTS, &i32Reg);
    *pu32Look4Con = (i32Reg & NPD48_CCSTS_LK4CONN_Msk);
    *pu32ConRlt = (i32Reg & NPD48_CCSTS_CONRLT_Msk);
    *pu32CC2Sts = (i32Reg & NPD48_CCSTS_CC2STATE_Msk);
    *pu32CC1Sts = (i32Reg & NPD48_CCSTS_CC1STATE_Msk);
    return rv;
}


/**
  * @brief      Get Power Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32VBUSDetEn.            pu32VCONNPresent
  *             - \ref UTCPD_PWRSTS_VBPSDTEN
  *             - \ref 0
  * @param[in]  pu32VBUSPresent.          VBUS Present or not
  *             - \ref UTCPD_PWRSTS_VBPS
  *             - \ref 0
  * @param[in]  pu32VCONNPresent          VCONN Present or not
  *             - \ref UTCPD_PWRSTS_VCPS
  *             - \ref 0                      |
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetPwrSts(int port, uint32_t* pu32VBUSDetEn, uint32_t* pu32VBUSPresent, uint32_t* pu32VCONNPresent, uint32_t* pu32SnkVBUS)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_PWRSTS, &i32Reg);
    *pu32VBUSDetEn = i32Reg & NPD48_PWRSTS_VBPSDTEN_Msk;
    *pu32VCONNPresent = i32Reg & NPD48_PWRSTS_VCPS_Msk;
    *pu32VBUSPresent = i32Reg & NPD48_PWRSTS_VBPS_Msk;
    *pu32SnkVBUS = i32Reg & NPD48_PWRSTS_SKVB_Msk;
    return rv;
}

/**
  * @brief      Get Power Status Extention
  *
  * @param[in]  port                      Specify UTCPD port
  * @param[in]  pu32DbgAccessory.         Debug Accessory Mode
  *             - \ref != 0               Debug Accessory Mode
  *             - \ref 0
  * @param[in]  pu32SrcNonDefVBUS.        Source Non-Default VBUS
  *             - \ref != 0               Source Non-Default VBUS
  *             - \ref 0
  * @param[in]  pu32SrcDefVBUS            Source Default VBUS
  *             - \ref != 0               Source Default VBUS
  *             - \ref 0
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetPwrStsExt(int port, uint32_t* pu32DbgAccessory, uint32_t* pu32SrcNonDefVBUS, uint32_t* pu32SrcDefVBUS)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_PWRSTS, &i32Reg);
    *pu32DbgAccessory = i32Reg & NPD48_PWRSTS_DACON_Msk;
    *pu32SrcNonDefVBUS = i32Reg & NPD48_PWRSTS_SRHV_Msk;
    *pu32SrcDefVBUS = i32Reg & NPD48_PWRSTS_SRVB_Msk;
    return rv;
}

/**
  * @brief      Get Power Status
  *
  * @param[in]  port                      Specify UTCPD port
  * @param[in]  pu32VBUSOverVolt.         VBUS Over Voltage Fault
  *             - \ref 1
  *             - \ref 0
  * @param[in]  pu32SrcNonDefVBUS.        VCONN Over Current Fault
  *             - \ref 1
  *             - \ref 0
  * @param[in]  pu32SrcDefVBUS            I2C Interface Error
  *             - \ref 1
  *             - \ref 0
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetFaultSts(int port, uint32_t* pu32VBUSOverCurr, uint32_t* pu32VBUSOverVolt, uint32_t* pu32VCONNOverCurr, uint32_t* pu32I2CInfErr)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_FAULTSTS, &i32Reg);
    *pu32VBUSOverCurr = i32Reg & NPD48_FAULTSTS_VBOCFUT_Msk;
    *pu32VBUSOverVolt = i32Reg & NPD48_FAULTSTS_VBOVFUT_Msk;
    *pu32VCONNOverCurr = i32Reg & NPD48_FAULTSTS_VCOCFUT_Msk;
    *pu32I2CInfErr = i32Reg & NPD48_FAULTSTS_I2CIFERR_Msk;
    return rv;
}


/**
  * @brief      Get Fault Status Extention
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32ForceOffFat.          Force Off VBUS Fault
  *             - \ref 1
  *             - \ref 0
  * @param[in]  pu32AutoDiscFat.          Auto Discharge Fault
  *             - \ref 1
  *             - \ref 0
  * @param[in]  pu32ForceDiscFat          Foece Discharge Fault
  *             - \ref 1
  *             - \ref 0                      |
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetFaultStsExt(int port, uint32_t* pu32ForceOffFat, uint32_t* pu32AutoDiscFat, uint32_t* pu32ForceDiscFat)
{
    int32_t i32Reg;
    int32_t rv;

    rv = tcpc_addr_read(port, NULL, NPD48_FAULTSTS, &i32Reg);
    *pu32ForceOffFat = i32Reg & NPD48_FAULTSTS_FOFFVB_Msk;
    *pu32AutoDiscFat = i32Reg & NPD48_FAULTSTS_ADGFAL_Msk;
    *pu32ForceDiscFat = i32Reg & NPD48_FAULTSTS_FDGFAL_Msk;
    return rv;
}


/**
  * @brief      Set Message Header Information
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32DataRole  Data Role
  *             - \ref UTCPD_MHINFO_DROLE_UFP
  *             - \ref UTCPD_MHINFO_DROLE_DFP
  * @param[in]  u32Revision  PD Revision
  *             - \ref UTCPD_MHINFO_PDREV_10
  *             - \ref UTCPD_MHINFO_PDREV_20
  *             - \ref UTCPD_MHINFO_PDREV_30
  *             - \ref UTCPD_MHINFO_PDREV_31
  * @param[in]  u32PwrRole  Power Role
  *             - \ref UTCPD_MHINFO_PROLE_SNK
  *             - \ref UTCPD_MHINFO_PROLE_SRC
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetMsgHeaderInfo(int port, uint32_t u32DataRole, uint32_t u32Revision, uint32_t u32PwrRole)
{
    uint8_t reg_set = u32DataRole | u32Revision | u32PwrRole;
    return tcpc_addr_write(port, NULL, NPD48_MHINFO, reg_set);
}

/**
  * @brief      Set Message Header Information
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32RegData   Combination from following set
  *             - \ref UTCPD_RDET_SOPEN
  *             - \ref UTCPD_RDET_SOPPEN
  *             - \ref UTCPD_RDET_SOPPPEN
  *             - \ref UTCPD_RDET_SDBGPEN
  *             - \ref UTCPD_RDET_SDBGPEN
  *             - \ref UTCPD_RDET_SDBGPPEN
  *             - \ref UTCPD_RDET_HRSTEN
  *             - \ref UTCPD_RDET_CABRSTEN
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetRecDetect(int port, uint32_t u32RegData)
{
    uint8_t reg_set = u32RegData;
    return tcpc_addr_write(port, NULL, NPD48_RDET, reg_set);
}

/**
  * @brief      Set VBUS Alarm High and Alarm Low Voltage
  *
  * @param[in]  port             Specify UTCPD port
  * @param[in]  u32AlarmH        VBUS Alarm High Threshold Voltage
  * @param[in]  u32AlarmL        VBUS Alarm Low Threshold Voltage
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetVBUSAlarm(int port, uint32_t u32AlarmH, uint32_t u32AlarmL)
{
    uint8_t u8TxBuf[4];
    u8TxBuf[0] = u32AlarmH;
    u8TxBuf[1] = u32AlarmH >> 8;
    u8TxBuf[2] = u32AlarmL;
    u8TxBuf[3] = u32AlarmL >> 8;
    return tcpc_write_block(0, NPD48_VBUSOVTL, u8TxBuf, 4);
}

/**
  * @brief      Set Sink Disconnection Voltage
  *
  * @param[in]  port             Specify UTCPD port
  * @param[in]  u32SnkDiscVolt   Sink Disconnection Threshold Voltae
  * @return     0: Successful,  1: Fail
  *
  * @details    Sink Will start the VBUS auto discharge if VBUS voltage less Sink Disconnection Threshold Level
  *             POWER_CONTROL[Auto Discharge] should set to 1.
  *
  */
int32_t UTCPD_SetSnkDisconnect(int port, uint32_t u32SnkDiscVolt)
{
    return tcpc_addr_write16(port, NULL, NPD48_VBUSDTL, u32SnkDiscVolt);
}

/**
  * @brief      Set VBUS Stop Discharge Threshold Voltage
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32StopDischgVolt   VBUS Stop Discharge Threshold Voltage
  * @return     0: Successful,  1: Fail
  *
  * @details    SRC/SNK Will stop VBUS force discharge if VBUS voltage less VBUS Stop Discharge Threshold Level
  *             POWER_CONTROL[Force Discharge] should set to 1.
  *
  */
int32_t UTCPD_SetStopDischargeVolt(int port, uint32_t u32StopDischgVolt)
{
    return tcpc_addr_write16(port, NULL, NPD48_VBUSTPDTL, u32StopDischgVolt);
}

/**
  * @brief      Set VBUS Present Threshold Voltage
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Threshold        VBUS Present Threshold Voltage
  * @return     0: Successful,  1: Fail
  *
  * @details    VBUS Present Threshold Level
  *
  *
  */
int32_t UTCPD_SetVbusPresentThreshold(int port, uint32_t u32Threshold)
{
    return tcpc_addr_write16(0, NULL, NPD48_VBUSTHL, u32Threshold);
}

/**
  * @brief      Specified the SRC Polarity to Drive External Gate Driver
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Polarity         Specify the Active Polarity of Gate Driver for Source VBUS
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  *
  */
int32_t UTCPD_SrcActivePolarity(int port, uint32_t u32Polarity)
{
    if( u32Polarity == UTCPD_PCACTL_SRCENLVL_LOW )
        return tcpc_update8(port, NPD48_PCACTL, BIT0, MASK_SET);
    else
        return tcpc_update8(port, NPD48_PCACTL, BIT0, MASK_CLR);
}

/**
  * @brief      Specified the SNK Polarity to Drive External Gate Driver
  *_I
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Polarity         Specify the Active Polarity of Gate Driver for Sink VBUS
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  *
  */
int32_t UTCPD_SnkActivePolarity(int port, uint32_t u32Polarity)
{
    if( u32Polarity == UTCPD_PCACTL_SNKENLVL_LOW )
        return tcpc_update8(port, NPD48_PCACTL, BIT1, MASK_SET);
    else
        return tcpc_update8(port, NPD48_PCACTL, BIT1, MASK_CLR);
}


int32_t UTCPD_ForceDischargeActivePolarity(int port, uint32_t u32Polarity)
{
    if( u32Polarity == UTCPD_PCACTL_FORCEDLVL_LOW )
        return tcpc_update8(port, NPD48_PCACTL, BIT2, MASK_SET);
    else
        return tcpc_update8(port, NPD48_PCACTL, BIT2, MASK_CLR);
}

int32_t UTCPD_BleedDischargeActivePolarity(int port, uint32_t u32Polarity)
{
    if( u32Polarity == UTCPD_PCACTL_BLEEDDLVL_LOW )
        return tcpc_update8(port, NPD48_PCACTL, BIT3, MASK_SET);
    else
        return tcpc_update8(port, NPD48_PCACTL, BIT3, MASK_CLR);
}

uint32_t UTCPD_GetVbusVolt(int port)
{
    uint32_t vbus_vol;
    i2c_read16(port, NULL, NPD48_VBUSVOL, &vbus_vol);
    vbus_vol = vbus_vol*50;	//*20*1000*2.56/1024. Unit: mV.
    return vbus_vol;
}

/*@}*/ /* end of group I2C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I2C_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
