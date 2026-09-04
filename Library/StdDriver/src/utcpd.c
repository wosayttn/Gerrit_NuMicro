/**************************************************************************//**
 * @file     utcpd.c
 * @version  V1.00
 * @brief    M2L31 series UTCPD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"
#include "utcpdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif

//uint8_t I2C_ADDR[4] = {0x0, 0x0, 0x0, 0x0};
/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UTCPD_Driver UTCPD Driver
  @{
*/

/** @addtogroup UTCPD_EXPORTED_FUNCTIONS UTCPD Exported Functions
  @{
*/


static uint16_t UTCPD_TO_U16(uint32_t u32Val)
{
    return (uint16_t)(u32Val & 0xFFFFUL);
}

static uint16_t UTCPD_TO_U8_IN_U16(uint32_t u32Val)
{
    return (uint16_t)(u32Val & 0x00FFUL);
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
    return tcpc_addr_read16(port,  0, TCPC_REG_ALERT, i32AlertSts);
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
    return tcpc_update16(port, TCPC_REG_ALERT, UTCPD_TO_U16((uint32_t)AlertStClr), MASK_SET);
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
    return tcpc_update16(port, TCPC_REG_ALERT_MASK,
                         UTCPD_TO_U16((uint32_t)mask_set), MASK_SET);
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
    return tcpc_update16(port, TCPC_REG_ALERT_MASK, UTCPD_TO_U16((uint32_t)mask_clr), MASK_CLR);
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
    return tcpc_update16(port, TCPC_REG_POWER_STATUS_MASK, UTCPD_TO_U16((uint32_t)mask_set), MASK_SET);
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
    return tcpc_update16(port, TCPC_REG_POWER_STATUS_MASK, UTCPD_TO_U16((uint32_t)mask_clr), MASK_CLR);
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
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK,
                         UTCPD_TO_U16((uint32_t)mask_set), MASK_SET);
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
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK,
                         UTCPD_TO_U16((uint32_t)mask_clr), MASK_CLR);
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
    return tcpc_update16(port, TCPC_REG_POWER_CTRL, UTCPD_TO_U16(mask_set), MASK_SET);  /* VBUS Monitor and VBUS Alarm are 0 to enable */
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
    return tcpc_update16(port, TCPC_REG_POWER_CTRL, UTCPD_TO_U16(maskclr), MASK_CLR);   /* VBUS Monitor and VBUS Alarm are 1 to disable */
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
    return tcpc_update16(port, TCPC_REG_FAULT_CTRL, UTCPD_TO_U16(mask_set), MASK_CLR);  /* 0 to enable */
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
    return tcpc_update16(port, TCPC_REG_FAULT_CTRL, UTCPD_TO_U16(maskclr), MASK_SET);   /* 1 to disable */
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
    return tcpc_update16(port, TCPC_REG_POWER_STATUS, UTCPD_TO_U16((uint32_t)PowerStClr), MASK_CLR);
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
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS,
                         UTCPD_TO_U16((uint32_t)mask_set), MASK_SET);
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
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK, UTCPD_TO_U16((uint32_t)mask_clr), MASK_CLR);
}

/**
  * @brief      Get UTCPD Fault Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pi32RegData  Fault status
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_GetFaultStatus(int port, int* pi32RegData)
{
    return tcpc_addr_read16(port, 0, TCPC_REG_FAULT_STATUS, pi32RegData);
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
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS, UTCPD_TO_U16((uint32_t)FaultStClr), MASK_CLR);
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
    uint32_t u32RegSet;
    uint16_t u16RegSet;

    u32RegSet = u32DrpToggle | u32Rpvalue | u32CC2 | u32CC1;
    u16RegSet = UTCPD_TO_U8_IN_U16(u32RegSet);

    return tcpc_addr_write16(port, 0, TCPC_REG_ROLE_CTRL, u16RegSet);
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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_ROLE_CTRL, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32DrpToggle = u32Reg & UTCPD_ROLCTL_DRP_Msk;
    *pu32Rpvalue   = u32Reg & UTCPD_ROLCTL_RPVALUE_Msk;
    *pu32CC2       = u32Reg & UTCPD_ROLCTL_CC2_Msk;
    *pu32CC1       = u32Reg & UTCPD_ROLCTL_CC1_Msk;

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
    uint32_t u32RegSet;
    uint16_t u16RegSet;

    u32RegSet = u32BistMode | u32Orient;
    u16RegSet = UTCPD_TO_U8_IN_U16(u32RegSet);

    return tcpc_addr_write16(port, 0, TCPC_REG_TCPC_CTRL, u16RegSet);

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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_TCPC_CTRL, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32BistMode = u32Reg & UTCPD_TCPCCTL_BISTEN;
    *pu32Orient   = u32Reg & UTCPD_TCPCCTL_ORIENT;

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
int32_t UTCPD_IsssueCmd(int port, uint32_t cmd)
{
    return tcpc_addr_write16(port, 0, TCPC_REG_COMMAND, UTCPD_TO_U16(cmd));
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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_CC_STATUS, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32Look4Con = u32Reg & TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK;
    *pu32ConRlt   = u32Reg & TCPC_REG_CC_STATUS_CONNECT_RESULT_MASK;
    *pu32CC2Sts   = u32Reg & TCPC_REG_CC_STATUS_CC2_STATE_MASK;
    *pu32CC1Sts   = u32Reg & TCPC_REG_CC_STATUS_CC1_STATE_MASK;

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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_POWER_STATUS, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32VBUSDetEn    = u32Reg & UTCPD_PWRSTS_VBPSDTEN_Msk;
    *pu32VBUSPresent  = u32Reg & UTCPD_PWRSTS_VBPS_Msk;
    *pu32VCONNPresent = u32Reg & UTCPD_PWRSTS_VCPS_Msk;
    *pu32SnkVBUS      = u32Reg & UTCPD_PWRSTS_SKVB_Msk;

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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_POWER_STATUS, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32DbgAccessory = u32Reg & UTCPD_PWRSTS_DACON_Msk;
    *pu32SrcNonDefVBUS = u32Reg & UTCPD_PWRSTS_SRHV_Msk;
    *pu32SrcDefVBUS    = u32Reg & UTCPD_PWRSTS_SRVB_Msk;

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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_FAULT_STATUS, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32VBUSOverCurr  = u32Reg & UTCPD_FUTSTS_VBOCFUT_Msk;
    *pu32VBUSOverVolt  = u32Reg & UTCPD_FUTSTS_VBOVFUT_Msk;
    *pu32VCONNOverCurr = u32Reg & UTCPD_FUTSTS_VCOCFUT_Msk;
    *pu32I2CInfErr     = 0UL; /* [MOD][FIX] M2L31 doesn't support I2C */

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
    int32_t  rv;
    int      i32Reg;
    uint32_t u32Reg;

    rv = tcpc_addr_read32(port, 0, TCPC_REG_FAULT_STATUS, &i32Reg);
    u32Reg = (uint32_t)i32Reg;

    *pu32ForceOffFat = u32Reg & UTCPD_FUTSTS_FOFFVB_Msk;
    *pu32AutoDiscFat = u32Reg & UTCPD_FUTSTS_ADGFAL_Msk;
    *pu32ForceDiscFat = u32Reg & UTCPD_FUTSTS_FDGFAL_Msk;

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
    uint32_t u32RegSet;
    uint16_t u16RegSet;

    u32RegSet = u32DataRole | u32Revision | u32PwrRole;
    u16RegSet = UTCPD_TO_U8_IN_U16(u32RegSet);

    return tcpc_addr_write16(port, 0, TCPC_REG_MSG_HDR_INFO, u16RegSet);

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
    uint16_t u16RegSet;

    u16RegSet = UTCPD_TO_U8_IN_U16(u32RegData);
    return tcpc_addr_write16(port, 0, TCPC_REG_RX_DETECT, u16RegSet);

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

    u8TxBuf[0] = (uint8_t)(u32AlarmH & 0xFFUL);
    u8TxBuf[1] = (uint8_t)((u32AlarmH >> 8UL) & 0xFFUL);
    u8TxBuf[2] = (uint8_t)(u32AlarmL & 0xFFUL);
    u8TxBuf[3] = (uint8_t)((u32AlarmL >> 8UL) & 0xFFUL);

    return tcpc_write_block(port, TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG, u8TxBuf, 4UL);

}

/**
  * @brief      Set Sink Disconnection Voltage
  *
  * @param[in]  port             Specify UTCPD port
  * @param[in]  u32SnkDiscVolt   The Value of Sink Disconnection Threshold Voltage.
  *                              It depends on the VREF pin and external voltage divider of VBUS
  * @return     0: Successful,  1: Fail
  *
  * @details    Sink Will start the VBUS auto discharge if VBUS voltage less Sink Disconnection Threshold Level
  *             POWER_CONTROL[Auto Discharge] should set to 1.
  *
  */
int32_t UTCPD_SetSnkDisconnect(int port, uint32_t u32SnkDiscVolt)
{
    return tcpc_addr_write16(port, 0, TCPC_REG_VBUS_SINK_DISCONNECT_THRESH,
                             UTCPD_TO_U16(u32SnkDiscVolt));

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
    return tcpc_addr_write16(port, 0, TCPC_REG_VBUS_STOP_DISCHARGE_THRESH,
                             UTCPD_TO_U16(u32StopDischgVolt));

}


/*
    The CC ststus interupt won't be issued if Enable_VCONN_SRC_CC with wrong VCONN source CC pin
*/
/**
  * @brief      Disable VCONN Power Source to CCx pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    None
  *
  */
void UTCPD_vconn_disable_src_cc(int port)
{
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_ENABLE_VCONN, MASK_CLR);
}
/**
  * @brief      Enable VCONN Power Source to CCx pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    None
  *
  */
void UTCPD_vconn_enable_src_cc(int port)
{
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_ENABLE_VCONN, MASK_SET);
}

/**
  * @brief      VCONN Power Source to CC2 pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    Enable VCONN Source CC2, Communication channel through CC1
  *
  */
void UTCPD_vconn_enable_from_cc2(int port)
{
    (void)tcpc_update16(port, TCPC_REG_TCPC_CTRL, TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION, MASK_CLR);
}

/**
  * @brief      VCONN Power Source to CC1 pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    Enable VCONN Source CC2, Communication channel through CC1
  *
  */
void UTCPD_vconn_enable_from_cc1(int port)
{
    (void)tcpc_update16(port, TCPC_REG_TCPC_CTRL, TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION, MASK_SET);
}

/**
  * @brief      Set VCONN Active Polarity Low
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    It depends on the external circuit
  *
  */
void UTCPD_vconn_polarity_active_low(int port)
{
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VCEN, MASK_CLR);
}

/**
  * @brief      Set VCONN Active Polarity High
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    It depends on the external circuit
  *
  */
void UTCPD_vconn_polarity_active_high(int port)
{
    /* Set VCONN Polarity Active Low due to CC1VCENS and CC2VCENS default high */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VCEN, MASK_SET);
}

/**
  * @brief      Disable VCONN OC Fault
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details
  *
  */
void UTCPD_vconn_disable_oc_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS, MASK_SET);
}

/**
  * @brief      Enable VCONN OC Fault
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details
  *
  */
void UTCPD_vconn_enable_oc_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS, MASK_CLR);
}

/**
  * @brief      Specified VCONN Enable Multiplex Path
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  cc1vcensel          It should be 1 default
  * @param[in]  cc2vcensel          It should be 1 default
  *
  * @details
  *
  */
void UTCPD_vconn_mux_selection(int port, uint32_t cc1vcensel, uint32_t cc2vcensel)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, UTCPD_MUXSEL, &i32Data);
    u32Data = (uint32_t)i32Data;

    u32Data = (u32Data & (uint32_t)~((uint32_t)CC2VCENS | (uint32_t)CC1VCENS)) |
              (((cc1vcensel & 0x1UL) << 24UL) | ((cc2vcensel & 0x1UL) << 28UL));

    (void)tcpc_addr_write32(port, 0, UTCPD_MUXSEL, u32Data);

}

/**
  * @brief      Configure VCONN OC source detection
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Src              Specify VCONN OC Source Detection
  *             - \ref UTCPD_VCONN_OC_EINT0
  *             - \ref UTCPD_VCONN_OC_EINT1
  *             - \ref UTCPD_VCONN_OC_EINT2
  *             - \ref UTCPD_VCONN_OC_EINT3
  *             - \ref UTCPD_VCONN_ADC_CMP0
  *             - \ref UTCPD_VCONN_ACMP0
  *             - \ref UTCPD_VCONN_ACMP1
  *             - \ref UTCPD_VCONN_ACMP2
  * @details
  *
  */
void UTCPD_vconn_configure_oc_detection_soruce(int port, uint32_t u32Src)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, UTCPD_MUXSEL, &i32Data);
    u32Data = (uint32_t)i32Data;

    u32Data = (u32Data & (uint32_t)~VCOCS) | ((u32Src & 0xFUL) << 4UL);

    (void)tcpc_addr_write16(port, 0, UTCPD_MUXSEL, UTCPD_TO_U16(u32Data));

}

/* ============  VBUS SRCEN Polarity ==========*/
void UTCPD_vbus_srcen_polarity_active_low(int port)
{
    /* Set VBUS SRCEN Polarity active Low */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SRCEN, MASK_CLR);
}
void UTCPD_vbus_srcen_polarity_active_high(int port)
{
    /* Set VBUS SRCEN Polarity Active high */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SRCEN, MASK_SET);
}
/* ============  VBUS SNKEN Polarity ==========*/
void UTCPD_vbus_snken_polarity_active_low(int port)
{
    /* Set VBUS SNKEN Polarity active Low */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SNKEN, MASK_CLR);
}
void UTCPD_vbus_snken_polarity_active_high(int port)
{
    /* Set VBUS SNKEN Polarity Active high */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SNKEN, MASK_SET);
}

void UTCPD_vbus_disable_oc_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS, MASK_SET);
}
void UTCPD_vbus_enable_oc_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS, MASK_CLR);
}

void UTCPD_vbus_discharge_polarity_active_low(int port)
{
    /* Set VBUS discharge Polarity Active low */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VBDCHG, MASK_CLR);
}
void UTCPD_vbus_discharge_polarity_active_high(int port)
{
    /* Set VBUS discharge Polarity Active high */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VBDCHG, MASK_SET);
}

/**
  * @brief      Configure VBUS OC source detection
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Src              Specify VBUS OC Source Detection
  *             - \ref UTCPD_VBUS_OC_EINT0
  *             - \ref UTCPD_VBUS_OC_EINT1
  *             - \ref UTCPD_VBUS_OC_EINT2
  *             - \ref UTCPD_VBUS_OC_EINT3
  *             - \ref UTCPD_VBUS_ADC_CMP0
  *             - \ref UTCPD_VBUS_ACMP0
  *             - \ref UTCPD_VBUS_ACMP1
  *             - \ref UTCPD_VBUS_ACMP2
  * @details
  *
  */
void UTCPD_vbus_configure_oc_soruce(int port, uint32_t u32Src)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, UTCPD_MUXSEL, &i32Data);
    u32Data = (uint32_t)i32Data;

    u32Data = (u32Data & (uint32_t)~VBOCS) | ((u32Src & 0xFUL) << 0UL);

    (void)tcpc_addr_write16(port, 0, UTCPD_MUXSEL, UTCPD_TO_U16(u32Data));

}

/* VBUS ovp fault */
void UTCPD_vbus_disable_ov_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS, MASK_SET);
}
void UTCPD_vbus_enable_ov_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS, MASK_CLR);
}

/* VBUS force off fault */
void UTCPD_vbus_disable_forceoff_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS, MASK_SET);
}
void UTCPD_vbus_enable_forceoff_fault(int port)
{
    (void)tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS, MASK_CLR);
}


uint32_t UTCPD_vbus_is_source(int port)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, TCPC_REG_POWER_STATUS, &i32Data);
    u32Data = (uint32_t)i32Data;

    if ((u32Data & (uint32_t)TCPC_REG_POWER_STATUS_SOURCING_VBUS) != 0UL)
    {
        return 1UL;
    }

    return 0UL;

}

uint32_t UTCPD_vbus_is_sink(int port)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, TCPC_REG_POWER_STATUS, &i32Data);
    u32Data = (uint32_t)i32Data;

    if ((u32Data & (uint32_t)TCPC_REG_POWER_STATUS_SINKING_VBUS) != 0UL)
    {
        return 1UL;
    }

    return 0UL;
}

uint32_t UTCPD_vbus_is_source_hv(int port)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, TCPC_REG_POWER_STATUS, &i32Data);
    u32Data = (uint32_t)i32Data;

    if ((u32Data & (uint32_t)TCPC_REG_POWER_STATUS_SOURCING_HIGH_VBUS) != 0UL)
    {
        return 1UL;
    }

    return 0UL;

}

//=================================== for VBUS and VCONN
void UTCPD_power_enable_monitor(int port)
{
    /* 0 enable monitor */
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS, MASK_CLR);
}

void UTCPD_power_disable_monitor(int port)
{
    /* 1 disable monitor */
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS, MASK_SET);
}

/* VBUS enable auto discharge */
void UTCPD_power_disable_auto_discharge(int port)
{
    /* Disable Auto Discharge = 0 */
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT, MASK_CLR);
}
void UTCPD_power_enable_auto_discharge(int port)
{
    /* Enable Auto Discharge = 0 */
    (void)tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT, MASK_SET);
}

void UTCPD_frs_tx_polarity_active_low(int port)
{
    /* Set FRS Polarity Active low */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_FRSTX, MASK_CLR);
}
void UTCPD_frs_tx_polarity_active_high(int port)
{
    /* Set FRS Polarity Active high */
    (void)tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_FRSTX, MASK_SET);
}

void UTCPD_frs_mux_selection(int port, uint32_t cc1frssel, uint32_t cc2frssel)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read16(port, 0, UTCPD_MUXSEL, &i32Data);
    u32Data = (uint32_t)i32Data;

    u32Data = (u32Data & (uint32_t)~((uint32_t)CC2FRSS | (uint32_t)CC1FRSS)) |
              (((cc1frssel & 0x1UL) << 25UL) | ((cc2frssel & 0x1UL) << 29UL));

    (void)tcpc_addr_write32(port, 0, UTCPD_MUXSEL, u32Data);
}

/**
  * @brief      Enable specify UTCPD Controller and set Clock Divider
  *
  * @param[in]  UTCPD         Specify UTCPD port
  * @return     Always 0
  *
  * @details    The function needs to enable the I2C Controller and set proper clock divider first.
  *
  */
uint32_t UTCPD_Open(int port)
{


    /* Reset UTCPD */
    SYS->IPRST3 |= SYS_IPRST3_UTCPD0RST_Msk;
    SYS->IPRST3 &= ~SYS_IPRST3_UTCPD0RST_Msk;

    /* Enable PHY */
    SYS->UTCPDCTL |= 0x02UL;

    (void)tcpc_addr_write16(port, 0, UTCPD_PHYCTL, 0x03U);
    (void)tcpc_addr_write16(port, 0, UTCPD_CC_DB_TM, 0x64U);
    (void)tcpc_addr_write16(port, 0, UTCPD_FILTM, 0x04U);

    UTCPD_frs_mux_selection(port, 1UL, 1UL);
    UTCPD_vconn_mux_selection(port, 1UL, 1UL);

    /* Sink disconnection threshold based on 1/10 divider:
       350 / (3300 / 1024) ~= 108.6 */
    (void)i2c_write16(0, 0, TCPC_REG_VBUS_SINK_DISCONNECT_THRESH, 108U);

    return 0UL;

}

/**
  * @brief      Set VBUS Scale Base On External Voltage Divider Circuit
  *
  * @param[in]  port          Specify UTCPD port
  * @param[in]  div           Always 10 or 20 to assign voltage divider to 1/10 or 1/20.
  * @return     Successful
  *
  * @details
  *  00 = External VBUS voltage divider circuit should be 1/20 for EPR application. The divided voltage compares with 200mV to set or clean VBUS Present bit.
  *  01 = External VBUS voltage divider circuit should be 1/10 for SPR application. The divided voltage compares with 400mV to set or clean VBUS Present bit.
  *  10 = Same as 00.
  *  11 = Same as 01.
  */
uint32_t UTCPD_SetExternalDivider(int port, int div)
{
    int      i32Data;
    uint32_t u32Data;

    (void)tcpc_addr_read32(port, 0, TCPC_REG_VBUS_VOLTAGE, &i32Data);
    u32Data = (uint32_t)i32Data;

    if (div == 10)
    {
        (void)tcpc_addr_write32(port, 0, TCPC_REG_VBUS_VOLTAGE, (u32Data | BIT11));
    }
    else
    {
        (void)tcpc_addr_write32(port, 0, TCPC_REG_VBUS_VOLTAGE, (u32Data & (uint32_t)~BIT11));
    }

    return 0UL;

}

/*@}*/ /* end of group I2C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I2C_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
