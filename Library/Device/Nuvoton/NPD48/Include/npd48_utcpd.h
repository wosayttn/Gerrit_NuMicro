/****************************************************************************//**
 * @file     utcpd.h
 * @version  V1.00
 * @brief    NPD48 series UTCPD driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __UTCPD_H__
#define __UTCPD_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UTCPD_Driver UTCPD Driver
  @{
*/
void UTCPD_GetTrimDACData(int port, uint16_t* pu16Trim5VData, uint16_t* pu16Trim48VData);
uint32_t UTCPD_Open(int port);
int32_t UTCPD_GetAlertStatus(int port, int* i32AlertSts);
int32_t UTCPD_ClearAlertStatus(int port, int AlertStClr);
int32_t UTCPD_EnableAlertMask(int port, int mask_set);
int32_t UTCPD_DisableAlertMask(int port, int mask_clr);
int32_t UTCPD_EnablePowerStatusMask(int port, int mask_set);
int32_t UTCPD_DisablePowerStatusMask(int port, int mask_clr);
int32_t UTCPD_EnableFaultMask(int port, int mask_set);
int32_t UTCPD_DisableFaultMask(int port, int mask_clr);
int32_t UTCPD_EnablePowerCtrl(int port, uint32_t mask_set);
int32_t UTCPD_DisablePowerCtrl(int port, uint32_t maskclr);
int32_t UTCPD_EnableFaultCtrl(int port, uint32_t mask_set);
int32_t UTCPD_DisableFaultCtrl(int port, uint32_t maskclr);
int32_t UTCPD_ClearPowerStatus(int port, int PowerStClr);
int32_t UTCPD_EnableFaultStatusMask(int port, int mask_set);
int32_t UTCPD_DisableFaultStatusMask(int port, int mask_clr);
int32_t UTCPD_GetFaultStatus(int port, int32_t* pi32RegData);
int32_t UTCPD_ClearFaultStatus(int port, int FaultStClr);
int32_t UTCPD_SetRoleCtrl(int port, uint32_t u32DrpToggle, uint32_t u32Rpvalue, uint32_t u32CC2, uint32_t u32CC1);
int32_t UTCPD_GetRoleCtrl(int port, uint32_t* pu32DrpTole, uint32_t* pu32CC1, uint32_t* pu32CC2, uint32_t* pu32Rpvalue);
int32_t UTCPD_SetTypeCPortCtrl(int port, uint32_t u32BistMode, uint32_t u32Orient);
int32_t UTCPD_GetTypeCPortCtrl(int port, uint32_t* pu32BistMode, uint32_t* pu32Orient);
int32_t UTCPD_IssueCmd(int port, uint32_t cmd);
int32_t UTCPD_GetCCSts(int port, uint32_t* pu32Look4Con, uint32_t* pu32ConRlt, uint32_t* pu32CC2Sts, uint32_t* pu32CC1Sts);
int32_t UTCPD_GetPwrSts(int port, uint32_t* pu32VBUSDetEn, uint32_t* pu32VBUSPresent, uint32_t* pu32VCONNPresent, uint32_t* pu32SnkVBUS);
int32_t UTCPD_GetPwrStsExt(int port, uint32_t* pu32DbgAccessory, uint32_t* pu32SrcNonDefVBUS, uint32_t* pu32SrcDefVBUS);
int32_t UTCPD_GetFaultSts(int port, uint32_t* pu32VBUSOverCurr, uint32_t* pu32VBUSOverVolt, uint32_t* pu32VCONNOverCurr, uint32_t* pu32I2CInfErr);
int32_t UTCPD_GetFaultStsExt(int port, uint32_t* pu32ForceOffFat, uint32_t* pu32AutoDiscFat, uint32_t* pu32ForceDiscFat);
int32_t UTCPD_SetMsgHeaderInfo(int port, uint32_t u32DataRole, uint32_t u32Revision, uint32_t u32PwrRole);
int32_t UTCPD_SetRecDetect(int port, uint32_t u32RegData);
int32_t UTCPD_SetVBUSAlarm(int port, uint32_t u32AlarmH, uint32_t u32AlarmL);
int32_t UTCPD_SetSnkDisconnect(int port, uint32_t u32SnkDiscVolt);
int32_t UTCPD_SetStopDischargeVolt(int port, uint32_t u32StopDischgVolt);
int32_t UTCPD_SetVbusPresentThreshold(int port, uint32_t u32Threshold);
int32_t UTCPD_SrcActivePolarity(int port, uint32_t u32Polarity);
int32_t UTCPD_SnkActivePolarity(int port, uint32_t u32Polarity);
int32_t UTCPD_ForceDischargeActivePolarity(int port, uint32_t u32Polarity);
int32_t UTCPD_BleedDischargeActivePolarity(int port, uint32_t u32Polarity);
uint32_t UTCPD_GetVbusVolt(int port);

/** @addtogroup UTCPD_EXPORTED_CONSTANTS UTCPD Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_ALERT constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ALERT_CCSCHIS                 NPD48_ALERTL_CCSCHIS_Msk         /*!< UTCPD_T::ALERT: CCSCHIS Mask          */
#define UTCPD_ALERT_PWRSCHIS                NPD48_ALERTL_PWRSCHIS_Msk        /*!< UTCPD_T::ALERT: PWRSCHIS Mask         */
#define UTCPD_ALERT_RXSOPIS                 NPD48_ALERTL_RXSOPIS_Msk         /*!< UTCPD_T::ALERT: RXSOPIS Mask          */
#define UTCPD_ALERT_RXHRSTIS                NPD48_ALERTL_RXHRSTIS_Msk        /*!< UTCPD_T::ALERT: RXHRSTIS Mask         */
#define UTCPD_ALERT_TXFAILIS                NPD48_ALERTL_TXFAILIS_Msk        /*!< UTCPD_T::ALERT: TXFAILIS Mask         */
#define UTCPD_ALERT_TXDCUIS                 NPD48_ALERTL_TXDCUIS_Msk         /*!< UTCPD_T::ALERT: TXDCUIS Mask          */
#define UTCPD_ALERT_TXOKIS                  NPD48_ALERTL_TXOKIS_Msk          /*!< UTCPD_T::ALERT: TXOKIS Mask           */
#define UTCPD_ALERT_VBAMHIS                 NPD48_ALERTL_VBAMHIS_Msk         /*!< UTCPD_T::ALERT: VBAMHIS Mask          */
#define UTCPD_ALERT_VBAMLIS                 (0x1ul << 8)                     /*!< UTCPD_T::ALERT: VBAMLIS Mask          */
#define UTCPD_ALERT_FUTIS                   (0x1ul << 9)                     /*!< UTCPD_T::ALERT: FUTIS Mask            */
#define UTCPD_ALERT_RXOFIS                  (0x1ul << 10)                    /*!< UTCPD_T::ALERT: RXOFIS Mask           */
#define UTCPD_ALERT_SKDCDTIS                (0x1ul << 11)                    /*!< UTCPD_T::ALERT: SKDCDTIS Mask         */
#define UTCPD_ALERT_VNDIS                   (0x1ul << 15)                    /*!< UTCPD_T::ALERT: VNDIS Mask            */


/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_ALERT Mask constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ALERTM_CCSCHIE        (0x1ul << NPD48_ALERTML_CCSCHIE_Pos)              /*!< UTCPD_T::ALERTML: CCSCHIE Mask         */
#define UTCPD_ALERTM_PWRSCHIE       (0x1ul << NPD48_ALERTML_PWRSCHIE_Pos)             /*!< UTCPD_T::ALERTML: PWRSCHIE Mask        */
#define UTCPD_ALERTM_RXSOPIE        (0x1ul << NPD48_ALERTML_RXSOPIE_Pos)              /*!< UTCPD_T::ALERTML: RXSOPIE Mask         */
#define UTCPD_ALERTM_RXHRSTIE       (0x1ul << NPD48_ALERTML_RXHRSTIE_Pos)             /*!< UTCPD_T::ALERTML: RXHRSTIE Mask        */
#define UTCPD_ALERTM_TXFAILIE       (0x1ul << NPD48_ALERTML_TXFAILIE_Pos)             /*!< UTCPD_T::ALERTML: TXFAILIE Mask        */
#define UTCPD_ALERTM_TXDCUIE        (0x1ul << NPD48_ALERTML_TXDCUIE_Pos)              /*!< UTCPD_T::ALERTML: TXDCUIE Mask         */
#define UTCPD_ALERTM_TXOKIE         (0x1ul << NPD48_ALERTML_TXOKIE_Pos)               /*!< UTCPD_T::ALERTML: TXOKIE Mask          */
#define UTCPD_ALERTM_VBAMHIE        (0x1ul << NPD48_ALERTML_VBAMHIE_Pos)              /*!< UTCPD_T::ALERTML: VBAMHIE Mask         */
#define UTCPD_ALERTM_VBAMLIE        (0x1ul << 8)                                      /*!< UTCPD_T::ALERTMH: VBAMLIE Mask         */
#define UTCPD_ALERTM_FUTIE          (0x1ul << 9)                                      /*!< UTCPD_T::ALERTMH: FUTIE Mask           */
#define UTCPD_ALERTM_RXOFIE         (0x1ul << 10)                                     /*!< UTCPD_T::ALERTMH: RXOFIE Mask          */
#define UTCPD_ALERTM_SKDCDTIE       (0x1ul << 11)                                     /*!< UTCPD_T::ALERTMH: SKDCDTIE Mask        */
#define UTCPD_ALERTM_VNDIE          (0x1ul << 15)                                     /*!< UTCPD_T::ALERTMH: VNDIE Mask           */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_PWRSM Power Status Mask constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRSM_SKVBIE           (0x1ul << NPD48_PWRSM_SKVBIE_Pos)                 /*!< UTCPD_T::PWRSM: SKVBIE Mask            */
#define UTCPD_PWRSM_VCPSIE           (0x1ul << NPD48_PWRSM_VCPSIE_Pos)                 /*!< UTCPD_T::PWRSM: VCPSIE Mask            */
#define UTCPD_PWRSM_VBPSIE           (0x1ul << NPD48_PWRSM_VBPSIE_Pos)                 /*!< UTCPD_T::PWRSM: VBPSIE Mask            */
#define UTCPD_PWRSM_VBDTDGIE         (0x1ul << NPD48_PWRSM_VBDTDGIE_Pos)               /*!< UTCPD_T::PWRSM: VBDTDGIE Mask          */
#define UTCPD_PWRSM_SRVBIE           (0x1ul << NPD48_PWRSM_SRVBIE_Pos)                 /*!< UTCPD_T::PWRSM: SRVBIE Mask            */
#define UTCPD_PWRSM_SRHVIE           (0x1ul << NPD48_PWRSM_SRHVIE_Pos)                 /*!< UTCPD_T::PWRSM: SRHVIE Mask            */
#define UTCPD_PWRSM_DACONIE          (0x1ul << NPD48_PWRSM_DACONIE_Pos)                /*!< UTCPD_T::PWRSM: DACONIE Mask           */


/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_FAULTSM Fault Status Mask constant definitions.                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FAULTSM_VCOCIE         (0x1ul << NPD48_FAULTSM_VCOCIE_Pos)               /*!< UTCPD_T::FAULTSM: VCOCIE Mask          */
#define UTCPD_FAULTSM_VBOVIE         (0x1ul << NPD48_FAULTSM_VBOVIE_Pos)               /*!< UTCPD_T::FAULTSM: VBOVIE Mask          */
#define UTCPD_FAULTSM_VBOCIE         (0x1ul << NPD48_FAULTSM_VBOCIE_Pos)               /*!< UTCPD_T::FAULTSM: VBOCIE Mask          */
#define UTCPD_FAULTSM_FDGFALIE       (0x1ul << NPD48_FAULTSM_FDGFALIE_Pos)             /*!< UTCPD_T::FAULTSM: FDGFALIE Mask        */
#define UTCPD_FAULTSM_ADGFALIE       (0x1ul << NPD48_FAULTSM_ADGFALIE_Pos)             /*!< UTCPD_T::FAULTSM: ADGFALIE Mask        */
#define UTCPD_FAULTSM_FOFFVBIE       (0x1ul << NPD48_FAULTSM_FOFFVBIE_Pos)             /*!< UTCPD_T::FAULTSM: FOFFVBIE Mask        */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_FAULTCTL constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FAULTCTL_VCOCDTDS      				NPD48_FAULTCTL_VCOCDTDS_Msk      /*!< UTCPD_T::FAULTCTL: VCOCDTDS Mask       */
#define UTCPD_FAULTCTL_VBOVDTDS      				NPD48_FAULTCTL_VBOVDTDS_Msk      /*!< UTCPD_T::FAULTCTL: VBOVDTDS Mask       */
#define UTCPD_FAULTCTL_VBOCDTDS      				NPD48_FAULTCTL_VBOCDTDS_Msk      /*!< UTCPD_T::FAULTCTL: VBOCDTDS Position   */
#define UTCPD_FAULTCTL_VBDGTMDS      				NPD48_FAULTCTL_VBDGTMDS_Msk      /*!< UTCPD_T::FAULTCTL: VBDGTMDS Mask       */
#define UTCPD_FAULTCTL_FOFFVBDS      				NPD48_FAULTCTL_FOFFVBDS_Msk      /*!< UTCPD_T::FAULTCTL: FOFFVBDS Mask       */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_PWRCTL constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRCTL_VCEN               		NPD48_PWRCTL_VCEN_Msk            /*!< UTCPD_T::PWRCTL: VCEN Mask             */
#define UTCPD_PWRCTL_VCPWR              		NPD48_PWRCTL_VCPWR_Msk           /*!< UTCPD_T::PWRCTL: VCPWR Mask            */
#define UTCPD_PWRCTL_FDGEN              		NPD48_PWRCTL_FDGEN_Msk           /*!< UTCPD_T::PWRCTL: FDGEN Mask            */
#define UTCPD_PWRCTL_BDGEN              		NPD48_PWRCTL_BDGEN_Msk           /*!< UTCPD_T::PWRCTL: BDGEN Mask            */
#define UTCPD_PWRCTL_ADGDC              		NPD48_PWRCTL_ADGDC_Msk           /*!< UTCPD_T::PWRCTL: ADGDC Mask            */
#define UTCPD_PWRCTL_DSVBAM_DIS             	NPD48_PWRCTL_DSVBAM_Msk          /*!< UTCPD_T::PWRCTL: DSVBAM Mask           */
#define UTCPD_PWRCTL_VBMONI_DIS                 NPD48_PWRCTL_VBMONI_Msk          /*!< UTCPD_T::PWRCTL: VBMONI Mask           */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_TCPCCTL constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_TCPCCTL_ORIENT                NPD48_TCPCCTL_ORIENT_Msk         /*!< UTCPD_T::TCPCCTL: ORIENT Mask          */
#define UTCPD_TCPCCTL_BISTEN                NPD48_TCPCCTL_BISTEN_Msk         /*!< UTCPD_T::TCPCCTL: BISTEN Mask          */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_ROLECTL constant definitions.                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ROLECTL_CC1              	   NPD48_ROLECTL_CC1_Msk             /*!< UTCPD_T::ROLECTL: CC1 Mask             */
#define UTCPD_ROLECTL_CC2                  NPD48_ROLECTL_CC1_Msk             /*!< UTCPD_T::ROLECTL: CC2 Mask             */
#define UTCPD_ROLECTL_RPVALUE              NPD48_ROLECTL_RPVALUE_Msk         /*!< UTCPD_T::ROLECTL: RPVALUE Mask         */
#define UTCPD_ROLECTL_DRP                  NPD48_ROLECTL_DRP_Msk             /*!< UTCPD_T::ROLECTL: DRP Mask             */

#define UTCPD_ROLECTL_CC1_RA               (0 << 0)
#define UTCPD_ROLECTL_CC1_RP               (1 << 0)
#define UTCPD_ROLECTL_CC1_RD               (2 << 0) 
#define UTCPD_ROLECTL_CC1_OPEN             (3 << 0)

#define UTCPD_ROLECTL_CC2_RA               (0 << 2)
#define UTCPD_ROLECTL_CC2_RP               (1 << 2)
#define UTCPD_ROLECTL_CC2_RD               (2 << 2) 
#define UTCPD_ROLECTL_CC2_OPEN             (3 << 2)

#define UTCPD_ROLECTL_RPVALUE_DEF          (0 << 4)
#define UTCPD_ROLECTL_RPVALUE_1P5A         (1 << 4)
#define UTCPD_ROLECTL_RPVALUE_3A           (2 << 4)

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_CCSTS constant definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_CCSTS_CC1STATE         				NPD48_CCSTS_CC1STATE_Msk              /*!< UTCPD_T::CCSTS: CC1STATE Mask          */
#define UTCPD_CCSTS_CC2STATE         				NPD48_CCSTS_CC2STATE_Msk              /*!< UTCPD_T::CCSTS: CC2STATE Mask          */
#define UTCPD_CCSTS_CONRLT           				NPD48_CCSTS_CONRLT_Msk                /*!< UTCPD_T::CCSTS: CONRLT Mask            */
#define UTCPD_CCSTS_LK4CONN          				NPD48_CCSTS_LK4CONN_Msk               /*!< UTCPD_T::CCSTS: LK4CONN Mask           */

#define UTCPD_UNDER_LK4CONN       			(1 << 5)
#define UTCPD_LK4CONN_DONE                  (0 << 5) 

#define UTCPD_CONN_RESULT_RP        	    (0 << 4)
#define UTCPD_CONN_RESULT_RD                (1 << 4) 

#define UTCPD_CCSTS_CC2STATE_SRC_OPEN	    (0 << 2)
#define UTCPD_CCSTS_CC2STATE_SRC_RA         (1 << 2) 
#define UTCPD_CCSTS_CC2STATE_SRC_RD         (2 << 2)

#define UTCPD_CCSTS_CC2STATE_SNK_OPEN	    (0 << 2)
#define UTCPD_CCSTS_CC2STATE_SNK_DEF        (1 << 2) 
#define UTCPD_CCSTS_CC2STATE_SNK_1P5A       (2 << 2)
#define UTCPD_CCSTS_CC2STATE_SNK_3A         (3 << 2)

#define UTCPD_CCSTS_CC1STATE_SRC_OPEN       (0 << 0) 
#define UTCPD_CCSTS_CC1STATE_SRC_RA         (1 << 0)  
#define UTCPD_CCSTS_CC1STATE_SRC_RD         (2 << 0)  

#define UTCPD_CCSTS_CC1STATE_SNK_OPEN	    (0 << 0)
#define UTCPD_CCSTS_CC1STATE_SNK_DEF        (1 << 0) 
#define UTCPD_CCSTS_CC1STATE_SNK_1P5A       (2 << 0)
#define UTCPD_CCSTS_CC1STATE_SNK_3A         (3 << 0)

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_PWRSTS constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRSTS_SKVB                   NPD48_PWRSTS_SKVB_Msk                 /*!< UTCPD_T::PWRSTS: SKVB Mask             */
#define UTCPD_PWRSTS_VCPS                   NPD48_PWRSTS_VCPS_Msk                 /*!< UTCPD_T::PWRSTS: VCPS Mask             */
#define UTCPD_PWRSTS_VBPS                   NPD48_PWRSTS_VBPS_Msk                 /*!< UTCPD_T::PWRSTS: VBPS Mask             */
#define UTCPD_PWRSTS_VBPSDTEN               NPD48_PWRSTS_VBPSDTEN_Msk             /*!< UTCPD_T::PWRSTS: VBPSDTEN Mask         */
#define UTCPD_PWRSTS_SRVB                   NPD48_PWRSTS_SRVB_Msk                 /*!< UTCPD_T::PWRSTS: SRVB Mask             */
#define UTCPD_PWRSTS_SRHV                   NPD48_PWRSTS_SRHV_Msk                 /*!< UTCPD_T::PWRSTS: SRHV Mask             */
#define UTCPD_PWRSTS_DACON                  NPD48_PWRSTS_DACON_Msk                /*!< UTCPD_T::PWRSTS: DACON Mask            */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_FAULTSTS constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FAULTSTS_I2CIFERR             NPD48_FAULTSTS_I2CIFERR_Msk           /*!< UTCPD_T::FAULTSTS: I2CIFERR Mask       */
#define UTCPD_FAULTSTS_VCOCFUT              NPD48_FAULTSTS_VCOCFUT_Msk            /*!< UTCPD_T::FAULTSTS: VCOCFUT Mask        */
#define UTCPD_FAULTSTS_VBOVFUT              NPD48_FAULTSTS_VBOVFUT_Msk            /*!< UTCPD_T::FAULTSTS: VBOVFUT Mask        */
#define UTCPD_FAULTSTS_VBOCFUT              NPD48_FAULTSTS_VBOCFUT_Msk            /*!< UTCPD_T::FAULTSTS: VBOCFUT Mask        */
#define UTCPD_FAULTSTS_FDGFAL               NPD48_FAULTSTS_FDGFAL_Msk             /*!< UTCPD_T::FAULTSTS: FDGFAL Mask         */
#define UTCPD_FAULTSTS_ADGFAL               NPD48_FAULTSTS_ADGFAL_Msk             /*!< UTCPD_T::FAULTSTS: ADGFAL Mask         */
#define UTCPD_FAULTSTS_FOFFVB               NPD48_FAULTSTS_FOFFVB_Msk             /*!< UTCPD_T::FAULTSTS: FOFFVB Mask         */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_CMD constant definitions.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
//#define NPD48_CMD_WAKEI2C                   (0x11)	/* Not Support */
#define UTCPD_CMD_DISABLE_VBUS_DETECT       (0x22)
#define UTCPD_CMD_ENABLE_VBUS_DETECT        (0x33)
#define UTCPD_CMD_DISABLE_SINK_VBUS         (0x44)
#define UTCPD_CMD_SINK_VBUS                 (0x55)
#define UTCPD_CMD_DISABLE_SRC_VBUS          (0x66)
#define UTCPD_CMD_SRC_VBUS_DEFAULT          (0x77)
#define UTCPD_CMD_SRC_VBUS_NONDEFAULT       (0x88)
#define UTCPD_CMD_LOOK4CONNECTION           (0x99)
//#define UTCPD_CMD_RX_ONE_MORE               (0xAA)  /* Not Support */
//#define UTCPD_CMD_SEND_FRS_SWAP_SIGNAL      (0xCC)	/* Not Support */
//#define UTCPD_CMD_SEND_RESET_TX_BUF         (0xDD)	/* Not Support */
//#define UTCPD_CMD_SEND_RESET_RX_BUF         (0xEE)	/* Not Support */
//#define UTCPD_CMD_I2C_IDLE                  (0xFF)	/* Not Support */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_MHINFO constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_MHINFO_DROLE_UFP              (0x0UL << MM NPD48_MHINFO_DROLE_Pos)      /*!< UTCPD_T::MHINFO: DROLE Position        */
#define UTCPD_MHINFO_DROLE_DFP              (0x1UL << MM NPD48_MHINFO_DROLE_Pos)      /*!< UTCPD_T::MHINFO: DROLE Position        */

#define UTCPD_MHINFO_PDREV_10               (0x0UL << MM NPD48_MHINFO_PDREV_Pos)      /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_20               (0x1UL << MM NPD48_MHINFO_PDREV_Pos)      /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_30               (0x2UL << MM NPD48_MHINFO_PDREV_Pos)      /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_31               (0x3UL << MM NPD48_MHINFO_PDREV_Pos)      /*!< UTCPD_T::MHINFO: PDREV Position        */

#define UTCPD_MHINFO_PROLE_SNK              (0x0UL << MM NPD48_MHINFO_PROLE_Pos)      /*!< UTCPD_T::MHINFO: PROLE Position        */
#define UTCPD_MHINFO_PROLE_SRC              (0x1UL << MM NPD48_MHINFO_PROLE_Pos)      /*!< UTCPD_T::MHINFO: PROLE Position        */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_RDET constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_RDET_SOPEN                    (0x1ul << NPD48_RDET_SOPEN_Pos)           /*!< UTCPD_T::RDET: SOPEN Mask              */
#define UTCPD_RDET_SOPPEN                   (0x1ul << NPD48_RDET_SOPPEN_Pos)          /*!< UTCPD_T::RDET: SOPPEN Mask             */
#define UTCPD_RDET_SOPPPEN                  (0x1ul << NPD48_RDET_SOPPPEN_Pos)         /*!< UTCPD_T::RDET: SOPPPEN Mask            */
#define UTCPD_RDET_SDBGPEN                  (0x1ul << NPD48_RDET_SDBGPEN_Pos)         /*!< UTCPD_T::RDET: SDBGPEN Mask            */
#define UTCPD_RDET_SDBGPPEN                 (0x1ul << NPD48_RDET_SDBGPPEN_Pos)        /*!< UTCPD_T::RDET: SDBGPPEN Mask           */
#define UTCPD_RDET_HRSTEN                   (0x1ul << NPD48_RDET_HRSTEN_Pos)          /*!< UTCPD_T::RDET: HRSTEN Mask             */
#define UTCPD_RDET_CABRSTEN                 (0x1ul << NPD48_RDET_CABRSTEN_Pos)        /*!< UTCPD_T::RDET: CABRSTEN Mask           */


/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_RDET constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_RDET_SOPEN                    (0x1ul << NPD48_RDET_SOPEN_Pos)           /*!< UTCPD_T::RDET: SOPEN Mask              */
#define UTCPD_RDET_SOPPEN                   (0x1ul << NPD48_RDET_SOPPEN_Pos)          /*!< UTCPD_T::RDET: SOPPEN Mask             */
#define UTCPD_RDET_SOPPPEN                  (0x1ul << NPD48_RDET_SOPPPEN_Pos)         /*!< UTCPD_T::RDET: SOPPPEN Mask            */
#define UTCPD_RDET_SDBGPEN                  (0x1ul << NPD48_RDET_SDBGPEN_Pos)         /*!< UTCPD_T::RDET: SDBGPEN Mask            */
#define UTCPD_RDET_SDBGPPEN                 (0x1ul << NPD48_RDET_SDBGPPEN_Pos)        /*!< UTCPD_T::RDET: SDBGPPEN Mask           */
#define UTCPD_RDET_HRSTEN                   (0x1ul << NPD48_RDET_HRSTEN_Pos)          /*!< UTCPD_T::RDET: HRSTEN Mask             */
#define UTCPD_RDET_CABRSTEN                 (0x1ul << NPD48_RDET_CABRSTEN_Pos)        /*!< UTCPD_T::RDET: CABRSTEN Mask           */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_FSTXCTL constant definitions.                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FSTXCTL_FSDISCTX8      (0x1ul << NPD48_FSTXCTL_FSDISCTX8_Pos)            /*!< UTCPD_T::FSTXCTL: FSDISCTX8 Mask       */
#define UTCPD_FSTXCTL_FSTXEN         (0x1ul << NPD48_FSTXCTL_FSTXEN_Pos)               /*!< UTCPD_T::FSTXCTL: FSTXEN Mask          */
#define UTCPD_FSTXCTL_FSRXDETEN      (0x1ul << NPD48_FSTXCTL_FSRXDETEN_Pos)            /*!< UTCPD_T::FSTXCTL: FSRXDETEN Mask       */
#define UTCPD_FSTXCTL_FSRXDET        (0x1ul << NPD48_FSTXCTL_FSRXDET_Pos)              /*!< UTCPD_T::FSTXCTL: FSRXDET Mask         */
#define UTCPD_FSTXCTL_CC1OVDET       (0x1ul << NPD48_FSTXCTL_CC1OVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC1OVDET Mask        */
#define UTCPD_FSTXCTL_CC2OVDET       (0x1ul << NPD48_FSTXCTL_CC2OVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC2OVDET Mask        */
#define UTCPD_FSTXCTL_CC1UVVDET      (0x1ul << NPD48_FSTXCTL_CC1UVVDET_Pos)            /*!< UTCPD_T::FSTXCTL: CC1UVVDET Mask       */
#define UTCPD_FSTXCTL_CC2UVDET       (0x1ul << NPD48_FSTXCTL_CC2UVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC2UVDET Mask        */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_PCACTL constant definitions.                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PCACTL_SRCENLVL_HIGH  (0x0ul << 0)                                       /*!< UTCPD_T::PWRASTS: SRC Enable Level     */
#define UTCPD_PCACTL_SRCENLVL_LOW   (0x1ul << 0)                                       /*!< UTCPD_T::PWRASTS: SRC Enable Level     */

#define UTCPD_PCACTL_SNKENLVL_HIGH  (0x0ul << 1)                                       /*!< UTCPD_T::PWRASTS: SNK Enable Level     */
#define UTCPD_PCACTL_SNKENLVL_LOW   (0x1ul << 1)                                       /*!< UTCPD_T::PWRASTS: SNK Enable Level     */

#define UTCPD_PCACTL_FORCEDLVL_HIGH (0x0ul << 2)                                       /*!< UTCPD_T::PWRASTS: Force Discharge Level*/
#define UTCPD_PCACTL_FORCEDLVL_LOW  (0x1ul << 2)                                       /*!< UTCPD_T::PWRASTS: Force Discharge Level*/
 
#define UTCPD_PCACTL_BLEEDDLVL_HIGH (0x0ul << 3)                                       /*!< UTCPD_T::PWRASTS: Bleed Discharge Level*/
#define UTCPD_PCACTL_BLEEDDLVL_LOW  (0x1ul << 3)                                       /*!< UTCPD_T::PWRASTS: Bleed Discharge Level*/
/*@}*/ /* end of group I2C_EXPORTED_CONSTANTS */ 


/*@}*/ /* end of group UTCPD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group UTCPD_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/