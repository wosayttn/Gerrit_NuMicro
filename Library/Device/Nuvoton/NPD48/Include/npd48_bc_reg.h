/*---------------------- ?????????????????????????????????????????? -------------------------*/
/**
    @addtogroup BC ??????????????????????????????????????????(BC)
    Memory Mapped Structure for BC Controller
@{ */
 
/**
 * @var BC_T::BCCR
 * Offset: 0x120  Battery Charger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BCDEN     |Battery   Charge Detect Enable
 * |        |          |Enable   battery charge detect, select DETMOD and then observer DETSTS to decide   contact port.
 * |        |          |PHY can be used for BCD, but cannot be used for communication when BCDEN   = 1.
 * |        |          |0 = Normal   operation.
 * |        |          |1 = Battery charge detect operation.
 * |        |          |[Design]   control MEGBC12LDO pin BC12DET_EN
 * |[3:1]   |DETMOD    |Detect Mode
 * |        |          |When BCDEN =   1, select detect mode to perform.
 * |        |          |000 = Idle,   nothing to detect.
 * |        |          |001 = VBUS   detect, detect USB VBUS whether great than threshold voltage.
 * |        |          |010 = Data   contact detect (DCD), detect data pin contact status.
 * |        |          |011 = Primary   detect (PD), distinguish between (SDP or NUSP) and (CDP or DCP).
 * |        |          |100 = Secondary   detect (SD), distinguish between CDP and DCP.
 * |        |          |Others = Reserved.
 * |[4]     |DETSTS    |Detect Status   (Read Only)
 * |        |          |When DETMOD =   000 (IDLE).
 * |        |          |DETSTS = 0.
 * |        |          |When DETMOD =   001 (VBUS detect).
 * |        |          |0 = VBUS   is less than threshold voltage.
 * |        |          |1 = VBUS   is greater than threshold voltage.
 * |        |          |When DETMOD =   010 (DCD detect).
 * |        |          |0 = Data pin   not contacted.
 * |        |          |1 = Data pin   contacted.
 * |        |          |When DETMOD =   011 (PD).
 * |        |          |0 = SDP port   or not USB support port. If it is not USB support port, NUSP is 1.
 * |        |          |1 = DCP or   CDP.
 * |        |          |When DETMOD =   100 (SD).
 * |        |          |0 = CDP.
 * |        |          |1 = DCP.
 * |[5]     |NUSP      |Not USB   Support Port (Read Only)
 * |        |          |When DETMOD =   011(PD), detect DM be pulled logic high, it means contact port not USB   support port.
 * |        |          |0 = USB   support port.
 * |        |          |1 = Not USB support port.
 * @var BC_T::BCAMSR
 * Offset: 0x127  Battery Charger Alpha Mode Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DCHGDET   |DCHG Detect Status
 * |        |          |This bit reflect BC12 analog   macro DCHG_DET status
 * |[1]     |CHGDET    |CHG Detect Status
 * |        |          |This bit reflect BC12 analog   macro CHG_DET status
 * |[2]     |PWRDET    |Power Detect Status
 * |        |          |This bit reflect BC12 analog   macro PWR_DET status
 * |[4]     |RXDP      |RXDP Status
 * |        |          |This bit reflect BC12 analog   macro RXDP status
 * |[5]     |RXDM      |RXDM Status
 * |        |          |This bit reflect BC12 analog   macro RXDM status
 */

#define NPD48_BCCR         0x0120            /*!< [0x0120] Battery Charger Control Register                                 */
#define NPD48_BCIR         0x0123            /*!< [0x0123] Battery Charger Interrupt Register                               */
#define NPD48_BCAMSR       0x0127            /*!< [0x0127] Battery Charger Alpha Mode Status Register                       */

/**
    @addtogroup BC_CONST BC Bit Field Definition
    Constant Definitions for BC Controller
@{ */

#define NPD48_BCCR_BCDEN_Pos                (0)                                               /*!< BC_T::BCCR: BCDEN Position             */
#define NPD48_BCCR_BCDEN_Msk                (0x1ul << NPD48_BCCR_BCDEN_Pos)                   /*!< BC_T::BCCR: BCDEN Mask                 */

#define NPD48_BCCR_DETMOD_Pos               (1)                                               /*!< BC_T::BCCR: DETMOD Position            */
#define NPD48_BCCR_DETMOD_Msk               (0x7ul << NPD48_BCCR_DETMOD_Pos)                  /*!< BC_T::BCCR: DETMOD Mask                */

#define NPD48_BCCR_DETSTS_Pos               (4)                                               /*!< BC_T::BCCR: DETSTS Position            */
#define NPD48_BCCR_DETSTS_Msk               (0x1ul << NPD48_BCCR_DETSTS_Pos)                  /*!< BC_T::BCCR: DETSTS Mask                */

#define NPD48_BCCR_NUSP_Pos                 (5)                                               /*!< BC_T::BCCR: NUSP Position              */
#define NPD48_BCCR_NUSP_Msk                 (0x1ul << NPD48_BCCR_NUSP_Pos)                    /*!< BC_T::BCCR: NUSP Mask                  */

#define NPD48_BCAMSR_DCHGDET_Pos            (0)                                               /*!< BC_T::BCAMSR: DCHGDET Position         */
#define NPD48_BCAMSR_DCHGDET_Msk            (0x1ul << NPD48_BCAMSR_DCHGDET_Pos)               /*!< BC_T::BCAMSR: DCHGDET Mask             */

#define NPD48_BCAMSR_CHGDET_Pos             (1)                                               /*!< BC_T::BCAMSR: CHGDET Position          */
#define NPD48_BCAMSR_CHGDET_Msk             (0x1ul << NPD48_BCAMSR_CHGDET_Pos)                /*!< BC_T::BCAMSR: CHGDET Mask              */

#define NPD48_BCAMSR_PWRDET_Pos             (2)                                               /*!< BC_T::BCAMSR: PWRDET Position          */
#define NPD48_BCAMSR_PWRDET_Msk             (0x1ul << NPD48_BCAMSR_PWRDET_Pos)                /*!< BC_T::BCAMSR: PWRDET Mask              */

#define NPD48_BCAMSR_RXDP_Pos               (4)                                               /*!< BC_T::BCAMSR: RXDP Position            */
#define NPD48_BCAMSR_RXDP_Msk               (0x1ul << NPD48_BCAMSR_RXDP_Pos)                  /*!< BC_T::BCAMSR: RXDP Mask                */

#define NPD48_BCAMSR_RXDM_Pos               (5)                                               /*!< BC_T::BCAMSR: RXDM Position            */
#define NPD48_BCAMSR_RXDM_Msk               (0x1ul << NPD48_BCAMSR_RXDM_Pos)                  /*!< BC_T::BCAMSR: RXDM Mask                */

/**@}*/ /* BC_CONST */
/**@}*/ /* end of BC register group */
