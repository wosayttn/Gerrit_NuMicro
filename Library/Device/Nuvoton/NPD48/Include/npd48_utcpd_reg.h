/*---------------------- ?????????????????????????????????????????? -------------------------*/
/**
    @addtogroup UTCPD ??????????????????????????????????????????(UTCPD)
    Memory Mapped Structure for UTCPD Controller
@{ */

/**
 * @var UTCPD_T::VIDL
 * Offset: 0x00  Vendor ID Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VIDL      |Vendor ID   Low Byte Register
 * |        |          |Vendor   identifier is used to identify the TCPC vendor, the VID is a unique 16-bit   unsigned integer assigned by USB-IF to the Vendor.
 * @var UTCPD_T::VIDH
 * Offset: 0x01  Vendor ID High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VIDH      |Vendor ID   High Byte Register
 * |        |          |Vendor   identifier is used to identify the TCPC vendor, the VID is a unique 16-bit   unsigned integer assigned by USB-IF to the Vendor.
 * @var UTCPD_T::PIDL
 * Offset: 0x02  Product ID Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PIDL      |USB   Product ID Low Byte Register
 * |        |          |USB product   ID is used to identify the product.
 * @var UTCPD_T::PIDH
 * Offset: 0x03  Product ID High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PIDH      |USB   Product ID High Byte Register
 * |        |          |USB product   ID is used to identify the product.
 * @var UTCPD_T::DIDL
 * Offset: 0x04  Device ID Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIDL      |Device ID   Low Byte Register
 * |        |          |USB device ID   is used to identify the release version of the product.
 * @var UTCPD_T::DIDH
 * Offset: 0x05  Device ID High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIDH      |Device ID   High Byte Register
 * |        |          |USB device ID   is used to identify the release version of the product.
 * @var UTCPD_T::UTCREVL
 * Offset: 0x06  USB Type-C Revision Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UTCREVL   |USB Type C   Revision Low Byte Register
 * |        |          |USB Power   Delivery Specification revision 1.0a.
 * @var UTCPD_T::UTCREVH
 * Offset: 0x07  USB Type-C Revision High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UTCREVH   |USB Type C   Revision High Byte Register
 * |        |          |USB Power   Delivery Specification revision 1.0a.
 * @var UTCPD_T::UPDREVL
 * Offset: 0x08  USB Type-C PD Revision Version Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UPDREVL   |USB Type C   PD Revision Version Low Byte Register
 * |        |          |USB Type-C Cable   and Connector Specification Revision 1.0.
 * @var UTCPD_T::UPDREVH
 * Offset: 0x09  USB Type-C PD Revision Version High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UPDREVH   |USB Type C   Revision Version High Byte Register
 * |        |          |USB Type-C   Cable and Connector Specification Revision 3.0.
 * @var UTCPD_T::UPDCREVL
 * Offset: 0x0A  USB Type-C PD Controller Revision Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UPDCREVL  |TCPCI   Version Low Byte
 * |        |          |USB-Port   Controller Specification 1.0
 * @var UTCPD_T::UPDCREVH
 * Offset: 0x0B  USB Type-C PD Controller Revision High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |UPDCREVH  |TCPCI   Version High Byte
 * |        |          |USB-Port   Controller Specification 3.0
 * @var UTCPD_T::ALERTL
 * Offset: 0x10  Alert Flag Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CCSCHIS   |CC Status Changed
 * |        |          |0 = CC status not change.
 * |        |          |1= CC status changed.
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * |[1]     |PWRSCHIS  |Power Status Changed
 * |        |          |0 = Power status not change.
 * |        |          |1 = Power status changed.
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * |[2]     |RXSOPIS   |Received SOP Message
 * |        |          |0 = No SOP message Received.
 * |        |          |1 = Received SOP message   (Set after sending GoodCRC).
 * |        |          |UTCPD_RXBCNT being set to 0   does not set this bit.
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * |[3]     |RXHRSTIS  |Received Hard Reset
 * |        |          |0 = No Hard reset Received.
 * |        |          |1 = Received Hard reset.
 * |        |          |Note: It is cleared by software   writing 1 into this bit.
 * |[4]     |TXFAILIS  |Transmit SOP Fail
 * |        |          |0 = No Transmit SOP fail.
 * |        |          |1 = SOP* message   transmission not successful, no GoodCRC response received on SOP* message   transmission
 * |        |          |Transmit SOP* message buffer registers are empty.
 * |        |          |Note: It is cleared by software   writing 1 into this bit.
 * |[5]     |TXDCUIS   |Transmit SOP* Message   Discarded
 * |        |          |0 = No TX SOP discarded.
 * |        |          |1 = Reset or SOP* message   transmission not sent due to incoming receive message
 * |        |          |Transmit SOP* message   buffer registers are empty.
 * |        |          |Note: It is cleared by software   writing 1 into this bit
 * |[6]     |TXOKIS    |Transmit SOP* Message Successful
 * |        |          |0 = No TX SOP* transmit.
 * |        |          |1 = Reset or SOP* message   transmission successful
 * |        |          |GoodCRC response received on SOP* message   transmission
 * |        |          |Transmit SOP* message buffer registers are empty.
 * |        |          |Note: It is cleared by software writing 1 into this bit
 * |[7]     |VBAMHIS   |VBUS Voltage Alarm High
 * |        |          |0 = No high voltage alarm   has occured.
 * |        |          |1 = A high voltage   alarm has occured.
 * |        |          |This bit will be set high when DSVBAM   (UTCPD_PWRCTL[5]) is low and VBUS voltage is higher than VBAMH   (UTCPD_VBAMH[9:0]).
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * @var UTCPD_T::ALERTH
 * Offset: 0x11  Alert Flag High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VBAMLIS   |VBUS Voltage Alarm Low
 * |        |          |0 = No Low voltage alarm has occurred.
 * |        |          |1= A Low voltage alarm has occurred.
 * |        |          |This bit will be set high when DSVBAM   (UTCPD_PWRCTL[5]]) is low and VBUS voltage is lower than VBAML   (UTCPD_VBAML[9:0]).
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * |[1]     |FUTIS     |Fault Occur
 * |        |          |0 = No fault occurs.
 * |        |          |1= A Fault has occurred. Read the FUT_STS register.
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * |[2]     |RXOFIS    |Rx Buffer Overflow
 * |        |          |0 = RX buffer is functioning properly.
 * |        |          |1 = RX buffer has overflowed.
 * |        |          |Note: Writing 1 to this bit acknowledges the overflow
 * |        |          |The   overflow is cleared by writing 1 to RXSOPIS (UTCPD_IS[2])
 * |[3]     |SKDCDTIS  |VBUS Sink Disconnect Detected
 * |        |          |0 = No disconnect detected.
 * |        |          |1 = A VBUS Sink disconnect threshold   crossing has been detected.
 * |        |          |Note: This bit will be set when VBMONI (UTCPD_PWRCTL[6]) is   enabled and VBUS voltage drop lower than SKVBDCTH   (UTCPD_SKVBDCTH[9:0])
 * |        |          |It is cleared by software writing 1 into this   bit.
 * |[7]     |VNDIS     |Vendor Define Event Detected
 * |        |          |0 = No vendor defined interrupt status has been   detected.
 * |        |          |1 = A vendor defined interrupt status has been   detected
 * |        |          |Refer the vender defined interrupt status register.
 * |        |          |Note: It is cleared by software writing 1 into this bit.
 * @var UTCPD_T::ALERTML
 * Offset: 0x12  Alert Mask Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CCSCHIE   |CC Status Changed Interrupt Enable Bit
 * |        |          |0 = CC pin status changed interrupt Disabled.
 * |        |          |1 = CC pin status changed interrupt   Enabled.
 * |[1]     |PWRSCHIE  |Power Status Changed Interrupt Enable Bit
 * |        |          |0 = Power status changed interrupt Disabled.
 * |        |          |1 = Power status changed interrupt   Enabled.
 * |[2]     |RXSOPIE   |Received SOP Message Interrupt Enable Bit
 * |        |          |0 = Received SOP message interrupt Disabled.
 * |        |          |1 = Received SOP message interrupt   Enabled.
 * |[3]     |RXHRSTIE  |Received Hard Reset Interrupt Enable Bit
 * |        |          |0 = Received Hard Reseet interrupt Disabled.
 * |        |          |1 = Received Hard Reseet interrupt   Enabled.
 * |[4]     |TXFAILIE  |Transmit SOP Fail Interrupt Enable Bit
 * |        |          |0 = Transmit SOP fail interrupt Disabled.
 * |        |          |1 = Transmit SOP fail interrupt Enabled.
 * |[5]     |TXDCUIE   |Transmit SOP* Message Discarded Interrupt Enable Bit
 * |        |          |0 = Transmit SOP* message discarded interrupt   Disabled.
 * |        |          |1 = Transmit SOP* message discarded   interrupt Enabled.
 * |[6]     |TXOKIE    |Transmit SOP* Message Successful Interrupt Enable Bit
 * |        |          |0 = Transmit SOP* message successful interrupt   Disabled.
 * |        |          |1 = Transmit SOP* message successful   interrupt Enabled.
 * |[7]     |VBAMHIE   |VBUS Voltage Alarm High Interrupt Enable Bit
 * |        |          |0 = VBUS voltage reach high threshold interrupt Disabled.
 * |        |          |1 = VBUS voltage reach high threshold   interrupt Enabled.
 * @var UTCPD_T::ALERTMH
 * Offset: 0x13  Alert Mask High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VBAMLIE   |VBUS Voltage Alarm High Interrupt Enable Bit
 * |        |          |0 = VBUS voltage reach high threshold interrupt Disabled.
 * |        |          |1 = VBUS voltage reach high threshold   interrupt Enabled.
 * |[1]     |FUTIE     |Fault Occur Interrupt Enable Bit
 * |        |          |0 = Fault event occur interrupt Disabled.
 * |        |          |1 = Fault event occur interrupt Enabled.
 * |[2]     |RXOFIE    |Rx Buffer Overflow Interrupt Enable Bit
 * |        |          |0 = Received buffer interrupt Disabled.
 * |        |          |1 = Received buffer interrupt Enabled.
 * |[3]     |SKDCDTIE  |VBUS Sink Disconnect Detected Interrupt Enable Bit
 * |        |          |0 = VBUS sink disconnect detected interrupt Disabled.
 * |        |          |1 = VBUS sink disconnect detected interrupt Enabled.
 * |[7]     |VNDIE     |Vendor Define Event Detected Interrupt Enable Bit
 * |        |          |0 = Vendor define event detected interrupt Disabled.
 * |        |          |1 = Vendor define event detected interrupt   Enabled.
 * @var UTCPD_T::PWRSM
 * Offset: 0x14  Power Status Mask Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SKVBIE    |Sinking VBUS Status Interrupt Enable Bit
 * |        |          |0 = Sinking VBUS Status Interrupt Disabled.
 * |        |          |1 = Sinking VBUS Status Interrupt Enabled.
 * |[1]     |VCPSIE    |VCONN Present Status Interrupt Enable Bit
 * |        |          |0 = VCONN Present Status Interrupt Disabled.
 * |        |          |1 = VCONN Present Status Interrupt Enabled.
 * |[2]     |VBPSIE    |VBUS Present Status Interrupt Enable Bit
 * |        |          |0 = VBUS Present Status Interrupt Disabled.
 * |        |          |1 = VBUS Present Status Interrupt Enabled.
 * |[3]     |VBDTDGIE  |VBUS Detection Status Change Interrupt   Enable Bit
 * |        |          |0 = VBUS Detection Status Change Interrup Disabled.
 * |        |          |1 = VBUS Detection Status Change Interrup Enabled.
 * |[4]     |SRVBIE    |Sourcing VBUS Status Interrupt Enable Bit
 * |        |          |0 = Sourcing VBUS Status Interrupt Disabled.
 * |        |          |1 = Sourcing VBUS Status Interrupt Enabled.
 * |[5]     |SRHVIE    |Sourcing High Voltage Status Interrupt Enable Bit
 * |        |          |0 = Sourcing High Voltage Status Interrupt Disabled.
 * |        |          |1 = Sourcing High Voltage Status Interrupt Enabled.
 * |[7]     |DACONIE   |Debug Accessory Connected Status Interrupt Enable Bit
 * |        |          |0 = Debug Accessory Connected Status Interrupt   Disabled.
 * |        |          |1 = Debug Accessory Connected status   Interrupt Enabled.
 * @var UTCPD_T::FAULTSM
 * Offset: 0x15  Fault Status Mask Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |VCOCIE    |VCONN OCP Fault Interrupt Enable Bit
 * |        |          |0 = VCONN OCP Fault Interrupt Disabled.
 * |        |          |1 = VCONN OCP Fault Interrupt Enabled.
 * |[2]     |VBOVIE    |Internal VBUS OVP Fault Interrupt Enable   Bit
 * |        |          |0 = Internal VBUS OVP Fault Interrupt Disabled.
 * |        |          |1 = Internal VBUS OVP Fault Interrupt Enabled.
 * |[3]     |VBOCIE    |External VBUS OCP Fault Interrupt Enable   Bit
 * |        |          |0 = External VBUS OCP Fault Interrupt Disabled.
 * |        |          |1 = External VBUS OCP Fault Interrupt Enabled.
 * |[4]     |FDGFALIE  |Force Discharge Failed Interrupt Enable Bit
 * |        |          |0 = Force Discharge Failed Interrupt Disabled.
 * |        |          |1 = Force Discharge Failed Interrupt Enabled.
 * |[5]     |ADGFALIE  |Auto Discharge Failed Interrupt Enable Bit
 * |        |          |0 = Auto Discharge Failed Interrupt Disabled.
 * |        |          |1 = Auto Discharge Failed Interrupt Enabled.
 * |[6]     |FOFFVBIE  |Force Off VBUS Interrupt Enable Bit
 * |        |          |0 = Force Off VBUS Interrupt Disabled.
 * |        |          |1 = Force Off VBUS Interrupt Enabled.
 * @var UTCPD_T::CFGSO
 * Offset: 0x18  Configuration Standard Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TCCO      |Type-C   Connector Orientation
 * |        |          |0 = Normal u2013   output low.
 * |        |          |1 = Flipped u2013 output high.
 * |        |          |Note: If TCPC_CONTROL.DebugAccessiryControl = 0,   the TCPC shall write to this bit and ignore input from TCPM.
 * |[5]     |AUDIOAC   |Audio   Accessory Connected#
 * |        |          |0 = Audio   Accessory connected.
 * |        |          |1 = No Audio   Accessory connected (default).
 * |[6]     |DBGAC     |Debug   Accessory Connected#
 * |        |          |0 = Debug   Accessory Connected# output is driven low.
 * |        |          |1 = Debug Accessory Connected#   output is driven high.
 * |        |          |Note: Controlled by either the TCPM or TCPC
 * |        |          |The TCPC shall   ignore write to this bit if TCPC_CONTROL.DebugAccessiryControl is 0.
 * |[7]     |HIOUT     |High   Impedance Outputs
 * |        |          |0 = Standard   output control.
 * |        |          |1 = Force all   outputs to high impedance.
 * @var UTCPD_T::TCPCCTL
 * Offset: 0x19  TCPC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ORIENT    |Plug Orientation
 * |        |          |0 = When VCONN is enabled, apply it to the CC2 pin
 * |        |          |Monitor the CC1 pin for BMC communications if PD messaging is enabled.
 * |        |          |1 = When VCONN is enabled, apply it to the CC1 pin.   Monitor the CC2 pin for BMC
 * |        |          |communications if PD messaging is enabled.
 * |[1]     |BISTEN    |BIST Test Mode
 * |        |          |Setting this bit to 1 is intended to be used only   when a USB compliance tester is using USB BIST Test Data to test the PHY   layer of the UTCPD
 * |        |          |The CPU should clear this bit when a detach is detected.
 * |        |          |0 = Normal Operation.
 * |        |          |1 = BIST Test Mode.
 * @var UTCPD_T::ROLECTL
 * Offset: 0x1A  Role Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |CC1       |CC1 Connected Resistance Vaule
 * |        |          |00 = Reserved.
 * |        |          |01b = Rp (Use Rp definition in RPVALUE).
 * |        |          |10 = Rd.
 * |        |          |11 = Open (Disconnect or donu2019t care).
 * |[3:2]   |CC2       |CC2 Connected Resistance Value
 * |        |          |00 = Reserved.
 * |        |          |01 = Rp (Use Rp definition in RPVALUE).
 * |        |          |10 = Rd.
 * |        |          |11 = Open (Disconnect or donu2019t care).
 * |[5:4]   |RPVALUE   |Rp Value
 * |        |          |00 = Rp default.
 * |        |          |01 = Rp 1.5A.
 * |        |          |10 = Rp 3.0A.
 * |        |          |11 = Reserved.
 * |[6]     |DRP       |Dual Role Port
 * |        |          |0 = No dual role port.
 * |        |          |1 = Dual role port.
 * |        |          |The UTCPD toggles CC1 & CC2 after   receiving LK4CON (UTCPD_CMD[5]) and until a connection is detected
 * |        |          |Upon   connection, the UTCPD shall resolve to either an Rp or Rd and report the   CC1/CC2 State in the CC_STS register
 * @var UTCPD_T::FAULTCTL
 * Offset: 0x1B  Fault Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VCOCDTDS  |External VCONN Overcurrent Fault Detect Disable Bit
 * |        |          |0 = External VCONN OCP circuit Enabled.
 * |        |          |1 = External VCONN OCP circuit Disabled.
 * |[1]     |VBOVDTDS  |Internal VBUS Over Voltage Protection   Fault Detect Disable Bit
 * |        |          |0 = Internal OVP circuit Enabled.
 * |        |          |1 = Internal OVP circuit Disabled.
 * |        |          |Note: Internal VBUS over voltage protection   means to use ADC to measure VBUS voltage.
 * |[2]     |VBOCDTDS  |External VBUS Overcurrent Protection Fault   Detect Disable Bit
 * |        |          |0 = External OCP circuit Enabled.
 * |        |          |1 = External OCP circuit Disabled.
 * |[3]     |VBDGTMDS  |VBUS Discharge Fault Detection Timer   Disable Bit
 * |        |          |0 = VBUS Discharge Fault Detection Timer Enabled.
 * |        |          |1 = VBUS Discharge Fault Detection Timer Disabled.
 * |        |          |This enables the timer for both force   discharge and auto discharge, and no timer for bleed discharge
 * |        |          |Once   time-out, UTCPD will compare VBUS voltage with VSAFE0V or SP_DGTH,   depending on the setting of ADGDC and FDGEN.
 * |[4]     |FOFFVBDS  |Force Off VBUS Disable Bit
 * |        |          |0 = Force Off VBUS Enabled.
 * |        |          |1 = Force Off VBUS Disabled.
 * @var UTCPD_T::PWRCTL
 * Offset: 0x1C  Power Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VCEN      |Enable VCONN
 * |        |          |0 = VCONN Source Disabled (default).
 * |        |          |1 = VCONN Source   to CC Enabled.
 * |[1]     |VCPWR     |VCONN Power Supported
 * |        |          |0 = UTCPD delivers at least 1W on VCONN.
 * |        |          |1 = UTCPD delivers at least the power   indicated in CPVCPWR (UTCPD_DVCAP2[3:1]).
 * |[2]     |FDGEN     |Enable Force Discharge
 * |        |          |0 = Disable forced discharge (default).
 * |        |          |1 = Enable forced discharge of VBUS.
 * |        |          |Note: Force discharge is used for source only. This bit   will only be cleared by CPU.
 * |[3]     |BDGEN     |Enable Bleed Discharge
 * |        |          |0 = Bleed discharge Disabled (default).
 * |        |          |1 = Bleed discharge of VBUS Enabled.
 * |        |          |Note: Bleed Discharge is a low current discharge to provide   a minimum load current if needed.
 * |        |          |10 KOhms or 2mA is recommended. Bleed discharge is   used for sink only.
 * |        |          |This bit will only be cleared by CPU.
 * |[4]     |ADGDC     |Auto Discharge Disconnect
 * |        |          |0 = The UTCPD shall not automatically discharge VBUS   based on VBUS voltage (default).
 * |        |          |1 = The UTCPD shall automatically discharge when   disconnect detected.
 * |        |          |Setting this bit in a Source UTCPD triggers the   following actions upon a disconnect detection:
 * |        |          |l   Disable sourcing power   over VBUS
 * |        |          |l   VBUS   discharge
 * |        |          |Sourcing power over VBUS shall be disabled before or at the   same time as starting VBUS discharge.
 * |        |          |Setting this bit in a Sink UTCPD triggers the VBUS   discharge upon a disconnect detection.
 * |        |          |The UTCPD shall automatically disable discharge   (without clearing this bit) once the voltage on VBUS is below vSafe0V (max)
 * |        |          |UTCPD shall   not re-apply discharge circuit if VBUS rises above vSafe0V.
 * |        |          |This bit will only be cleared by CPU.
 * |[5]     |DSVBAM    |Disable VBUS Voltage Alarms
 * |        |          |0 = VBUS Voltage Alarms Power status   reporting Enabled.
 * |        |          |1 = VBUS Voltage Alarms Power   status reporting Disabled (default).
 * |[6]     |VBMONI    |VBUS Voltage Monitor Enable Bit
 * |        |          |0 = VBUS voltage monitoring Enabled.
 * |        |          |1 = VBUS voltage monitoring Disabled   (default).
 * |        |          |Note: This bit only controls VBUS voltage   monitoring
 * |        |          |UTCPD_VBVOL shall report all zeroes if disabled
 * @var UTCPD_T::CCSTS
 * Offset: 0x1D  CC Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |CC1STATE  |CC1 State
 * |        |          |If (CC1 (ROLCTL[1:0])=Rp) or (CONRLT (CCSTS[4]) = 0).
 * |        |          |00 = SRC.Open (Open, Rp).
 * |        |          |01 = SRC.Ra (below maximum vRa).
 * |        |          |10 = SRC.Rd (within the vRd range).
 * |        |          |11 = reserved.
 * |        |          |If (CC1 (ROLCTL[1:0])=Rd) or (CONRLT (CCSTS[4]) = 1).
 * |        |          |00 = SNK.Open (Below maximum vRa).
 * |        |          |01 = SNK.Default (Above minimum vRd-Connect).
 * |        |          |10 = SNK.Power1.5 (Above minimum vRd-Connect) Detects   Rp 1.5A.
 * |        |          |11 = SNK.Power3.0 (Above minimum vRd-Connect) Detects   Rp 3.0A.
 * |        |          |If CC1 (ROLCTL[1:0])=Reserved, this field is set to   00.
 * |        |          |If CC1 (ROLCTL[1:0])=Open, this field is set to 00.
 * |        |          |This field always returns 00 if   (LK4CONN=1) or (VCEN (PWRCTL[0])]=1 and ORIENT (TCPCCTL[0])=1)
 * |        |          |Otherwise,   the returned value depends upon CC1 (ROLCTL[1:0])
 * |[3:2]   |CC2STATE  |CC2 State
 * |        |          |If (CC2 (ROLCTL[3:2]=Rp) or (CONRLT (CCSTS[4]) = 0).
 * |        |          |00 = SRC.Open (Open, Rp).
 * |        |          |01 = SRC.Ra (below maximum vRa).
 * |        |          |10 = SRC.Rd (within the vRd range).
 * |        |          |11 = reserved.
 * |        |          |If (CC2 (ROLCTL[3:2])=Rd) or (CONRLT (CCSTS[4]) = 1).
 * |        |          |00 = SNK.Open (Below maximum vRa).
 * |        |          |01 = SNK.Default (Above minimum vRd-Connect).
 * |        |          |10 = SNK.Power1.5 (Above minimum vRd-Connect) Detects   Rp 1.5A.
 * |        |          |11 = SNK.Power3.0 (Above minimum vRd-Connect) Detects   Rp 3.0A.
 * |        |          |If CC2 (ROLCTL[3:2])=Reserved, this field is set to   00.
 * |        |          |If CC2 (ROLCTL[3:2])=Open, this field is set to 00.
 * |        |          |This field always returns 00 if   (LK4CONN=1) or (VCEN (PWRCTL[0])=1 and ORIENT (ROLECTL[0])=0)
 * |        |          |Otherwise, the   returned value depends upon CC2 (ROLCTL[3:2])
 * |[4]     |CONRLT    |Connect Result (read only)
 * |        |          |0 = the UTCPD is presenting Rp.
 * |        |          |1 = the UTCPD is presenting Rd.
 * |[5]     |LK4CONN   |Looking for Connection
 * |        |          |0 = UTCPD is not actively looking for a connection
 * |        |          |A   transition from '1' to '0' indicates a potential connection has been found.
 * |        |          |1 = UTCPD is looking for a connection   (toggling as a DRP or looking for a connection as Sink/Source only   condition)
 * @var UTCPD_T::PWRSTS
 * Offset: 0x1E  Power Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SKVB      |Sinking VBUS
 * |        |          |0 = Sink is Disconnected (Default).
 * |        |          |1 = UTCPD is sinking VBUS to   the system load.
 * |[1]     |VCPS      |VCONN Present
 * |        |          |0 = VCONN is not present.
 * |        |          |1 = This bit is asserted when VCONN present CC1 or   CC2. Threshold is fixed at 2.4V.
 * |        |          |If VCEN (UTCPD_PWRCTL[0])] is disabled   VCONN Present should be set to 0.
 * |[2]     |VBPS      |VBUS Present
 * |        |          |0 = VBUS Disconnected.
 * |        |          |1 = VBUS Connected.
 * |        |          |The UTCPD shall report VBUS present when   UTCPD detects VBUS rises above 4V.
 * |        |          |The UTCPD shall report VBUS is   not present when UTCPD detects VBUS falls below 3.5V.
 * |[3]     |VBPSDTEN  |VBUS Present Detection Enabled
 * |        |          |0 = VBUS Present Detection Disabled.
 * |        |          |1 = VBUS Present Detection   Enabled (default).
 * |[4]     |SRVB      |Sourcing VBUS
 * |        |          |0 = Sourcing VBUS Disabled.
 * |        |          |1 = Sourcing VBUS Enabled.
 * |        |          |This bit does not control the path, just   provides a monitor of the status.
 * |[5]     |SRHV      |Sourcing High Voltage
 * |        |          |0 = VSAFE5V.
 * |        |          |1 = Higher Voltage.
 * |        |          |This bit does not control the power path,   it just provides a monitor of the status
 * |        |          |This bit is asserted as long as the   UTCPD is sourcing nondefault voltage over VBUS (i.e
 * |        |          |not VSAFE5V) as   a response to CPU writing 0x88 to CMD (Source VBUS High Voltage)
 * |[7]     |DACON     |Debug Accessory Connected
 * |        |          |0 = No Debug Accessory connected (default).
 * |        |          |1 = Debug Accessory connected.
 * @var UTCPD_T::FAULTSTS
 * Offset: 0x1F  Fault Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |I2CIFERR  |I2C Interface Error
 * |        |          |0 = No Error.
 * |        |          |1 = I2C error has occurred.
 * |        |          |A TRANSMIT has been sent with an   empty TRANSMIT_BUFFER
 * |        |          |May be asserted if a non-zero value has been written   to a reserved bit in a register.
 * |[1]     |VCOCFUT   |VCONN Overcurrent Protection Fault
 * |        |          |0 = Not in an overcurrent protection state.
 * |        |          |1 = Overcurrent fault latched.
 * |[2]     |VBOVFUT   |Internal VBUS Over Voltage Protection   Fault
 * |        |          |0 = No Fault detected.
 * |        |          |1 = Over-voltage fault latched.
 * |[3]     |VBOCFUT   |External VBUS Overcurrent Protection Fault
 * |        |          |0 = Not in an overcurrent protection state.
 * |        |          |1 = Overcurrent fault latched.
 * |[4]     |FDGFAL    |Force Discharge Failed
 * |        |          |0 = No discharge failure.
 * |        |          |1 = Discharge commanded by the TCPM failed.
 * |        |          |If FDGEN (PWRCTL[2]) is set, the UTCPD   shall report a discharge fails if VBUS is not below vSafe0V within   tSafe0V
 * |[5]     |ADGFAL    |Auto Discharge Failed
 * |        |          |0 = No discharge failure.
 * |        |          |1 = Discharge commanded by the TCPM failed.
 * |        |          |If ADGDC (PWRCTL[4]) is set, the UTCPD   shall report discharge fails if VBUS is not below vSafe0V within   tSafe0V
 * |[6]     |FOFFVB    |Force Off VBUS
 * |        |          |0 = No Fault Detected, no action (default and not   supported).
 * |        |          |1 = VBUS Source/Sink has been forced off   due to external fault.
 * |        |          |The UTCPD has disconnected VBUS   due to external inputs (EINT0 ~ EINT5)
 * @var UTCPD_T::CMD
 * Offset: 0x23  Command Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CMD       |Command Set
 * |        |          |0x111 = Wake up I2C.
 * |        |          |0x22 = Disable VBUS Detect.
 * |        |          |0x33 = Enable VBUS Detect.
 * |        |          |0x44 = Disable Sink VBUS.
 * |        |          |0x55 = Enable Sink VBUS.
 * |        |          |0x66 = Disable Source VBUS.
 * |        |          |0x77 = Enable Source VBUS 5V.
 * |        |          |0x88 = Source VBUS High Voltage.
 * |        |          |0x99 = Look4Connection.
 * |        |          |0xAA = RxOneMore
 * |        |          |(u9019u662Fu4EC0u9EBC??? &agrave;after send   good CRC, then clear receive event DT_RXVNT)
 * |        |          |0xBB = Reserved.
 * |        |          |0xCC = Reserved.
 * |        |          |0xDD = Reserved.
 * |        |          |0xEE = Reserved.
 * |        |          |0xFF = I2C Idle.
 * |        |          |The Command is issued by the TCPM
 * |        |          |The Command is   cleared by the TCPD after being acted upon
 * |        |          |It always read as 0.
 * @var UTCPD_T::DCAP1L
 * Offset: 0x24  Device Capabilities 1 Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CPSRVB    |Source VBUS
 * |        |          |0 = TCPC is not capable of controlling the source   path to VBUS.
 * |        |          |1 = TCPC is capable of controlling the   source path to VBUS.
 * |[1]     |CPSRHV    |Source High Voltage VBUS
 * |        |          |0 = UTCPD is not capable of controlling the source   high voltage path to VBUS.
 * |        |          |1 = UTCPD is capable of controlling the   source high voltage path to VBUS.
 * |[2]     |CPSKVB    |Sink VBUS
 * |        |          |0 = UTCPD is not capable controlling the sink path to   the system load.
 * |        |          |1 = UTCPD is capable of controlling the   sink path to the system load.
 * |[3]     |CPSRVC    |Source VCONN
 * |        |          |0 = UTCPD is not capable of switching VCONN.
 * |        |          |1 = UTCPD is capable of switching VCONN.
 * |[4]     |CPSDBG    |SOPu2019_DBG/SOPu201D_DBG Support
 * |        |          |0 = All SOP* except SOPu2019_DBG/SOPu201D_DBG.
 * |        |          |1 = All SOP* messages are supported.
 * |[7:5]   |CPROL     |Roles Supported
 * |        |          |000 = USB Type-C Port Manager can configure the Port   as Source only or Sink only (not DRP).
 * |        |          |001 = Source only.
 * |        |          |010 = Sink only.
 * |        |          |011 = Sink with accessory support.
 * |        |          |100 = DRP only.
 * |        |          |101 = Source, Sink, DRP, Adapter/Cable all supported.
 * |        |          |110 = Source, Sink, DRP.
 * |        |          |111 = Not valid.
 * @var UTCPD_T::DCAP1H
 * Offset: 0x25  Device Capabilities 1 High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |CPSRRE    |Source Resistor Supported
 * |        |          |00 = Rp default only.
 * |        |          |01 = Rp 1.5A and default.
 * |        |          |10 = Rp 3.0A, 1.5A, and default.
 * |        |          |11 = Reserved.
 * |[2]     |CPVBAM    |VBUS Measurement and Alarm Capable
 * |        |          |0 = No VBUS voltage measurement nor VBUS   Alarms.
 * |        |          |1 = VBUS voltage measurement   and VBUS Alarms.
 * |[3]     |CPFDG     |Force Discharge
 * |        |          |0 = No Force Discharge implemented.
 * |        |          |1 = Force Discharge is implemented.
 * |[4]     |CPBDG     |Bleed Discharge
 * |        |          |0 = No Bleed Discharge implemented.
 * |        |          |1 = Bleed Discharge is implemented.
 * |[5]     |CPVBOVP   |VBUS OVP Reporting
 * |        |          |0 = VBUS OVP is not reported.
 * |        |          |1 = VBUS OVP is reported.
 * |[6]     |CPVBOCP   |VBUS OCP Reporting
 * |        |          |0 = VBUS OCP is not reported.
 * |        |          |1 = VBUS OCP is reported.
 * @var UTCPD_T::DCAP2L
 * Offset: 0x26  Device Capabilities 2 Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CPVCOC    |VCONN Overcurrent Fault Capable
 * |        |          |0 = UTCPD is not capable of detecting a Vconn fault.
 * |        |          |1 = UTCPD is capable of detecting a Vconn   fault.
 * |[3:1]   |CPVCPWR   |VCONN Power Supported
 * |        |          |000 = 1.0W.
 * |        |          |001 = 1.5W.
 * |        |          |010 = 2.0W.
 * |        |          |011 = 3W.
 * |        |          |100 = 4W.
 * |        |          |101 = 5W.
 * |        |          |110 = 6W.
 * |        |          |111 = External.
 * |[5:4]   |CPVBAMLS  |VBUS Voltage Alarm LSB
 * |        |          |00 = TCPD has 25mV LSB for its voltage alarm and uses   all 10 bits in.
 * |        |          |VBUSVOLTAGE_ALARM_HI_CFG and
 * |        |          |VBUSVOLTAGE_ALARM_LO_CFG.
 * |        |          |01 = TCPD has 50mV LSB for its voltage alarm and uses   all 9 bits in.
 * |        |          |VBUSVOLTAGE_ALARM_HI_CFG and
 * |        |          |VBUSVOLTAGE_ALARM_LO_CFG are ignored by TCPD.
 * |        |          |10 = TCPD has 100mV LSB for its voltage alarm and   uses all 8 bits in.
 * |        |          |VBUSVOLTAGE_ALARM_HI_CFG and
 * |        |          |VBUSVOLTAGE_ALARM_LO_CFG are ignored by TCPD.
 * |        |          |11 = Reserved.
 * |        |          |Support for VBUS_VOLTAGE_ALARM_LO_CFG and VBUS_VOLTAGE_ALARM_HI   implemented.
 * |[6]     |CPSPDGTH  |Stop Discharge Threshold
 * |        |          |0 = VBUS_STOP_DISCONNECT_THRESHOLD not   implemented.
 * |        |          |1 = VBUS_STOP   DISCONNECT_THRESHOLD implemented.
 * |[7]     |CPSKDCDT  |Sink Disconnect Detection
 * |        |          |0 = VBUS_SINK_DISCONNECT_THRESHOLD not   implemented.
 * |        |          |1 = VBUS_SINK_DISCONNECT_THRESHOLD   implemented.
 * @var UTCPD_T::DCAP2H
 * Offset: 0x27  Device Capabilities 2 High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var UTCPD_T::SICAP
 * Offset: 0x28  Stand Input Capabilities Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CPVCOC    |Force OFF VBUS
 * |        |          |0 = Force OFF VBUS not present in TCPD.
 * |        |          |1 = Force OFF VBUS present in TCPD.
 * |        |          |Note: This bit for sink or source
 * |[1]     |CPVCPWR   |VBUS External Overcurrent Fault
 * |        |          |0 = VBUS externsl overcurrent fault not   present in TCPD.
 * |        |          |1 = VBUS externsl overcurrent   fault present in TCPD.
 * |[2]     |VBEOVF    |VBUS External Over Voltage Fault
 * |        |          |0 = VBUS externsl over voltage fault not   present in TCPD.
 * |        |          |1 = VBUS externsl over voltage fault   present in TCPD.
 * @var UTCPD_T::SOCAP
 * Offset: 0x29  Stand Output Capabilities Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CONORI    |Connector Orientation
 * |        |          |0 = Connect orientation not present in TCPD.
 * |        |          |1 = Connect orientation present in TCPD.
 * |[1]     |CONRES    |Connector Present
 * |        |          |0 = Connect not present in TCPD.
 * |        |          |1 = Connect present in TCPD.
 * |        |          |Note: Controlled by the TCPM
 * |[2]     |MUXCINF   |MUC Configuration Conttrol
 * |        |          |0 = MUX configuration control not present in TCPD.
 * |        |          |1 = MUX configuration control present in TCPD.
 * |[3]     |ACIND     |Active Cable Indicator
 * |        |          |0 = Active cable not present in TCPD.
 * |        |          |1 = Active cable present in TCPD.
 * |[4]     |AAAIND    |Audio Adapter Accessory Indicator
 * |        |          |0 = Audio adapter accessory not present in TCPD.
 * |        |          |1 = Audio adapter accessory present in TCPD.
 * |[5]     |VPMON     |VBUS Present Monitor
 * |        |          |0 = VBUS not present in TCPD.
 * |        |          |1 = VBUS present in TCPD.
 * |[6]     |DAIND     |Debug Accessory Indicator
 * |        |          |0 = Debug accessory not present in TCPD.
 * |        |          |1 = Debug accessory present in TCPD.
 * @var UTCPD_T::MHINFO
 * Offset: 0x2E  Message Header Information Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PROLE     |Power Role
 * |        |          |0 = Sink.
 * |        |          |1 = Source.
 * |[2:1]   |PDREV     |USB PD specification Revision
 * |        |          |00 = Revision 1.0.
 * |        |          |01 = Revision 2.0.
 * |        |          |10 = Reserved.
 * |        |          |11 = Reserved.
 * |[3]     |DROLE     |Data Role
 * |        |          |0 = UFP.
 * |        |          |1 = DFP.
 * |[4]     |CPLUG     |Cable Plug
 * |        |          |0 = Message originated from Source, Sink, or DRP.
 * |        |          |1 = Message originated from a Cable plug.
 * @var UTCPD_T::RDET
 * Offset: 0x2F  Receive Detect Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SOPEN     |Enable SOP message
 * |        |          |0 = UTCPD does not detect SOP message (default).
 * |        |          |1 = UTCPD detects SOP message.
 * |[1]     |SOPPEN    |Enable SOPu2019 message
 * |        |          |0 = UTCPD does not detect SOPu2019 message (default).
 * |        |          |1 = UTCPD detects SOPu2019 message.
 * |[2]     |SOPPPEN   |Enable SOPu2019u2019 message
 * |        |          |0 = UTCPD does not detect SOPu2019u2019 message (default).
 * |        |          |1 = UTCPD detects SOPu2019u2019 message.
 * |[3]     |SDBGPEN   |Enable SOP_DBGu2019 message
 * |        |          |0 = UTCPD does not detect SOP_DBGu2019 message (default).
 * |        |          |1 = UTCPD detects SOP_DBGu2019 message.
 * |[4]     |SDBGPPEN  |Enable SOP_DBGu2019u2019 message
 * |        |          |0 = UTCPD does not detect SOP_DBGu2019u2019 message   (default).
 * |        |          |1 = UTCPD detects SOP_DBGu2019u2019 message.
 * |[5]     |HRSTEN    |Enable Hard Reset
 * |        |          |0 = UTCPD does not detect Hard Reset signaling   (default).
 * |        |          |1 = UTCPD detects Hard Reset signaling.
 * |[6]     |CABRSTEN  |Enable Cable Reset
 * |        |          |0 = UTCPD does not detect Cable Reset signaling   (default).
 * |        |          |1 = UTCPD detects Cable Reset signaling.
 * @var UTCPD_T::RBCNT
 * Offset: 0x30  Receive Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBCNT    |Receive Byte Count
 * |        |          |Indicates number of bytes in this register that are   not stale
 * |        |          |TCPM should read the first RXBCNT bytes in this register
 * |        |          |This is   the number of bytes in the RXDAx plus three (for the RXFTYPE and RXHEAD).
 * |        |          |The TCPD shall clear the DTRXEVNT and the RXBCNT register   to disable the PD message passing when CPU writes the TXCTL register   requesting a Hard Reset
 * @var UTCPD_T::RBFTYP
 * Offset: 0x31  Receive Buffer Frame Type Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |RXFTYPE   |Receive Buffer Frame Type
 * |        |          |000 = Received SOP.
 * |        |          |001 = Received SOP'.
 * |        |          |010 = Received SOP''.
 * |        |          |011 = Received SOP_DBGu2019.
 * |        |          |100 = Received SOP_DBGu2019u2019.
 * |        |          |110 = Received Cable Reset.
 * |        |          |All others are reserved
 * @var UTCPD_T::RXBHEADL
 * Offset: 0x32  Receive Buffer Header Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXHEAD    |Receive Buffer Header 0
 * |        |          |USB PD message header byte 0
 * @var UTCPD_T::RXBHEADH
 * Offset: 0x33  Receive Buffer Header High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXHEAD    |Receive Buffer Header 1
 * |        |          |USB PD message header byte 1
 * @var UTCPD_T::RXBUF0
 * Offset: 0x34  Receive Buffer Byte 0 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF1
 * Offset: 0x35  Receive Buffer Byte 1 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF2
 * Offset: 0x36  Receive Buffer Byte 2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF3
 * Offset: 0x37  Receive Buffer Byte 3 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF4
 * Offset: 0x38  Receive Buffer Byte 4 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF5
 * Offset: 0x39  Receive Buffer Byte 5 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF6
 * Offset: 0x3A  Receive Buffer Byte 6 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF7
 * Offset: 0x3B  Receive Buffer Byte 7 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF8
 * Offset: 0x3C  Receive Buffer Byte 8 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF9
 * Offset: 0x3D  Receive Buffer Byte 9 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF10
 * Offset: 0x3E  Receive Buffer Byte 10 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF11
 * Offset: 0x3F  Receive Buffer Byte 11 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF12
 * Offset: 0x40  Receive Buffer Byte 12 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF13
 * Offset: 0x41  Receive Buffer Byte 13 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF14
 * Offset: 0x42  Receive Buffer Byte 14 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF15
 * Offset: 0x43  Receive Buffer Byte 15 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF16
 * Offset: 0x44  Receive Buffer Byte 16 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF17
 * Offset: 0x45  Receive Buffer Byte 17 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF18
 * Offset: 0x46  Receive Buffer Byte 18 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF19
 * Offset: 0x47  Receive Buffer Byte 19 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF20
 * Offset: 0x48  Receive Buffer Byte 20 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF21
 * Offset: 0x49  Receive Buffer Byte 21 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF22
 * Offset: 0x4A  Receive Buffer Byte 22 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF23
 * Offset: 0x4B  Receive Buffer Byte 23 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF24
 * Offset: 0x4C  Receive Buffer Byte 24 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF25
 * Offset: 0x4D  Receive Buffer Byte 25 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF26
 * Offset: 0x4E  Receive Buffer Byte 26 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::RXBUF27
 * Offset: 0x4F  Receive Buffer Byte 27 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RXBUF     |Receive Buffer
 * |        |          |TCPD recevie buffer
 * @var UTCPD_T::TRANSMIT
 * Offset: 0x50  Transmit Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |TXSTYPE   |Transmit SOP* message
 * |        |          |000 = Transmit SOP.
 * |        |          |001 = Transmit SOP'.
 * |        |          |010 = Transmit SOP''.
 * |        |          |011 = Transmit SOP_DBGu2019.
 * |        |          |100 = Transmit SOP_DBGu2019u2019.
 * |        |          |101 = Transmit Hard Reset.
 * |        |          |110 = Transmit Cable Reset.
 * |        |          |111 = Transmit BIST Carrier Mode 2.
 * |        |          |The TCPD shall ignore the Retry Counter bits when   transmitting a Hard Reset, Cable Reset, or BIST Carrier
 * |[5:4]   |RXBCNT    |Retry Counter
 * |        |          |00 = No message retry is required.
 * |        |          |01 = Automatically retry message transmission once.
 * |        |          |10 = Automatically retry message transmission twice.
 * |        |          |11 = Automatically retry message transmission three   times.
 * @var UTCPD_T::TBCNT
 * Offset: 0x51  Transmit Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TBCNT     |Transmit Byte Count
 * |        |          |This is the number of bytes in the UTCPD_TXDA plus   two (for the TXHEAD)
 * |        |          |If a previous transmit request has not yet completed   when TX_CTL is written requesting a Hard Reset, the TCPD shall assert the   Transmission Discarded bit(TXDCUTIS (ALERTM[5])).
 * |        |          |The UTCPD shall assert both TXOKIS (ALERTM[6]) and   TXFALIS (ALERTM[4]) after it completes the sending of a Hard Reset
 * @var UTCPD_T::TXBHEADL
 * Offset: 0x52  Transmit Buffer Header Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXHEAD    |Transmit Buffer Header 0
 * |        |          |USB PD trasnmit message header byte 0
 * @var UTCPD_T::TXBHEADH
 * Offset: 0x53  Transmit Buffer Header High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXHEAD    |Transmit Buffer Header 1
 * |        |          |USB PD transmit message header byte 1
 * @var UTCPD_T::TXBUF0
 * Offset: 0x54  Transmit Buffer Byte 0 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF1
 * Offset: 0x55  Transmit Buffer Byte 1 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF2
 * Offset: 0x56  Transmit Buffer Byte 2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF3
 * Offset: 0x57  Transmit Buffer Byte 3 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF4
 * Offset: 0x58  Transmit Buffer Byte 4 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF5
 * Offset: 0x59  Transmit Buffer Byte 5 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF6
 * Offset: 0x5A  Transmit Buffer Byte 6 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF7
 * Offset: 0x5B  Transmit Buffer Byte 7 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF8
 * Offset: 0x5C  Transmit Buffer Byte 8 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF9
 * Offset: 0x5D  Transmit Buffer Byte 9 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF10
 * Offset: 0x5E  Transmit Buffer Byte 10 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF11
 * Offset: 0x5F  Transmit Buffer Byte 11 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF12
 * Offset: 0x60  Transmit Buffer Byte 12 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF13
 * Offset: 0x61  Transmit Buffer Byte 13 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF14
 * Offset: 0x62  Transmit Buffer Byte 14 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF15
 * Offset: 0x63  Transmit Buffer Byte 15 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF16
 * Offset: 0x64  Transmit Buffer Byte 16 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF17
 * Offset: 0x65  Transmit Buffer Byte 17 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF18
 * Offset: 0x66  Transmit Buffer Byte 18 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF19
 * Offset: 0x67  Transmit Buffer Byte 19 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF20
 * Offset: 0x68  Transmit Buffer Byte 20 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF21
 * Offset: 0x69  Transmit Buffer Byte 21 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF22
 * Offset: 0x6A  Transmit Buffer Byte 22 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF23
 * Offset: 0x6B  Transmit Buffer Byte 23 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF24
 * Offset: 0x6C  Transmit Buffer Byte 24 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF25
 * Offset: 0x6D  Transmit Buffer Byte 25 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF26
 * Offset: 0x6E  Transmit Buffer Byte 26 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::TXBUF27
 * Offset: 0x6F  Transmit Buffer Byte 27 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TXBUF     |Transmit Buffer
 * |        |          |TCPD transmit buffer
 * @var UTCPD_T::VBUSVOL
 * Offset: 0x70  VBUS Voltage Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VBVOL     |VBUS voltage measurement
 * |        |          |10-bit measurement of (VBUS / Scale   Factor)
 * |        |          |TCPM multiplies this value by the scale factor to   obtain the voltage measurement
 * |        |          |Voltages greater than or equal to 4V shall   meet +/-2% absolute value or +/- 50mV, whichever is greater
 * |        |          |The LSB is 25mV
 * @var UTCPD_T::VBUSVOH
 * Offset: 0x71  VBUS Voltage High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |VBVOL     |VBUS voltage measurement
 * |        |          |10-bit measurement of (VBUS / Scale   Factor)
 * |        |          |TCPM multiplies this value by the scale factor to   obtain the voltage measurement
 * |        |          |Voltages greater than or equal to 4V shall   meet +/-2% absolute value or +/- 50mV, whichever is greater
 * |        |          |The LSB is 25mV
 * |[3:2]   |VBSCALE   |VBUS   Scale Factor
 * |        |          |00 = VBUS measurement not scaled.
 * |        |          |01 = VBUS measurement divided by 10.
 * |        |          |10 = VBUS measurement divided by 20.
 * |        |          |11 = reserved.
 * @var UTCPD_T::VBUSDTL
 * Offset: 0x72  VBUS Sink Disconnect Threshold Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SKVBDCTH  |Sink VBUS Ddisconnect Threshold
 * |        |          |10-bit for voltage threshold with 25mV LSB. (Default   3.5V)
 * |        |          |A value of zero disables this threshold.
 * @var UTCPD_T::VBUSDTH
 * Offset: 0x73  VBUS Sink Disconnect Threshold High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SKVBDCTH  |Sink VBUS Ddisconnect Threshold
 * |        |          |10-bit for voltage threshold with 25mV LSB. (Default   3.5V)
 * |        |          |A value of zero disables this threshold.
 * @var UTCPD_T::VBUSTPDTL
 * Offset: 0x74  VBUS Stop Discharge Threshold Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SPGDTH    |VBUS Stop Force Discharge Threshold
 * |        |          |10-bit for voltage threshold with 25mV LSB. (Default   3.5V)
 * |        |          |A value of zero disables this threshold.
 * @var UTCPD_T::VBUSTPDTH
 * Offset: 0x75  VBUS Stop Discharge Threshold High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SPGDTH    |VBUS Stop Force Discharge Threshold
 * |        |          |10-bit for voltage threshold with 25mV LSB. (Default   3.5V)
 * |        |          |A value of zero disables this threshold.
 * @var UTCPD_T::VBUSOVTL
 * Offset: 0x76  VBUS Voltage Alarm High CFG Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VBAMTH    |VBUS High Voltage Alarm Threshold Register
 * |        |          |10-bit for high voltage threshold with 25mV LSB.   (Default vSafe5V)
 * @var UTCPD_T::VBUSOVTH
 * Offset: 0x77  VBUS Voltage Alarm High CFG High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |VBAMTH    |VBUS High Voltage Alarm Threshold Register
 * |        |          |10-bit for high voltage threshold with 25mV LSB.   (Default vSafe5V)
 * @var UTCPD_T::VBUSUVTL
 * Offset: 0x78  VBUS Voltage Alarm Low CFG Low Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VBAMTL    |VBUS Low Voltage Alarm Threshold Register
 * |        |          |10-bit for low voltage threshold with 25mV LSB.   (Default vSafe5V)
 * @var UTCPD_T::VBUSUVTH
 * Offset: 0x79  VBUS Voltage Alarm Low CFG High Byte Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |VBAMTL    |VBUS Low Voltage Alarm Threshold Register
 * |        |          |10-bit for low voltage threshold with 25mV LSB.   (Default vSafe5V)
 * @var UTCPD_T::FSASTS
 * Offset: 0x80  Fast Swap Alert Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FSDISRXSTS|Fast Swap Discharge RX Bit
 * |        |          |0 = No fast swap receive detected.
 * |        |          |1 = Fast swap receive detected.
 * |[1]     |FSDISTXSTS|Fast Swap Discharge TX Bit
 * |        |          |0 = No fast swap transmit detected.
 * |        |          |1 = Fast swap transmit detected.
 * |[3]     |WDTSTS    |Watch Dog Time Out Bit
 * |        |          |0 = No watch dog time out.
 * |        |          |1 = Watch dog time out event active.
 * |[4]     |EHWRSTK   |External Hardware Reset Keep
 * |        |          |0 = No external hardware reset.
 * |        |          |1 = Detected external hardware   reset.
 * |[5]     |VCUSTS    |VCON Under Voltage
 * |        |          |0 = No in uunder voltage.
 * |        |          |1 = VCON under voltage.
 * |[6]     |VCOTPSTS  |VCON Over Tamperature Protection
 * |        |          |0 = No in an over-temperature   protection state.
 * |        |          |1 = VCON Over-temperature.
 * @var UTCPD_T::DRPTRCTL
 * Offset: 0x83  DRP Toggle Ratio Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |DRPRATIO  |The Percent of Time that a DRP   Shall Advertise Source & Sink During tDRP
 * |        |          |000 = 50:50 (Sink : Source).
 * |        |          |010 = 30:70 (Sink : Source).
 * |        |          |001 = 40:60 (Sink : Source).
 * |        |          |101 = 60:40 (Sink : Source).
 * |        |          |110 =70:30 (Sink: Source).
 * |        |          |111 = Reserved.
 * @var UTCPD_T::FSAMASK
 * Offset: 0x84  Fasrt Swap Alert Mask Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FSDISRXEN |Fast Swap Discharge RX Mask Bit
 * |        |          |0 = Fast swap discharge receive mask bit is Disabled.
 * |        |          |1 = Fast swap discharge receive mask bit is Enabled.
 * |[1]     |FSDISTXEN |Fast Swap Discharge TX Bit
 * |        |          |0 = Fast swap discharge transmit mask bit is   Disabled.
 * |        |          |1 = Fast swap discharge transmit mask bit is Enabled.
 * |[3]     |WDTEN     |Watch Dog Time Out Bit
 * |        |          |0 = Watch dog time out mask bit is Disabled.
 * |        |          |1 = Watch dog time out mask bit is Enabled.
 * |[4]     |EHWRSTKEN |External Hardware Reset Keep
 * |        |          |0 = External hardware reset keep mask bit is   Disabled.
 * |        |          |1 = External hardware reset keep mask bit is Enabled.
 * |[5]     |VCUEN     |VCON Under Voltage
 * |        |          |0 = VCON under voltage mask bit is Disabled.
 * |        |          |1 = VCON under voltage mask bit is Enabled.
 * |[6]     |VCOTPEN   |VCON Over Tamperature Protection
 * |        |          |0 = VCON over tamperature proection mask bit is   Disabled.
 * |        |          |1 = VCON over tamperature protection mask bit is   Enabled.
 * @var UTCPD_T::BMCSCTL
 * Offset: 0x87  BMC Slice Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SLICEL    |TX Slice Low Level Control
 * |        |          |Low level slice control (The LSB is 2mV.)
 * |        |          |00 = 0.18V.
 * |        |          |01 = 0.2V.
 * |        |          |10 = 0.22V.
 * |        |          |11 = 0.24V (Default).
 * |[3:2]   |SLICEH    |TX Slice High Level Control
 * |        |          |High level slice control (The LSB is 2mV.)
 * |        |          |00 = 0.84V (Default).
 * |        |          |01 = 0.86V.
 * |        |          |10 = 0.88V.
 * |        |          |11 = 0.9V.
 * |[6:5]   |SLICEM    |TX Slice Middle Level Control
 * |        |          |Middle level slice control (The LSB is 2mV.)
 * |        |          |000 = 0.48V.
 * |        |          |100 = 0.56V (Default).
 * |        |          |111 = 0.62V.
 * |        |          |Others = Reserved.
 * @var UTCPD_T::NCIDL
 * Offset: 0x88  Nuvoton Chip ID Low Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var UTCPD_T::NCIDH
 * Offset: 0x89  Nuvoton Chip ID High Byte
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var UTCPD_T::VCDISCTL
 * Offset: 0x8A  VCONN Discharge Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VCONDISEN |VCONN Discharge Enable
 * |        |          |0 = VCON discharge is Disabled.
 * |        |          |1 = VCON discharge is Enabled.
 * |[3:1]   |VBSTPWT   |VBUS stop discharge   de-bounce timer
 * |        |          |The awit time is 31u * 4 * Value
 * |[6:4]   |VBOPNWT   |VBUS open discharge   de-bounce timer
 * |        |          |31u * Value
 * @var UTCPD_T::CCTRIM
 * Offset: 0x8B  CC Trim Value Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |CCTRIMR   |CC Driver Trim Rising Value
 * |        |          |Trim value of CC driver for PHY TX   rising slew rate (equal AFE T_cc_tr)
 * |[5:3]   |CCTRIMF   |CC Driver Trim Falling Value
 * |        |          |Trim value of CC driver for PHY TX   falling slew rate (equal AFE T_cc_tf)
 * |[7:6]   |BIDLSEL   |Bus Idle Select
 * |        |          |00 = Digital bus no transaction   over 12us.
 * |        |          |01 = Digital and analog bus no   transaction over 12us.
 * |        |          |10 = Analog bus no transaction over   12us.
 * |        |          |11 = CC pins no activity.
 * |        |          |Only after bus idle, the de-bounce   circuit starts.
 * @var UTCPD_T::AFESCTL
 * Offset: 0x8C  Analog Front End Slice Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |AFETST    |Analog   test control bits
 * |        |          |The   corresponding signal can be measured at pin u201Cabusu201D.
 * |        |          |111=internal   reference 2.6V (for internal reference trimming. Please refer to truth table   #8)
 * |        |          |110= Reserved.
 * |        |          |101=Reserved.
 * |        |          |100=internal VTOP.
 * |[4]     |TSMIDEN   |T Slice Middle Channel Enable
 * |        |          |0 = T slice middle channel is Disabled.
 * |        |          |1 = T slice moiddle channel is Enabled (equal   T_slice_en).
 * |[5]     |TSALLEN   |T Slice All Channel Enable
 * |        |          |0 = T slice all channel is Disabled.
 * |        |          |1 = T slice all channel is Enabled (equal   T_slice_all_EN).
 * @var UTCPD_T::ADTIME
 * Offset: 0x8D  TCPD PHY Auto Discharge Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ADGTM     |Auto Discharge time
 * |        |          |Default Time = 31.25us x 16 x 100 (0x16) = 49.9ms.
 * @var UTCPD_T::ADVSAFE
 * Offset: 0x8E  Auto Discharge VSAFE0V Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VSAFE0V   |Set the vSafe0V voltage level
 * |        |          |vSafe0V = Value * 25mV.
 * @var UTCPD_T::FRSVSAFE
 * Offset: 0x8F  Fast Role Swap VSAFE5V Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |VSAFE5V   |Set the vSafe5V voltage level
 * |        |          |For fast role swap voltage comparison.
 * |        |          |vSafe5V = Value * 25mV.
 * @var UTCPD_T::CCDEBTM
 * Offset: 0x90  CC Debounce Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CCDEBTM   |CC Debounce Time
 * |        |          |It is used to cc debounce time. The   default value is 100ms.
 * @var UTCPD_T::CCFILTM
 * Offset: 0x91  CC Filter Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CCFILTERTM|CC Filter Time
 * |        |          |It is used to filter the cc bus.
 * @var UTCPD_T::CCDRPTTM
 * Offset: 0x92  CC DRP Toggle Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CCDRPTTM  |CC DRP Toggle Time
 * |        |          |It is used to define the time of   drp toggle to play as UFP or DFP.
 * |        |          |T_30 = 737 (~22.5ms).
 * |        |          |T_40 = 983 (~30ms).
 * |        |          |T_50 = 1129 (~37.5ms).
 * |        |          |T_60 = 1475 (~45.0ms).
 * |        |          |T_70 = 1720 (~52.5ms).
 * @var UTCPD_T::DRPTCTL
 * Offset: 0x93  DRP Toggle Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |CCDRPTR   |CC DRP Toggle Ratio
 * |        |          |000 = Sink of CC ratio equal 50   percent.
 * |        |          |001 = Sink of CC ratio equal 40   percent.
 * |        |          |010 = Sinkl of CC ratio equal 30   percent.
 * |        |          |001 = Sink of CC ratio equal 40   percent.
 * |        |          |110 = Sink of CC ratio equal 70   percent.
 * |        |          |111 = Sink of CC ratio equal random   percen.
 * |        |          |Others = Reserved.
 * |[3]     |VBUSVSEL  |VBUS Voltage Selection
 * |        |          |0 = If VBUS monitor   VBMONI(PWR_CTL[6] is Enable and AFE ADC is disable, the VBUS   voltage is equal 0
 * |        |          |Otherwise, it select the AFE ADC averge during 4 time.
 * |        |          |1 = VBUS voltage average   during 4 times.
 * |[4]     |CLKSTOEN  |ADC Clock Stop Enable
 * |        |          |0 = ADC enter power down is   Disabled.
 * |        |          |1 = ADC enter power down is Enable.
 * |[5]     |FSTPEN    |VBUS Force Stop Enable   bit
 * |        |          |0 = VBUS force stop is   Disabled.
 * |        |          |1 = VBUS force stop is   Enable.
 * |[6]     |VBUSMEN   |VBUS Moving Enable bit
 * |        |          |0 = VBUS is average   during 4 times from ADC is Disabled.
 * |        |          |1 = VBUS is average   during 4 times from ADC is Enable.
 * |[7]     |TESTMEN   |Test Mode Enable bit
 * |        |          |0 = Test mode is Disabled.
 * |        |          |1 = Test mode is Enabled and.
 * |        |          |(i). the VBUS   auto discharge counter is 50us
 * |        |          |(ii) cc debounce   is 100ms
 * |        |          |(iii) VBUS   auto discharge fault counter is 650ms
 * |        |          |(iv) if VBUS   stop discharge thresohld > 0.8V, then the VBUS auto discharge   fault counter is 275ms..
 * @var UTCPD_T::INTFRMG
 * Offset: 0x94  TCPD Interframe Gap Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |INTFRMG   |TCPD Interframe Gap
 * |        |          |Timer period of the TCPD Interfame   Gap
 * |        |          |The default value is fixed 33us.
 * @var UTCPD_T::EHWRSTK
 * Offset: 0x95  External Hardware Reset Keep
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CTLRSTS   |Control Reset Status
 * |        |          |Respone the status of EHWRSTK   (FSWPA[4]) from 32K to 12M
 * @var UTCPD_T::SOPNGCRC
 * Offset: 0x96  SOP No Good CRC
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |SOPNGCRC  |SOP No Good CRC Response
 * |        |          |00000 = Present no good CRC.
 * |        |          |00001 = SOP Normal.
 * |        |          |00010 = SOP Primeary.
 * |        |          |00100 = SOP D Primeary.
 * |        |          |01000 = SOP debrg Primeary.
 * |        |          |10000 = SOP debrg D primeary.
 * |        |          |Others = Reserved.
 * @var UTCPD_T::VBDISSTS
 * Offset: 0x99  VBUS Disconnect Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SRCOPNLVL |CCSource Open Indicator Level Bit
 * |        |          |0 = CC Source is not open.
 * |        |          |1 = CC Source is open.
 * |[1]     |SNKOPNLVL |CCSink Open Indicator Level Bit
 * |        |          |0 = CC Sink is not open.
 * |        |          |1 = CC Sink is open.
 * |[3]     |PWRDONPHY |Power Down of PHY and Comparator
 * |        |          |0 = PHY and Comparator power down   is Disabled.
 * |        |          |1 = PHY and Comparator power down   is Enable.
 * |[5]     |BISTMALLIN|Bist Mode All Input bit
 * |        |          |0 = Bist mode all input bit is   Disabled.
 * |        |          |1 = Bist mode all input bit is   Enabled updated from received packet information.
 * |[6]     |BISTMTDIN |Bist Mode Test Data Input bit
 * |        |          |0 = Bist mode test data is   Disabled.
 * |        |          |1 = Bist mode test data is Enabled   updated from received packet information.
 * |[7]     |BISTM2IN  |Bist Mode 2 Input bit
 * |        |          |0 = Bist mode 2 input is Disabled.
 * |        |          |1 = Bist mode 2 input is Enabled   updated from received packet information.
 * @var UTCPD_T::PICFG
 * Offset: 0x9A  UCPD Power Identifial Configuration Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |CFG1ID    |PD Configuration 1 Identify Bits
 * |        |          |Indicates the idnetify bits of the   PD configuration 1.
 * |[6:5]   |CFG1RSVD  |PD Configuration 1 Reserved Bits
 * |        |          |Indicates the reserved bits of the   PD configuration 1.
 * |[7]     |CFG1IDFILL|PD Configuration 1 Identify Fill   Bit
 * |        |          |0 = PD configuration idnetify is no   fill.
 * |        |          |1 = PD configuration identify is   fill.
 * @var UTCPD_T::BMCCTL
 * Offset: 0x9B  BMC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BMCEXT    |BMC Extend Control Bit
 * |[1]     |HRINTDSCRD|HR Interrupt Discard Control Bit
 * |[2]     |CRINTDSCRD|CR Interrupt Discard Control Bit
 * |[3]     |CTL13PDDIS|PD Disable Control Bit
 * |[4]     |CTL13DETDIS|Detection Disable Bit
 * |[5]     |CTL13LB   |Loopback Bit
 * |[6]     |CTL13IGNVB|Ignore VBUS Bit
 * |[7]     |CTL13TM   |ATE Test Mode
 * @var UTCPD_T::AFEPWRSTE
 * Offset: 0x9C  AFE Power State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PHYPWR    |PHY Power State
 * |        |          |0 = PHY power is Disabled.
 * |        |          |1 = PHY power is Enabled.
 * |[3:1]   |AFEPWR    |Ananlog Frond Enad Power State
 * |        |          |Bit 1 is POR Status
 * |        |          |0 = Dead Battery circuit control   internal Rd/Rp.
 * |        |          |1 = Role Control Register   control internal Rd/Rp.
 * @var UTCPD_T::IDLETM
 * Offset: 0x9D  Idle Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |IDLETM    |Idle Time
 * |        |          |The default idle time is 6.0us
 * @var UTCPD_T::CCSTATE
 * Offset: 0x9E  CC State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |CCFSM     |CC State Machine
 * |        |          |0 = cc_state_type_maintain.
 * |        |          |1 = cc_state_type_apply_Rd.
 * |        |          |2 =cc_state_type_apply_Rp.
 * |        |          |3 =cc_state_type_p_connect_as_snk.
 * |        |          |4 = cc_state_type_p_connect_as_src.
 * |        |          |5 = cc_state_type_attached_src.
 * |        |          |6 = cc_state_type_attached_snk.
 * |        |          |7 = cc_state_type_connected_invalid.
 * |        |          |8 = cc_state_type_apply_rc.
 * |        |          |9 =   cc_state_type_disconnected_as_src.
 * |        |          |10 =   cc_state_type_disconnected_as_snk.
 * |        |          |Others = Reserved.
 * |[7:5]   |CCSTATE   |CC pin 1 state
 * |        |          |0 = cc_pin_type_cc_unknown.
 * |        |          |1 = cc_pin_type_cc_src_open.
 * |        |          |2 = cc_pin_type_cc_src_ra.
 * |        |          |3 = cc_pin_type_cc_src_rd.
 * |        |          |4 = cc_pin_type_cc_snk_rp.
 * |        |          |5 = cc_pin_type_cc_snk_open.
 * |        |          |Others = Reserved.
 * @var UTCPD_T::CCSTS1
 * Offset: 0x9F  CC Status Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |CC1STSSRC |CC1 Source Status
 * |[3:2]   |CC1STSSNK |CC1 Sink Status
 * |[5:4]   |CC2STSSRC |CC2 Source Status
 * |[7:6]   |CC2STSSNK |CC2 Sink Status
 * @var UTCPD_T::FSTXCTL
 * Offset: 0xA0  Fast Swap Signal Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FSDISCTX8 |Fast Role Swap Signal Pulse Width
 * |        |          |0 = Fast role swap transmit signal not reach 85us.
 * |        |          |1 = Fast role swap transmit signal reach 85us.
 * |[1]     |FSTXEN    |Fast Role Swap Signal Transmit Enable Bit
 * |        |          |0 = Fast role swap transmit signal is Disabled.
 * |        |          |1 = Fast role swap transmit signal is Enabled.
 * |[2]     |FSRXDETEN |Fast Role Swap Signal Receive Detection Enable Bit
 * |        |          |0 = Fast role swap receive signal is Disabled.
 * |        |          |1 = Fast role swap receive signal is Enabled.
 * |[3]     |FSRXDET   |Fast Role Swap Signal Receive Detection
 * |        |          |0 = Fast role swap receive signal is not detected.
 * |        |          |1 = Fast role swap receive signal is detected.
 * |[4]     |CC1OVDET  |CC1 Overrun Voltage Detection Input Signal
 * |        |          |0 = Indicates the CC1 voltage is not in over run   level.
 * |        |          |1 = Indicates the CC1 voltage is in over run level.
 * |[5]     |CC2OVDET  |CC2 Overrun Voltage Detection Input Signal
 * |        |          |0 = Indicates the CC2 voltage is not in over run   level.
 * |        |          |1 = Indicates the CC2 voltage is in over run level.
 * |[6]     |CC1UVVDET |CC1 Under run Voltage Detection Input Signal
 * |        |          |0 = Indicates the CC1 voltage is not in under run   level.
 * |        |          |1 = Indicates the CC1 voltage is in under run level.
 * |[7]     |CC2UVDET  |CC2 Under run Voltage Detection Input Signal
 * |        |          |0 = Indicates the CC2 voltage is not in under run   level.
 * |        |          |1 = Indicates the CC2 voltage is in under run level.
 * @var UTCPD_T::EARLYTM
 * Offset: 0xA1  Early Time Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |EARTMLO   |Early Time Low
 * |        |          |Indicated the BMC recovery early time for high   boundary
 * |[7:4]   |EARTMHI   |Early Time High
 * |        |          |Indicated the BMC recovery early time for high   boundary
 * @var UTCPD_T::DISCARD
 * Offset: 0xA3  PD Discard Gap Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DISCARDTM |PD Discard Gap Time
 * |        |          |Time = 83ns x 16 x Discard Time.
 * |        |          |Example = 83ns x 16 x 24(8u2019b00011000) = 31.87us. .
 * |        |          |It is used to cancel GoodCRC, timer start only for   GoodCRC.
 * @var UTCPD_T::CCPDSTS
 * Offset: 0xA4  CC Pull Down Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PUEN      |Pull Enable of Rp
 * |        |          |Indicates the CC pin pull enable status
 * |[4:3]   |PUDWN     |Pull Down of Rd
 * |        |          |Indicates the CC pin pull down status
 * @var UTCPD_T::BMCSTS
 * Offset: 0xA5  BMC Transmit and Receive Stateus Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |TXSTS     |Transmit State
 * |        |          |Indicates the transmit state of BMC
 * |[7:4]   |RXSTS     |Receive State
 * |        |          |Indicates the Receive State of BMC
 * @var UTCPD_T::PWRASTS
 * Offset: 0xA6  Power Control Active Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SRCENLVL  |Source Enable Bit Active Level
 * |        |          |0 = Source enable bit active High.
 * |        |          |1 = Source enable bit active Low.
 * |[1]     |SNKENLVL  |Sink Enable Bit Active Level
 * |        |          |0 = Sink enable bit active High.
 * |        |          |1 = Sink enable bit active Low.
 * |[2]     |FCDENLVL  |Force Discharge Enable Bit Active Level
 * |        |          |0 = Force discharge enable bit active High.
 * |        |          |1 = Force discharge enable bit active Low.
 * |[3]     |BDCLVL    |Bleed Discharge Enable Bit Active Level
 * |        |          |0 = Bleed discharge enable bit active High.
 * |        |          |1 = Bleed discharge enable bit active Low.
 * @var UTCPD_T::PHYCC1CMP
 * Offset: 0xA7  PHY CC1 Comparator State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CC1DETRA  |CC1_det_ra
 * |[1]     |CC1DETRD  |CC1_det_rd
 * |[2]     |CC1DETDEF |CC1_det_def
 * |[3]     |CC1DET15  |CC1_det_15
 * |[4]     |CC1DET3A  |CC1_det_3a
 * |[7:5]   |CC1STS    |CC1 State
 * |        |          |Monitor CC1 state
 * |        |          |0 = cc pin type cc unknown.
 * |        |          |1 = cc pin type cc open.
 * |        |          |2 = cc pin type cc SRC_Ra.
 * |        |          |3 = cc pin type cc SRC_Rd.
 * |        |          |4 = cc pin type cc SNK_Rp.
 * |        |          |5 = cc pin type cc SNK_Open.
 * |        |          |Others = Reserved.
 * @var UTCPD_T::PHYCC2CMP
 * Offset: 0xA8  PHY CC2 Comparator State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CC2DETRA  |CC2_det_ra
 * |[1]     |CC2DETRD  |CC2_det_rd
 * |[2]     |CC2DETDEF |CC2_det_def
 * |[3]     |CC2DET15  |CC2_det_15
 * |[4]     |CC2DET3A  |CC2_det_3a
 * |[7:5]   |CC2STS    |CC2 State
 * |        |          |Monitor CC1 state
 * |        |          |0 = cc pin type cc unknown.
 * |        |          |1 = cc pin type cc open.
 * |        |          |2 = cc pin type cc SRC_Ra.
 * |        |          |3 = cc pin type cc SRC_Rd.
 * |        |          |4 = cc pin type cc SNK_Rp.
 * |        |          |5 = cc pin type cc SNK_Open.
 * |        |          |Others = Reserved.
 * @var UTCPD_T::VDRINIT
 * Offset: 0xA9  Vendor Initial Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |PDVER     |TCPD Version
 * |        |          |Indicates the supported TCPD specification version.
 * |[3]     |DEVCAPDEF |Device Capiability Defition
 * |        |          |Default Device Capability Setting Write the Register   will update the value to Device_Cap.RoleSupport, ROLE_CONTROL and   MESSAGE_HEADER_INFO register, please refer the following table
 * |        |          |(DevCap_DEF   Table)
 * @var UTCPD_T::VDSNKDIS
 * Offset: 0xAA  Vendor Sink Disconnect Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VDSNKDISRC|Vendor Sink Disconnect Enable Bit
 * |        |          |0 = Normal.
 * |        |          |1 = RC mode.
 * |[1]     |CCSNKOPNEN|CC Sink Open Enable Bit
 * |        |          |0 = CC Sink open is Disabled.
 * |        |          |1 = CC Sink open is Enabled.
 * |[2]     |DISSNKOPNEN|Disconnect Sink Open Enable Bit
 * |        |          |0 = Disconnect Sink open is   Disabled.
 * |        |          |1 = Disconnect Sink open is   Enabled.
 * |[3]     |CONRESEN  |Connect Result Enable Bit
 * |        |          |0 = Connection result check is   Disabled.
 * |        |          |1 = Connect result check is   Enabled.
 * |[6:5]   |CCSRCOPNEN|CC Source Open Indicator
 * |        |          |000 = VBUS source   Disconnect detection.
 * |        |          |001 = CC source change to open mux1.
 * |        |          |010 = CC source change to   open mux2.
 * |        |          |011 = CC source change to   open mux3.
 * |        |          |100 = CC source change to   open mux4.
 * |        |          |101 = CC source change to   open mux5.
 * |        |          |110 = CC source change to   open mux6.
 * |        |          |111 = CC source open   indicator level.
 * @var UTCPD_T::VS0VDISLO
 * Offset: 0xAB  VSafe0V Disconnect Low Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |VBSNKDCSTS|VBUS Sink Discharge State
 * |        |          |Monitor VBUS sink discharge state
 * |        |          |0 = VBUS sink auto discharge idle.
 * |        |          |1 = VBUS sink bleed discharge start.
 * |        |          |2 = VBUS sink auto discharge start.
 * |        |          |3 = VBUS sink auto discharge stop wait   time.
 * |        |          |Others = Reserved.
 * |[5:3]   |VBSRCDCSTS|VBUS Source Discharge State
 * |        |          |Monitor VBUS source discharge state
 * |        |          |0 = VBUS source auto discharge idle.
 * |        |          |1 = VBUS source auto discharge start.
 * |        |          |2 = VBUS source auto dis discharge stop.
 * |        |          |3 = VBUS source auto discharge stop wait   time.
 * |        |          |Others = Reserved.
 * |[6]     |VBSNKDISCTHL|VBUS Voltage Sink Disconnect threshold   Lower Detection Bit
 * |        |          |0 = No detect VBUS voltage lower than VBUS_SINK_DISCONNECT_THRESHOLD.
 * |        |          |1 = Detect VBUS voltage lower than VBUS_SINK_DISCONNECT_THRESHOLD.
 * |[7]     |VBSTPDCTHL|VBUS Voltage Stop Discharge Threshold   Lower Detection Bit
 * |        |          |0 = No detect VBUS voltage lower than VBUS_STOP_DISHARGE_THRESHOLD.
 * |        |          |1 = Detect VBUS voltage lower than VBUS_STOP_DISHARGE_THRESHOLD.
 * @var UTCPD_T::VBRTOCTL
 * Offset: 0xAC  VBUS Voltage Read Time Out Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VBRTOEN   |VBUS Voltage Read Timeout Enable Bit
 * |        |          |The time out period is 10ms.
 * |        |          |0 = The VBUS voltage read timeout is   Disabled.
 * |        |          |1 = The VBUS voltage read timeout is   Enabled.
 * @var UTCPD_T::ITNERTSTS
 * Offset: 0xAD  Internal Test Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3]     |VBADCDONE |VBUS ADC Done Detection
 * |        |          |0 = VBUS adc circuit is at conversion   status.
 * |        |          |1 = VBUS adc circuit is at un-conversion   status.
 * |[4]     |VBS0VDET  |VBUS Vsafe0v Lower Detection
 * |        |          |0 = No detect vbus voltage lower than vsafe0v.
 * |        |          |1 = Detect vbus vboltage lower than vsafe0v.
 * |[5]     |ADCVUD    |ADC Voltage Reading Updated State
 * |        |          |0 = Update is on-going.
 * |        |          |1 = Update stopped.
 * |[6]     |VCONDET   |VCONN Detection
 * |        |          |0 = No detect VCONN present.
 * |        |          |1 = Detect VCONN present.
 * |[7]     |VBDET     |VBUS Detection
 * |        |          |0 = No detect VBUS present.
 * |        |          |1 = Detect VBUS present.
 * @var UTCPD_T::CTLSTS
 * Offset: 0xAE  Controller Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BISTM2    |BIST Mode 2
 * |        |          |0 = BIST2 output is no-going.
 * |        |          |1 = BIST 2 output on-going.
 * |[1]     |FOROFFVB  |Force OFF VBUS
 * |        |          |0 = No force VBUS OFF.
 * |        |          |1 = Force off VBUS from input.
 * |[2]     |CHTCKTDET |Short Circuit Detection
 * |        |          |0 = No detect short circuit.
 * |        |          |1 = Detect short circuit.
 * |[3]     |OVTMPDET  |Over Temperature Detection
 * |        |          |0 = No detect over temperature.
 * |        |          |1 = Detect over temperature.
 * |[6:4]   |RXBUFSTS  |Rx Buffer Status
 * |        |          |Rx buffer1~3u2019s status
 * |[7]     |TXBUFRDY  |Tx Buffer Ready
 * |        |          |0 = Tx buffer not empty.
 * |        |          |1 = Tx buffer ready for write.
 * @var UTCPD_T::CTLSTS2
 * Offset: 0xAF  Controller Status 2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TCACTM    |Type C Active Mode
 * |        |          |0 = Type C no active.
 * |        |          |1 = Type C in active.
 * |[3:1]   |TXPCKSTS  |Tx Package State
 * |        |          |000 = Reset.
 * |        |          |001 = Success and fail.
 * |        |          |010 = Success.
 * |        |          |011 = Fail.
 * |        |          |100 = Busy.
 * |        |          |101 = Discard.
 * |        |          |Others = Reserved.
 * |[6:4]   |RXPCKSTS  |Rx Package State
 * |        |          |000 = Reset.
 * |        |          |001 = Receive message.
 * |        |          |010 = Receive hard reset.
 * |        |          |011 = Receive cable rese.
 * |        |          |Others = Reserved.
 * |[7]     |RXPCKINBUF|Rx Package in Buffer
 * |        |          |0 = Rx Packet no ready in buffer.
 * |        |          |1 = Rx Package ready in buffer.
 * @var UTCPD_T::BMCTXBP
 * Offset: 0xCE  BMC TX Bit Period Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |BITPRD    |BMC TX Bit Period -For BMC Eye Diagram
 * |        |          |Example 12 MHz = 83.33ns.
 * |        |          |83.33ns x (39+1) = 3.33us.
 * @var UTCPD_T::BMCTXDR
 * Offset: 0xCF  BMC TX Duty Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |DUTYSET2  |BMC TX Duty Offset Parameter 2 -For BMC   eye Diagram
 * |        |          |Offset count value.
 * |        |          |Example 12 MHz = 83.33ns.
 * |[7]     |DUTYSET1  |BMC TX Duty Offset Parameter 1 -For BMC   eye Diagram
 * |        |          |0 = Increase duty offset?. (+)
 * |        |          |1 = Decrease duty offset?. (-)
 * @var UTCPD_T::SLICECR
 * Offset: 0xF9  BMC Slice Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SLICELS   |Low Level Slice Control   Selection
 * |        |          |00 = 0.18V.
 * |        |          |01 = 0.20V.
 * |        |          |10 = 0.22V.
 * |        |          |11 = 0.24V (Deafult).
 * |        |          |[Design]   control UCPDVCONN pin T_slice_l
 * |[3:2]   |SLICEHS   |High Level Slice Control   Selection
 * |        |          |00 = 0.84V (Default).
 * |        |          |01 = 0.86V.
 * |        |          |10 = 0.88V.
 * |        |          |11 = 0.90V.
 * |        |          |[Design]   control UCPDVCONN pin T_slice_h
 * |[6:4]   |SLICEMS   |Middle Level Slice   Control Selection
 * |        |          |000 = 0.48V.
 * |        |          |100 = 0.56V (Default).
 * |        |          |111 = 0.62V.
 * |        |          |Others = Reserved.
 * |        |          |[Design]   control UCPDVCONN pin T_slice_m
 */
#define NPD48_VIDL         0x0000           /*!< [0x0000] Vendor ID Low Byte Register                                      */
#define NPD48_VIDH         0x0001           /*!< [0x0001] Vendor ID High Byte Register                                     */
#define NPD48_PIDL         0x0002           /*!< [0x0002] Product ID Low Byte Register                                     */
#define NPD48_PIDH         0x0003           /*!< [0x0003] Product ID High Byte Register                                    */
#define NPD48_DIDL         0x0004           /*!< [0x0004] Device ID Low Byte Register                                      */
#define NPD48_DIDH         0x0005           /*!< [0x0005] Device ID High Byte Register                                     */
#define NPD48_UTCREVL      0x0006           /*!< [0x0006] USB Type-C Revision Low Byte Register                            */
#define NPD48_UTCREVH      0x0007           /*!< [0x0007] USB Type-C Revision High Byte Register                           */
#define NPD48_UPDREVL      0x0008           /*!< [0x0008] USB Type-C PD Revision Version Low Byte Register                 */
#define NPD48_UPDREVH      0x0009           /*!< [0x0009] USB Type-C PD Revision Version High Byte Register                */
#define NPD48_UPDCREVL     0x000a           /*!< [0x000a] USB Type-C PD Controller Revision Low Byte Register              */
#define NPD48_UPDCREVH     0x000b           /*!< [0x000b] USB Type-C PD Controller Revision High Byte Register             */
#define NPD48_ALERTL       0x0010           /*!< [0x0010] Alert Flag Low Byte Register                                     */
#define NPD48_ALERTH       0x0011           /*!< [0x0011] Alert Flag High Byte Register                                    */
#define NPD48_ALERTML      0x0012           /*!< [0x0012] Alert Mask Low Byte Register                                     */
#define NPD48_ALERTMH      0x0013           /*!< [0x0013] Alert Mask High Byte Register                                    */
#define NPD48_PWRSM        0x0014           /*!< [0x0014] Power Status Mask Control Register                               */
#define NPD48_FAULTSM      0x0015           /*!< [0x0015] Fault Status Mask Control Register                               */
#define NPD48_CFGSO        0x0018           /*!< [0x0018] Configuration Standard Output Register                           */
#define NPD48_TCPCCTL      0x0019           /*!< [0x0019] TCPC Control Register                                            */
#define NPD48_ROLECTL      0x001a           /*!< [0x001a] Role Control Register                                            */
#define NPD48_FAULTCTL     0x001b           /*!< [0x001b] Fault Control Register                                           */
#define NPD48_PWRCTL       0x001c           /*!< [0x001c] Power Control Register                                           */
#define NPD48_CCSTS        0x001d           /*!< [0x001d] CC Status Register                                               */
#define NPD48_PWRSTS       0x001e           /*!< [0x001e] Power Status Register                                            */
#define NPD48_FAULTSTS     0x001f           /*!< [0x001f] Fault Status Register                                            */
#define NPD48_CMD          0x0023           /*!< [0x0023] Command Register                                                 */
#define NPD48_DCAP1L       0x0024           /*!< [0x0024] Device Capabilities 1 Low Byte Register                          */
#define NPD48_DCAP1H       0x0025           /*!< [0x0025] Device Capabilities 1 High Byte Register                         */
#define NPD48_DCAP2L       0x0026           /*!< [0x0026] Device Capabilities 2 Low Byte Register                          */
#define NPD48_DCAP2H       0x0027           /*!< [0x0027] Device Capabilities 2 High Byte Register                         */
#define NPD48_SICAP        0x0028           /*!< [0x0028] Stand Input Capabilities Register                                */
#define NPD48_SOCAP        0x0029           /*!< [0x0029] Stand Output Capabilities Register                               */
#define NPD48_MHINFO       0x002e           /*!< [0x002e] Message Header Information Register                              */
#define NPD48_RDET         0x002f           /*!< [0x002f] Receive Detect Register                                          */
#define NPD48_RBCNT        0x0030           /*!< [0x0030] Receive Byte Count Register                                      */
#define NPD48_RBFTYP       0x0031           /*!< [0x0031] Receive Buffer Frame Type Register                               */
#define NPD48_RXBHEADL     0x0032           /*!< [0x0032] Receive Buffer Header Low Byte Register                          */
#define NPD48_RXBHEADH     0x0033           /*!< [0x0033] Receive Buffer Header High Byte Register                         */
#define NPD48_RXBUF0       0x0034           /*!< [0x0034] Receive Buffer Byte 0 Register                                   */
#define NPD48_RXBUF1       0x0035           /*!< [0x0035] Receive Buffer Byte 1 Register                                   */
#define NPD48_RXBUF2       0x0036           /*!< [0x0036] Receive Buffer Byte 2 Register                                   */
#define NPD48_RXBUF3       0x0037           /*!< [0x0037] Receive Buffer Byte 3 Register                                   */
#define NPD48_RXBUF4       0x0038           /*!< [0x0038] Receive Buffer Byte 4 Register                                   */
#define NPD48_RXBUF5       0x0039           /*!< [0x0039] Receive Buffer Byte 5 Register                                   */
#define NPD48_RXBUF6       0x003a           /*!< [0x003a] Receive Buffer Byte 6 Register                                   */
#define NPD48_RXBUF7       0x003b           /*!< [0x003b] Receive Buffer Byte 7 Register                                   */
#define NPD48_RXBUF8       0x003c           /*!< [0x003c] Receive Buffer Byte 8 Register                                   */
#define NPD48_RXBUF9       0x003d           /*!< [0x003d] Receive Buffer Byte 9 Register                                   */
#define NPD48_RXBUF10      0x003e           /*!< [0x003e] Receive Buffer Byte 10 Register                                  */
#define NPD48_RXBUF11      0x003f           /*!< [0x003f] Receive Buffer Byte 11 Register                                  */
#define NPD48_RXBUF12      0x0040           /*!< [0x0040] Receive Buffer Byte 12 Register                                  */
#define NPD48_RXBUF13      0x0041           /*!< [0x0041] Receive Buffer Byte 13 Register                                  */
#define NPD48_RXBUF14      0x0042           /*!< [0x0042] Receive Buffer Byte 14 Register                                  */
#define NPD48_RXBUF15      0x0043           /*!< [0x0043] Receive Buffer Byte 15 Register                                  */
#define NPD48_RXBUF16      0x0044           /*!< [0x0044] Receive Buffer Byte 16 Register                                  */
#define NPD48_RXBUF17      0x0045           /*!< [0x0045] Receive Buffer Byte 17 Register                                  */
#define NPD48_RXBUF18      0x0046           /*!< [0x0046] Receive Buffer Byte 18 Register                                  */
#define NPD48_RXBUF19      0x0047           /*!< [0x0047] Receive Buffer Byte 19 Register                                  */
#define NPD48_RXBUF20      0x0048           /*!< [0x0048] Receive Buffer Byte 20 Register                                  */
#define NPD48_RXBUF21      0x0049           /*!< [0x0049] Receive Buffer Byte 21 Register                                  */
#define NPD48_RXBUF22      0x004a           /*!< [0x004a] Receive Buffer Byte 22 Register                                  */
#define NPD48_RXBUF23      0x004b           /*!< [0x004b] Receive Buffer Byte 23 Register                                  */
#define NPD48_RXBUF24      0x004c           /*!< [0x004c] Receive Buffer Byte 24 Register                                  */
#define NPD48_RXBUF25      0x004d           /*!< [0x004d] Receive Buffer Byte 25 Register                                  */
#define NPD48_RXBUF26      0x004e           /*!< [0x004e] Receive Buffer Byte 26 Register                                  */
#define NPD48_RXBUF27      0x004f           /*!< [0x004f] Receive Buffer Byte 27 Register                                  */
#define NPD48_TRANSMIT     0x0050           /*!< [0x0050] Transmit Control Register                                        */
#define NPD48_TBCNT        0x0051           /*!< [0x0051] Transmit Byte Count Register                                     */
#define NPD48_TXBHEADL     0x0052           /*!< [0x0052] Transmit Buffer Header Low Byte Register                         */
#define NPD48_TXBHEADH     0x0053           /*!< [0x0053] Transmit Buffer Header High Byte Register                        */
#define NPD48_TXBUF0       0x0054           /*!< [0x0054] Transmit Buffer Byte 0 Register                                  */
#define NPD48_TXBUF1       0x0055           /*!< [0x0055] Transmit Buffer Byte 1 Register                                  */
#define NPD48_TXBUF2       0x0056           /*!< [0x0056] Transmit Buffer Byte 2 Register                                  */
#define NPD48_TXBUF3       0x0057           /*!< [0x0057] Transmit Buffer Byte 3 Register                                  */
#define NPD48_TXBUF4       0x0058           /*!< [0x0058] Transmit Buffer Byte 4 Register                                  */
#define NPD48_TXBUF5       0x0059           /*!< [0x0059] Transmit Buffer Byte 5 Register                                  */
#define NPD48_TXBUF6       0x005a           /*!< [0x005a] Transmit Buffer Byte 6 Register                                  */
#define NPD48_TXBUF7       0x005b           /*!< [0x005b] Transmit Buffer Byte 7 Register                                  */
#define NPD48_TXBUF8       0x005c           /*!< [0x005c] Transmit Buffer Byte 8 Register                                  */
#define NPD48_TXBUF9       0x005d           /*!< [0x005d] Transmit Buffer Byte 9 Register                                  */
#define NPD48_TXBUF10      0x005e           /*!< [0x005e] Transmit Buffer Byte 10 Register                                 */
#define NPD48_TXBUF11      0x005f           /*!< [0x005f] Transmit Buffer Byte 11 Register                                 */
#define NPD48_TXBUF12      0x0060           /*!< [0x0060] Transmit Buffer Byte 12 Register                                 */
#define NPD48_TXBUF13      0x0061           /*!< [0x0061] Transmit Buffer Byte 13 Register                                 */
#define NPD48_TXBUF14      0x0062           /*!< [0x0062] Transmit Buffer Byte 14 Register                                 */
#define NPD48_TXBUF15      0x0063           /*!< [0x0063] Transmit Buffer Byte 15 Register                                 */
#define NPD48_TXBUF16      0x0064           /*!< [0x0064] Transmit Buffer Byte 16 Register                                 */
#define NPD48_TXBUF17      0x0065           /*!< [0x0065] Transmit Buffer Byte 17 Register                                 */
#define NPD48_TXBUF18      0x0066           /*!< [0x0066] Transmit Buffer Byte 18 Register                                 */
#define NPD48_TXBUF19      0x0067           /*!< [0x0067] Transmit Buffer Byte 19 Register                                 */
#define NPD48_TXBUF20      0x0068           /*!< [0x0068] Transmit Buffer Byte 20 Register                                 */
#define NPD48_TXBUF21      0x0069           /*!< [0x0069] Transmit Buffer Byte 21 Register                                 */
#define NPD48_TXBUF22      0x006a           /*!< [0x006a] Transmit Buffer Byte 22 Register                                 */
#define NPD48_TXBUF23      0x006b           /*!< [0x006b] Transmit Buffer Byte 23 Register                                 */
#define NPD48_TXBUF24      0x006c           /*!< [0x006c] Transmit Buffer Byte 24 Register                                 */
#define NPD48_TXBUF25      0x006d           /*!< [0x006d] Transmit Buffer Byte 25 Register                                 */
#define NPD48_TXBUF26      0x006e           /*!< [0x006e] Transmit Buffer Byte 26 Register                                 */
#define NPD48_TXBUF27      0x006f           /*!< [0x006f] Transmit Buffer Byte 27 Register                                 */
#define NPD48_VBUSVOL      0x0070           /*!< [0x0070] VBUS Voltage Low Byte Register                                   */
#define NPD48_VBUSVOH      0x0071           /*!< [0x0071] VBUS Voltage High Byte Register                                  */
#define NPD48_VBUSDTL      0x0072           /*!< [0x0072] VBUS Sink Disconnect Threshold Low Byte Register                 */
#define NPD48_VBUSDTH      0x0073           /*!< [0x0073] VBUS Sink Disconnect Threshold High Byte Register                */
#define NPD48_VBUSTPDTL    0x0074           /*!< [0x0074] VBUS Stop Discharge Threshold Low Byte Register                  */
#define NPD48_VBUSTPDTH    0x0075           /*!< [0x0075] VBUS Stop Discharge Threshold High Byte Register                 */
#define NPD48_VBUSOVTL     0x0076           /*!< [0x0076] VBUS Voltage Alarm High CFG Low Byte Register                    */
#define NPD48_VBUSOVTH     0x0077           /*!< [0x0077] VBUS Voltage Alarm High CFG High Byte Register                   */
#define NPD48_VBUSUVTL     0x0078           /*!< [0x0078] VBUS Voltage Alarm Low CFG Low Byte Register                     */
#define NPD48_VBUSUVTH     0x0079           /*!< [0x0079] VBUS Voltage Alarm Low CFG High Byte Register                    */
#define NPD48_VBUSTHL      0x007c           /*!< [0x007c] VBUS Voltage Threshold CFG Low Byte Register                     */
#define NPD48_VBUSTHH      0x007d           /*!< [0x007d] VBUS Voltage Threshold CFG High Byte Register                    */
#define NPD48_FSASTS       0x0080           /*!< [0x0080] Fast Swap Alert Status Register                                  */
#define NPD48_DRPTRCTL     0x0083           /*!< [0x0083] DRP Toggle Ratio Control Register                                */
#define NPD48_FSAMASK      0x0084           /*!< [0x0084] Fasrt Swap Alert Mask Register                                   */
#define NPD48_FUNCTL       0x0085           /*!< [0x0085] Function Control Register                                        */
#define NPD48_BMCSCTL      0x0087           /*!< [0x0087] BMC Slice Control Register                                       */
#define NPD48_NCIDL        0x0088           /*!< [0x0088] Nuvoton Chip ID Low Byte                                         */
#define NPD48_NCIDH        0x0089           /*!< [0x0089] Nuvoton Chip ID High Byte                                        */
#define NPD48_VCDISCTL     0x008a           /*!< [0x008a] VCONN Discharge Control Register                                 */
#define NPD48_CCTRIM       0x008b           /*!< [0x008b] CC Trim Value Control Register                                   */
#define NPD48_AFESCTL      0x008c           /*!< [0x008c] Analog Front End Slice Control Register                          */
#define NPD48_ADTIME       0x008d           /*!< [0x008d] TCPD PHY Auto Discharge Time Register                            */
#define NPD48_ADVSAFE      0x008e           /*!< [0x008e] Auto Discharge VSAFE0V Register                                  */
#define NPD48_FRSVSAFE     0x008f           /*!< [0x008f] Fast Role Swap VSAFE5V Register                                  */
#define NPD48_CCDEBTM      0x0090           /*!< [0x0090] CC Debounce Time Register                                        */
#define NPD48_CCFILTM      0x0091           /*!< [0x0091] CC Filter Time Register                                          */
#define NPD48_CCDRPTTM     0x0092           /*!< [0x0092] CC DRP Toggle Time Register                                      */
#define NPD48_DRPTCTL      0x0093           /*!< [0x0093] DRP Toggle Control Register                                      */
#define NPD48_INTFRMG      0x0094           /*!< [0x0094] TCPD Interframe Gap Register                                     */
#define NPD48_EHWRSTK      0x0095           /*!< [0x0095] External Hardware Reset Keep                                     */
#define NPD48_SOPNGCRC     0x0096           /*!< [0x0096] SOP No Good CRC                                                  */
#define NPD48_VBDISSTS     0x0099           /*!< [0x0099] VBUS Disconnect Status Register                                  */
#define NPD48_PICFG        0x009a           /*!< [0x009a] UCPD Power Identifial Configuration Register                     */
#define NPD48_BMCCTL       0x009b           /*!< [0x009b] BMC Control Register                                             */
#define NPD48_AFEPWRSTE    0x009c           /*!< [0x009c] AFE Power State Register                                         */
#define NPD48_IDLETM       0x009d           /*!< [0x009d] Idle Time Register                                               */
#define NPD48_CCSTATE      0x009e           /*!< [0x009e] CC State Register                                                */
#define NPD48_CCSTS1       0x009f           /*!< [0x009f] CC Status Register 1                                             */
#define NPD48_FSTXCTL      0x00a0           /*!< [0x00a0] Fast Swap Signal Control Register                                */
#define NPD48_EARLYTM      0x00a1           /*!< [0x00a1] Early Time Register                                              */
#define NPD48_DISCARD      0x00a3           /*!< [0x00a3] PD Discard Gap Register                                          */
#define NPD48_CCPDSTS      0x00a4           /*!< [0x00a4] CC Pull Down Status Register                                     */
#define NPD48_BMCSTS       0x00a5           /*!< [0x00a5] BMC Transmit and Receive Stateus Register                        */
#define NPD48_PCACTL       0x00a6            /*!< [0x00a6] Power Control Active Control Register                            */
#define NPD48_PHYCC1CMP    0x00a7           /*!< [0x00a7] PHY CC1 Comparator State Register                                */
#define NPD48_PHYCC2CMP    0x00a8           /*!< [0x00a8] PHY CC2 Comparator State Register                                */
#define NPD48_VDRINIT      0x00a9           /*!< [0x00a9] Vendor Initial Register                                          */
#define NPD48_VDSNKDIS     0x00aa           /*!< [0x00aa] Vendor Sink Disconnect Register                                  */
#define NPD48_VS0VDISLO    0x00ab           /*!< [0x00ab] VSafe0V Disconnect Low Register                                  */
#define NPD48_VBRTOCTL     0x00ac           /*!< [0x00ac] VBUS Voltage Read Time Out Control Register                      */
#define NPD48_ITNERTSTS    0x00ad           /*!< [0x00ad] Internal Test Status Register                                    */
#define NPD48_CTLSTS       0x00ae           /*!< [0x00ae] Controller Status Register                                       */
#define NPD48_CTLSTS2      0x00af           /*!< [0x00af] Controller Status 2 Register                                     */
#define NPD48_SWRST        0x00cd           /*!< [0x00cd] Software Reset Register                                          */
#define NPD48_BMCTXBP      0x00ce           /*!< [0x00ce] BMC TX Bit Period Register                                       */
#define NPD48_BMCTXDR      0x00cf           /*!< [0x00cf] BMC TX Duty Register                                             */
#define NPD48_TCPCSFT      0x00e2           /*!< [0x00e2] TCPC State Filter Time Register                                  */
#define NPD48_PREDET1      0x00e5           /*!< [0x00e5] UTCPD Preamble Detect 1 Register                                 */
#define NPD48_PREDET2      0x00e6           /*!< [0x00e6] UTCPD Preamble Detect 2 Register                                 */
#define NPD48_PREDET3      0x00e7           /*!< [0x00e7] UTCPD Preamble Detect 3 Register                                 */
#define NPD48_PREDET4      0x00e8           /*!< [0x00e8] UTCPD Preamble Detect 4 Register                                 */
#define NPD48_NIRR1        0x00e9           /*!< [0x00e9] UTCPD Noise Immunity Record 1 Register                           */
#define NPD48_NIRR2        0x00ea           /*!< [0x00ea] UTCPD Noise Immunity Record 2 Register                           */
#define NPD48_SLICECR      0x00f9           /*!< [0x00f9] BMC Slice Control Register                                       */
#define NPD48_SLICECR2     0x00fa           /*!< [0x00fa] BMC Slice Control 2 Register                                     */

/**
    @addtogroup UTCPD_CONST UTCPD Bit Field Definition
    Constant Definitions for UTCPD Controller
@{ */

#define NPD48_VIDL_VIDL_Pos              (0)                                               /*!< UTCPD_T::VIDL: VIDL Position           */
#define NPD48_VIDL_VIDL_Msk              (0xfful << NPD48_VIDL_VIDL_Pos)                   /*!< UTCPD_T::VIDL: VIDL Mask               */

#define NPD48_VIDH_VIDH_Pos              (0)                                               /*!< UTCPD_T::VIDH: VIDH Position           */
#define NPD48_VIDH_VIDH_Msk              (0xfful << NPD48_VIDH_VIDH_Pos)                   /*!< UTCPD_T::VIDH: VIDH Mask               */

#define NPD48_PIDL_PIDL_Pos              (0)                                               /*!< UTCPD_T::PIDL: PIDL Position           */
#define NPD48_PIDL_PIDL_Msk              (0xfful << NPD48_PIDL_PIDL_Pos)                   /*!< UTCPD_T::PIDL: PIDL Mask               */

#define NPD48_PIDH_PIDH_Pos              (0)                                               /*!< UTCPD_T::PIDH: PIDH Position           */
#define NPD48_PIDH_PIDH_Msk              (0xfful << NPD48_PIDH_PIDH_Pos)                   /*!< UTCPD_T::PIDH: PIDH Mask               */

#define NPD48_DIDL_DIDL_Pos              (0)                                               /*!< UTCPD_T::DIDL: DIDL Position           */
#define NPD48_DIDL_DIDL_Msk              (0xfful << NPD48_DIDL_DIDL_Pos)                   /*!< UTCPD_T::DIDL: DIDL Mask               */

#define NPD48_DIDH_DIDH_Pos              (0)                                               /*!< UTCPD_T::DIDH: DIDH Position           */
#define NPD48_DIDH_DIDH_Msk              (0xfful << NPD48_DIDH_DIDH_Pos)                   /*!< UTCPD_T::DIDH: DIDH Mask               */

#define NPD48_UTCREVL_UTCREVL_Pos        (0)                                               /*!< UTCPD_T::UTCREVL: UTCREVL Position     */
#define NPD48_UTCREVL_UTCREVL_Msk        (0xfful << NPD48_UTCREVL_UTCREVL_Pos)             /*!< UTCPD_T::UTCREVL: UTCREVL Mask         */

#define NPD48_UTCREVH_UTCREVH_Pos        (0)                                               /*!< UTCPD_T::UTCREVH: UTCREVH Position     */
#define NPD48_UTCREVH_UTCREVH_Msk        (0xfful << NPD48_UTCREVH_UTCREVH_Pos)             /*!< UTCPD_T::UTCREVH: UTCREVH Mask         */

#define NPD48_UPDREVL_UPDREVL_Pos        (0)                                               /*!< UTCPD_T::UPDREVL: UPDREVL Position     */
#define NPD48_UPDREVL_UPDREVL_Msk        (0xfful << NPD48_UPDREVL_UPDREVL_Pos)             /*!< UTCPD_T::UPDREVL: UPDREVL Mask         */

#define NPD48_UPDREVH_UPDREVH_Pos        (0)                                               /*!< UTCPD_T::UPDREVH: UPDREVH Position     */
#define NPD48_UPDREVH_UPDREVH_Msk        (0xfful << NPD48_UPDREVH_UPDREVH_Pos)             /*!< UTCPD_T::UPDREVH: UPDREVH Mask         */

#define NPD48_UPDCREVL_UPDCREVL_Pos      (0)                                               /*!< UTCPD_T::UPDCREVL: UPDCREVL Position   */
#define NPD48_UPDCREVL_UPDCREVL_Msk      (0xfful << NPD48_UPDCREVL_UPDCREVL_Pos)           /*!< UTCPD_T::UPDCREVL: UPDCREVL Mask       */

#define NPD48_UPDCREVH_UPDCREVH_Pos      (0)                                               /*!< UTCPD_T::UPDCREVH: UPDCREVH Position   */
#define NPD48_UPDCREVH_UPDCREVH_Msk      (0xfful << NPD48_UPDCREVH_UPDCREVH_Pos)           /*!< UTCPD_T::UPDCREVH: UPDCREVH Mask       */

#define NPD48_ALERTL_CCSCHIS_Pos         (0)                                               /*!< UTCPD_T::ALERTL: CCSCHIS Position      */
#define NPD48_ALERTL_CCSCHIS_Msk         (0x1ul << NPD48_ALERTL_CCSCHIS_Pos)               /*!< UTCPD_T::ALERTL: CCSCHIS Mask          */

#define NPD48_ALERTL_PWRSCHIS_Pos        (1)                                               /*!< UTCPD_T::ALERTL: PWRSCHIS Position     */
#define NPD48_ALERTL_PWRSCHIS_Msk        (0x1ul << NPD48_ALERTL_PWRSCHIS_Pos)              /*!< UTCPD_T::ALERTL: PWRSCHIS Mask         */

#define NPD48_ALERTL_RXSOPIS_Pos         (2)                                               /*!< UTCPD_T::ALERTL: RXSOPIS Position      */
#define NPD48_ALERTL_RXSOPIS_Msk         (0x1ul << NPD48_ALERTL_RXSOPIS_Pos)               /*!< UTCPD_T::ALERTL: RXSOPIS Mask          */

#define NPD48_ALERTL_RXHRSTIS_Pos        (3)                                               /*!< UTCPD_T::ALERTL: RXHRSTIS Position     */
#define NPD48_ALERTL_RXHRSTIS_Msk        (0x1ul << NPD48_ALERTL_RXHRSTIS_Pos)              /*!< UTCPD_T::ALERTL: RXHRSTIS Mask         */

#define NPD48_ALERTL_TXFAILIS_Pos        (4)                                               /*!< UTCPD_T::ALERTL: TXFAILIS Position     */
#define NPD48_ALERTL_TXFAILIS_Msk        (0x1ul << NPD48_ALERTL_TXFAILIS_Pos)              /*!< UTCPD_T::ALERTL: TXFAILIS Mask         */

#define NPD48_ALERTL_TXDCUIS_Pos         (5)                                               /*!< UTCPD_T::ALERTL: TXDCUIS Position      */
#define NPD48_ALERTL_TXDCUIS_Msk         (0x1ul << NPD48_ALERTL_TXDCUIS_Pos)               /*!< UTCPD_T::ALERTL: TXDCUIS Mask          */

#define NPD48_ALERTL_TXOKIS_Pos          (6)                                               /*!< UTCPD_T::ALERTL: TXOKIS Position       */
#define NPD48_ALERTL_TXOKIS_Msk          (0x1ul << NPD48_ALERTL_TXOKIS_Pos)                /*!< UTCPD_T::ALERTL: TXOKIS Mask           */

#define NPD48_ALERTL_VBAMHIS_Pos         (7)                                               /*!< UTCPD_T::ALERTL: VBAMHIS Position      */
#define NPD48_ALERTL_VBAMHIS_Msk         (0x1ul << NPD48_ALERTL_VBAMHIS_Pos)               /*!< UTCPD_T::ALERTL: VBAMHIS Mask          */

#define NPD48_ALERTH_VBAMLIS_Pos         (0)                                               /*!< UTCPD_T::ALERTH: VBAMLIS Position      */
#define NPD48_ALERTH_VBAMLIS_Msk         (0x1ul << NPD48_ALERTH_VBAMLIS_Pos)               /*!< UTCPD_T::ALERTH: VBAMLIS Mask          */

#define NPD48_ALERTH_FUTIS_Pos           (1)                                               /*!< UTCPD_T::ALERTH: FUTIS Position        */
#define NPD48_ALERTH_FUTIS_Msk           (0x1ul << NPD48_ALERTH_FUTIS_Pos)                 /*!< UTCPD_T::ALERTH: FUTIS Mask            */

#define NPD48_ALERTH_RXOFIS_Pos          (2)                                               /*!< UTCPD_T::ALERTH: RXOFIS Position       */
#define NPD48_ALERTH_RXOFIS_Msk          (0x1ul << NPD48_ALERTH_RXOFIS_Pos)                /*!< UTCPD_T::ALERTH: RXOFIS Mask           */

#define NPD48_ALERTH_SKDCDTIS_Pos        (3)                                               /*!< UTCPD_T::ALERTH: SKDCDTIS Position     */
#define NPD48_ALERTH_SKDCDTIS_Msk        (0x1ul << NPD48_ALERTH_SKDCDTIS_Pos)              /*!< UTCPD_T::ALERTH: SKDCDTIS Mask         */

#define NPD48_ALERTH_PECERRIS_Pos        (6)                                               /*!< UTCPD_T::ALERTH: PECERRIS Position     */
#define NPD48_ALERTH_PECERRIS_Msk        (0x1ul << NPD48_ALERTH_PECERRIS_Pos)              /*!< UTCPD_T::ALERTH: PECERRIS Mask         */

#define NPD48_ALERTH_VNDIS_Pos           (7)                                               /*!< UTCPD_T::ALERTH: VNDIS Position        */
#define NPD48_ALERTH_VNDIS_Msk           (0x1ul << NPD48_ALERTH_VNDIS_Pos)                 /*!< UTCPD_T::ALERTH: VNDIS Mask            */

#define NPD48_ALERTML_CCSCHIE_Pos        (0)                                               /*!< UTCPD_T::ALERTML: CCSCHIE Position     */
#define NPD48_ALERTML_CCSCHIE_Msk        (0x1ul << NPD48_ALERTML_CCSCHIE_Pos)              /*!< UTCPD_T::ALERTML: CCSCHIE Mask         */

#define NPD48_ALERTML_PWRSCHIE_Pos       (1)                                               /*!< UTCPD_T::ALERTML: PWRSCHIE Position    */
#define NPD48_ALERTML_PWRSCHIE_Msk       (0x1ul << NPD48_ALERTML_PWRSCHIE_Pos)             /*!< UTCPD_T::ALERTML: PWRSCHIE Mask        */

#define NPD48_ALERTML_RXSOPIE_Pos        (2)                                               /*!< UTCPD_T::ALERTML: RXSOPIE Position     */
#define NPD48_ALERTML_RXSOPIE_Msk        (0x1ul << NPD48_ALERTML_RXSOPIE_Pos)              /*!< UTCPD_T::ALERTML: RXSOPIE Mask         */

#define NPD48_ALERTML_RXHRSTIE_Pos       (3)                                               /*!< UTCPD_T::ALERTML: RXHRSTIE Position    */
#define NPD48_ALERTML_RXHRSTIE_Msk       (0x1ul << NPD48_ALERTML_RXHRSTIE_Pos)             /*!< UTCPD_T::ALERTML: RXHRSTIE Mask        */

#define NPD48_ALERTML_TXFAILIE_Pos       (4)                                               /*!< UTCPD_T::ALERTML: TXFAILIE Position    */
#define NPD48_ALERTML_TXFAILIE_Msk       (0x1ul << NPD48_ALERTML_TXFAILIE_Pos)             /*!< UTCPD_T::ALERTML: TXFAILIE Mask        */

#define NPD48_ALERTML_TXDCUIE_Pos        (5)                                               /*!< UTCPD_T::ALERTML: TXDCUIE Position     */
#define NPD48_ALERTML_TXDCUIE_Msk        (0x1ul << NPD48_ALERTML_TXDCUIE_Pos)              /*!< UTCPD_T::ALERTML: TXDCUIE Mask         */

#define NPD48_ALERTML_TXOKIE_Pos         (6)                                               /*!< UTCPD_T::ALERTML: TXOKIE Position      */
#define NPD48_ALERTML_TXOKIE_Msk         (0x1ul << NPD48_ALERTML_TXOKIE_Pos)               /*!< UTCPD_T::ALERTML: TXOKIE Mask          */

#define NPD48_ALERTML_VBAMHIE_Pos        (7)                                               /*!< UTCPD_T::ALERTML: VBAMHIE Position     */
#define NPD48_ALERTML_VBAMHIE_Msk        (0x1ul << NPD48_ALERTML_VBAMHIE_Pos)              /*!< UTCPD_T::ALERTML: VBAMHIE Mask         */

#define NPD48_ALERTMH_VBAMLIE_Pos        (0)                                               /*!< UTCPD_T::ALERTMH: VBAMLIE Position     */
#define NPD48_ALERTMH_VBAMLIE_Msk        (0x1ul << NPD48_ALERTMH_VBAMLIE_Pos)              /*!< UTCPD_T::ALERTMH: VBAMLIE Mask         */

#define NPD48_ALERTMH_FUTIE_Pos          (1)                                               /*!< UTCPD_T::ALERTMH: FUTIE Position       */
#define NPD48_ALERTMH_FUTIE_Msk          (0x1ul << NPD48_ALERTMH_FUTIE_Pos)                /*!< UTCPD_T::ALERTMH: FUTIE Mask           */

#define NPD48_ALERTMH_RXOFIE_Pos         (2)                                               /*!< UTCPD_T::ALERTMH: RXOFIE Position      */
#define NPD48_ALERTMH_RXOFIE_Msk         (0x1ul << NPD48_ALERTMH_RXOFIE_Pos)               /*!< UTCPD_T::ALERTMH: RXOFIE Mask          */

#define NPD48_ALERTMH_SKDCDTIE_Pos       (3)                                               /*!< UTCPD_T::ALERTMH: SKDCDTIE Position    */
#define NPD48_ALERTMH_SKDCDTIE_Msk       (0x1ul << NPD48_ALERTMH_SKDCDTIE_Pos)             /*!< UTCPD_T::ALERTMH: SKDCDTIE Mask        */

#define NPD48_ALERTMH_PECERRIE_Pos       (6)                                               /*!< UTCPD_T::ALERTMH: PECERRIE Position    */
#define NPD48_ALERTMH_PECERRIE_Msk       (0x1ul << NPD48_ALERTMH_PECERRIE_Pos)             /*!< UTCPD_T::ALERTMH: PECERRIE Mask        */

#define NPD48_ALERTMH_VNDIE_Pos          (7)                                               /*!< UTCPD_T::ALERTMH: VNDIE Position       */
#define NPD48_ALERTMH_VNDIE_Msk          (0x1ul << NPD48_ALERTMH_VNDIE_Pos)                /*!< UTCPD_T::ALERTMH: VNDIE Mask           */

#define NPD48_PWRSM_SKVBIE_Pos           (0)                                               /*!< UTCPD_T::PWRSM: SKVBIE Position        */
#define NPD48_PWRSM_SKVBIE_Msk           (0x1ul << NPD48_PWRSM_SKVBIE_Pos)                 /*!< UTCPD_T::PWRSM: SKVBIE Mask            */

#define NPD48_PWRSM_VCPSIE_Pos           (1)                                               /*!< UTCPD_T::PWRSM: VCPSIE Position        */
#define NPD48_PWRSM_VCPSIE_Msk           (0x1ul << NPD48_PWRSM_VCPSIE_Pos)                 /*!< UTCPD_T::PWRSM: VCPSIE Mask            */

#define NPD48_PWRSM_VBPSIE_Pos           (2)                                               /*!< UTCPD_T::PWRSM: VBPSIE Position        */
#define NPD48_PWRSM_VBPSIE_Msk           (0x1ul << NPD48_PWRSM_VBPSIE_Pos)                 /*!< UTCPD_T::PWRSM: VBPSIE Mask            */

#define NPD48_PWRSM_VBDTDGIE_Pos         (3)                                               /*!< UTCPD_T::PWRSM: VBDTDGIE Position      */
#define NPD48_PWRSM_VBDTDGIE_Msk         (0x1ul << NPD48_PWRSM_VBDTDGIE_Pos)               /*!< UTCPD_T::PWRSM: VBDTDGIE Mask          */

#define NPD48_PWRSM_SRVBIE_Pos           (4)                                               /*!< UTCPD_T::PWRSM: SRVBIE Position        */
#define NPD48_PWRSM_SRVBIE_Msk           (0x1ul << NPD48_PWRSM_SRVBIE_Pos)                 /*!< UTCPD_T::PWRSM: SRVBIE Mask            */

#define NPD48_PWRSM_SRHVIE_Pos           (5)                                               /*!< UTCPD_T::PWRSM: SRHVIE Position        */
#define NPD48_PWRSM_SRHVIE_Msk           (0x1ul << NPD48_PWRSM_SRHVIE_Pos)                 /*!< UTCPD_T::PWRSM: SRHVIE Mask            */

#define NPD48_PWRSM_DACONIE_Pos          (7)                                               /*!< UTCPD_T::PWRSM: DACONIE Position       */
#define NPD48_PWRSM_DACONIE_Msk          (0x1ul << NPD48_PWRSM_DACONIE_Pos)                /*!< UTCPD_T::PWRSM: DACONIE Mask           */

#define NPD48_FAULTSM_I2CERRIE_Pos       (0)                                               /*!< UTCPD_T::FAULTSM: I2CERRIE Position    */
#define NPD48_FAULTSM_I2CERRIE_Msk       (0x1ul << NPD48_FAULTSM_I2CERRIE_Pos)             /*!< UTCPD_T::FAULTSM: I2CERRIE Mask        */

#define NPD48_FAULTSM_VCOCIE_Pos         (1)                                               /*!< UTCPD_T::FAULTSM: VCOCIE Position      */
#define NPD48_FAULTSM_VCOCIE_Msk         (0x1ul << NPD48_FAULTSM_VCOCIE_Pos)               /*!< UTCPD_T::FAULTSM: VCOCIE Mask          */

#define NPD48_FAULTSM_VBOVIE_Pos         (2)                                               /*!< UTCPD_T::FAULTSM: VBOVIE Position      */
#define NPD48_FAULTSM_VBOVIE_Msk         (0x1ul << NPD48_FAULTSM_VBOVIE_Pos)               /*!< UTCPD_T::FAULTSM: VBOVIE Mask          */

#define NPD48_FAULTSM_VBOCIE_Pos         (3)                                               /*!< UTCPD_T::FAULTSM: VBOCIE Position      */
#define NPD48_FAULTSM_VBOCIE_Msk         (0x1ul << NPD48_FAULTSM_VBOCIE_Pos)               /*!< UTCPD_T::FAULTSM: VBOCIE Mask          */

#define NPD48_FAULTSM_FDGFALIE_Pos       (4)                                               /*!< UTCPD_T::FAULTSM: FDGFALIE Position    */
#define NPD48_FAULTSM_FDGFALIE_Msk       (0x1ul << NPD48_FAULTSM_FDGFALIE_Pos)             /*!< UTCPD_T::FAULTSM: FDGFALIE Mask        */

#define NPD48_FAULTSM_ADGFALIE_Pos       (5)                                               /*!< UTCPD_T::FAULTSM: ADGFALIE Position    */
#define NPD48_FAULTSM_ADGFALIE_Msk       (0x1ul << NPD48_FAULTSM_ADGFALIE_Pos)             /*!< UTCPD_T::FAULTSM: ADGFALIE Mask        */

#define NPD48_FAULTSM_FOFFVBIE_Pos       (6)                                               /*!< UTCPD_T::FAULTSM: FOFFVBIE Position    */
#define NPD48_FAULTSM_FOFFVBIE_Msk       (0x1ul << NPD48_FAULTSM_FOFFVBIE_Pos)             /*!< UTCPD_T::FAULTSM: FOFFVBIE Mask        */

#define NPD48_CFGSO_TCCO_Pos             (0)                                               /*!< UTCPD_T::CFGSO: TCCO Position          */
#define NPD48_CFGSO_TCCO_Msk             (0x1ul << NPD48_CFGSO_TCCO_Pos)                   /*!< UTCPD_T::CFGSO: TCCO Mask              */

#define NPD48_CFGSO_AUDIOAC_Pos          (5)                                               /*!< UTCPD_T::CFGSO: AUDIOAC Position       */
#define NPD48_CFGSO_AUDIOAC_Msk          (0x1ul << NPD48_CFGSO_AUDIOAC_Pos)                /*!< UTCPD_T::CFGSO: AUDIOAC Mask           */

#define NPD48_CFGSO_DBGAC_Pos            (6)                                               /*!< UTCPD_T::CFGSO: DBGAC Position         */
#define NPD48_CFGSO_DBGAC_Msk            (0x1ul << NPD48_CFGSO_DBGAC_Pos)                  /*!< UTCPD_T::CFGSO: DBGAC Mask             */

#define NPD48_CFGSO_HIOUT_Pos            (7)                                               /*!< UTCPD_T::CFGSO: HIOUT Position         */
#define NPD48_CFGSO_HIOUT_Msk            (0x1ul << NPD48_CFGSO_HIOUT_Pos)                  /*!< UTCPD_T::CFGSO: HIOUT Mask             */

#define NPD48_TCPCCTL_ORIENT_Pos         (0)                                               /*!< UTCPD_T::TCPCCTL: ORIENT Position      */
#define NPD48_TCPCCTL_ORIENT_Msk         (0x1ul << NPD48_TCPCCTL_ORIENT_Pos)               /*!< UTCPD_T::TCPCCTL: ORIENT Mask          */

#define NPD48_TCPCCTL_BISTEN_Pos         (1)                                               /*!< UTCPD_T::TCPCCTL: BISTEN Position      */
#define NPD48_TCPCCTL_BISTEN_Msk         (0x1ul << NPD48_TCPCCTL_BISTEN_Pos)               /*!< UTCPD_T::TCPCCTL: BISTEN Mask          */

#define NPD48_ROLECTL_CC1_Pos            (0)                                               /*!< UTCPD_T::ROLECTL: CC1 Position         */
#define NPD48_ROLECTL_CC1_Msk            (0x3ul << NPD48_ROLECTL_CC1_Pos)                  /*!< UTCPD_T::ROLECTL: CC1 Mask             */

#define NPD48_ROLECTL_CC2_Pos            (2)                                               /*!< UTCPD_T::ROLECTL: CC2 Position         */
#define NPD48_ROLECTL_CC2_Msk            (0x3ul << NPD48_ROLECTL_CC2_Pos)                  /*!< UTCPD_T::ROLECTL: CC2 Mask             */

#define NPD48_ROLECTL_RPVALUE_Pos        (4)                                               /*!< UTCPD_T::ROLECTL: RPVALUE Position     */
#define NPD48_ROLECTL_RPVALUE_Msk        (0x3ul << NPD48_ROLECTL_RPVALUE_Pos)              /*!< UTCPD_T::ROLECTL: RPVALUE Mask         */

#define NPD48_ROLECTL_DRP_Pos            (6)                                               /*!< UTCPD_T::ROLECTL: DRP Position         */
#define NPD48_ROLECTL_DRP_Msk            (0x1ul << NPD48_ROLECTL_DRP_Pos)                  /*!< UTCPD_T::ROLECTL: DRP Mask             */

#define NPD48_FAULTCTL_VCOCDTDS_Pos      (0)                                               /*!< UTCPD_T::FAULTCTL: VCOCDTDS Position   */
#define NPD48_FAULTCTL_VCOCDTDS_Msk      (0x1ul << NPD48_FAULTCTL_VCOCDTDS_Pos)            /*!< UTCPD_T::FAULTCTL: VCOCDTDS Mask       */

#define NPD48_FAULTCTL_VBOVDTDS_Pos      (1)                                               /*!< UTCPD_T::FAULTCTL: VBOVDTDS Position   */
#define NPD48_FAULTCTL_VBOVDTDS_Msk      (0x1ul << NPD48_FAULTCTL_VBOVDTDS_Pos)            /*!< UTCPD_T::FAULTCTL: VBOVDTDS Mask       */

#define NPD48_FAULTCTL_VBOCDTDS_Pos      (2)                                               /*!< UTCPD_T::FAULTCTL: VBOCDTDS Position   */
#define NPD48_FAULTCTL_VBOCDTDS_Msk      (0x1ul << NPD48_FAULTCTL_VBOCDTDS_Pos)            /*!< UTCPD_T::FAULTCTL: VBOCDTDS Mask       */

#define NPD48_FAULTCTL_VBDGTMDS_Pos      (3)                                               /*!< UTCPD_T::FAULTCTL: VBDGTMDS Position   */
#define NPD48_FAULTCTL_VBDGTMDS_Msk      (0x1ul << NPD48_FAULTCTL_VBDGTMDS_Pos)            /*!< UTCPD_T::FAULTCTL: VBDGTMDS Mask       */

#define NPD48_FAULTCTL_FOFFVBDS_Pos      (4)                                               /*!< UTCPD_T::FAULTCTL: FOFFVBDS Position   */
#define NPD48_FAULTCTL_FOFFVBDS_Msk      (0x1ul << NPD48_FAULTCTL_FOFFVBDS_Pos)            /*!< UTCPD_T::FAULTCTL: FOFFVBDS Mask       */

#define NPD48_PWRCTL_VCEN_Pos            (0)                                               /*!< UTCPD_T::PWRCTL: VCEN Position         */
#define NPD48_PWRCTL_VCEN_Msk            (0x1ul << NPD48_PWRCTL_VCEN_Pos)                  /*!< UTCPD_T::PWRCTL: VCEN Mask             */

#define NPD48_PWRCTL_VCPWR_Pos           (1)                                               /*!< UTCPD_T::PWRCTL: VCPWR Position        */
#define NPD48_PWRCTL_VCPWR_Msk           (0x1ul << NPD48_PWRCTL_VCPWR_Pos)                 /*!< UTCPD_T::PWRCTL: VCPWR Mask            */

#define NPD48_PWRCTL_FDGEN_Pos           (2)                                               /*!< UTCPD_T::PWRCTL: FDGEN Position        */
#define NPD48_PWRCTL_FDGEN_Msk           (0x1ul << NPD48_PWRCTL_FDGEN_Pos)                 /*!< UTCPD_T::PWRCTL: FDGEN Mask            */

#define NPD48_PWRCTL_BDGEN_Pos           (3)                                               /*!< UTCPD_T::PWRCTL: BDGEN Position        */
#define NPD48_PWRCTL_BDGEN_Msk           (0x1ul << NPD48_PWRCTL_BDGEN_Pos)                 /*!< UTCPD_T::PWRCTL: BDGEN Mask            */

#define NPD48_PWRCTL_ADGDC_Pos           (4)                                               /*!< UTCPD_T::PWRCTL: ADGDC Position        */
#define NPD48_PWRCTL_ADGDC_Msk           (0x1ul << NPD48_PWRCTL_ADGDC_Pos)                 /*!< UTCPD_T::PWRCTL: ADGDC Mask            */

#define NPD48_PWRCTL_DSVBAM_Pos          (5)                                               /*!< UTCPD_T::PWRCTL: DSVBAM Position       */
#define NPD48_PWRCTL_DSVBAM_Msk          (0x1ul << NPD48_PWRCTL_DSVBAM_Pos)                /*!< UTCPD_T::PWRCTL: DSVBAM Mask           */

#define NPD48_PWRCTL_VBMONI_Pos          (6)                                               /*!< UTCPD_T::PWRCTL: VBMONI Position       */
#define NPD48_PWRCTL_VBMONI_Msk          (0x1ul << NPD48_PWRCTL_VBMONI_Pos)                /*!< UTCPD_T::PWRCTL: VBMONI Mask           */

#define NPD48_CCSTS_CC1STATE_Pos         (0)                                               /*!< UTCPD_T::CCSTS: CC1STATE Position      */
#define NPD48_CCSTS_CC1STATE_Msk         (0x3ul << NPD48_CCSTS_CC1STATE_Pos)               /*!< UTCPD_T::CCSTS: CC1STATE Mask          */

#define NPD48_CCSTS_CC2STATE_Pos         (2)                                               /*!< UTCPD_T::CCSTS: CC2STATE Position      */
#define NPD48_CCSTS_CC2STATE_Msk         (0x3ul << NPD48_CCSTS_CC2STATE_Pos)               /*!< UTCPD_T::CCSTS: CC2STATE Mask          */

#define NPD48_CCSTS_CONRLT_Pos           (4)                                               /*!< UTCPD_T::CCSTS: CONRLT Position        */
#define NPD48_CCSTS_CONRLT_Msk           (0x1ul << NPD48_CCSTS_CONRLT_Pos)                 /*!< UTCPD_T::CCSTS: CONRLT Mask            */

#define NPD48_CCSTS_LK4CONN_Pos          (5)                                               /*!< UTCPD_T::CCSTS: LK4CONN Position       */
#define NPD48_CCSTS_LK4CONN_Msk          (0x1ul << NPD48_CCSTS_LK4CONN_Pos)                /*!< UTCPD_T::CCSTS: LK4CONN Mask           */

#define NPD48_PWRSTS_SKVB_Pos            (0)                                               /*!< UTCPD_T::PWRSTS: SKVB Position         */
#define NPD48_PWRSTS_SKVB_Msk            (0x1ul << NPD48_PWRSTS_SKVB_Pos)                  /*!< UTCPD_T::PWRSTS: SKVB Mask             */

#define NPD48_PWRSTS_VCPS_Pos            (1)                                               /*!< UTCPD_T::PWRSTS: VCPS Position         */
#define NPD48_PWRSTS_VCPS_Msk            (0x1ul << NPD48_PWRSTS_VCPS_Pos)                  /*!< UTCPD_T::PWRSTS: VCPS Mask             */

#define NPD48_PWRSTS_VBPS_Pos            (2)                                               /*!< UTCPD_T::PWRSTS: VBPS Position         */
#define NPD48_PWRSTS_VBPS_Msk            (0x1ul << NPD48_PWRSTS_VBPS_Pos)                  /*!< UTCPD_T::PWRSTS: VBPS Mask             */

#define NPD48_PWRSTS_VBPSDTEN_Pos        (3)                                               /*!< UTCPD_T::PWRSTS: VBPSDTEN Position     */
#define NPD48_PWRSTS_VBPSDTEN_Msk        (0x1ul << NPD48_PWRSTS_VBPSDTEN_Pos)              /*!< UTCPD_T::PWRSTS: VBPSDTEN Mask         */

#define NPD48_PWRSTS_SRVB_Pos            (4)                                               /*!< UTCPD_T::PWRSTS: SRVB Position         */
#define NPD48_PWRSTS_SRVB_Msk            (0x1ul << NPD48_PWRSTS_SRVB_Pos)                  /*!< UTCPD_T::PWRSTS: SRVB Mask             */

#define NPD48_PWRSTS_SRHV_Pos            (5)                                               /*!< UTCPD_T::PWRSTS: SRHV Position         */
#define NPD48_PWRSTS_SRHV_Msk            (0x1ul << NPD48_PWRSTS_SRHV_Pos)                  /*!< UTCPD_T::PWRSTS: SRHV Mask             */

#define NPD48_PWRSTS_DACON_Pos           (7)                                               /*!< UTCPD_T::PWRSTS: DACON Position        */
#define NPD48_PWRSTS_DACON_Msk           (0x1ul << NPD48_PWRSTS_DACON_Pos)                 /*!< UTCPD_T::PWRSTS: DACON Mask            */

#define NPD48_FAULTSTS_I2CIFERR_Pos      (0)                                               /*!< UTCPD_T::FAULTSTS: I2CIFERR Position   */
#define NPD48_FAULTSTS_I2CIFERR_Msk      (0x1ul << NPD48_FAULTSTS_I2CIFERR_Pos)            /*!< UTCPD_T::FAULTSTS: I2CIFERR Mask       */

#define NPD48_FAULTSTS_VCOCFUT_Pos       (1)                                               /*!< UTCPD_T::FAULTSTS: VCOCFUT Position    */
#define NPD48_FAULTSTS_VCOCFUT_Msk       (0x1ul << NPD48_FAULTSTS_VCOCFUT_Pos)             /*!< UTCPD_T::FAULTSTS: VCOCFUT Mask        */

#define NPD48_FAULTSTS_VBOVFUT_Pos       (2)                                               /*!< UTCPD_T::FAULTSTS: VBOVFUT Position    */
#define NPD48_FAULTSTS_VBOVFUT_Msk       (0x1ul << NPD48_FAULTSTS_VBOVFUT_Pos)             /*!< UTCPD_T::FAULTSTS: VBOVFUT Mask        */

#define NPD48_FAULTSTS_VBOCFUT_Pos       (3)                                               /*!< UTCPD_T::FAULTSTS: VBOCFUT Position    */
#define NPD48_FAULTSTS_VBOCFUT_Msk       (0x1ul << NPD48_FAULTSTS_VBOCFUT_Pos)             /*!< UTCPD_T::FAULTSTS: VBOCFUT Mask        */

#define NPD48_FAULTSTS_FDGFAL_Pos        (4)                                               /*!< UTCPD_T::FAULTSTS: FDGFAL Position     */
#define NPD48_FAULTSTS_FDGFAL_Msk        (0x1ul << NPD48_FAULTSTS_FDGFAL_Pos)              /*!< UTCPD_T::FAULTSTS: FDGFAL Mask         */

#define NPD48_FAULTSTS_ADGFAL_Pos        (5)                                               /*!< UTCPD_T::FAULTSTS: ADGFAL Position     */
#define NPD48_FAULTSTS_ADGFAL_Msk        (0x1ul << NPD48_FAULTSTS_ADGFAL_Pos)              /*!< UTCPD_T::FAULTSTS: ADGFAL Mask         */

#define NPD48_FAULTSTS_FOFFVB_Pos        (6)                                               /*!< UTCPD_T::FAULTSTS: FOFFVB Position     */
#define NPD48_FAULTSTS_FOFFVB_Msk        (0x1ul << NPD48_FAULTSTS_FOFFVB_Pos)              /*!< UTCPD_T::FAULTSTS: FOFFVB Mask         */

#define NPD48_CMD_CMD_Pos                (0)                                               /*!< UTCPD_T::CMD: CMD Position             */
#define NPD48_CMD_CMD_Msk                (0xfful << NPD48_CMD_CMD_Pos)                     /*!< UTCPD_T::CMD: CMD Mask                 */

#define NPD48_DCAP1L_CPSRVB_Pos          (0)                                               /*!< UTCPD_T::DCAP1L: CPSRVB Position       */
#define NPD48_DCAP1L_CPSRVB_Msk          (0x1ul << NPD48_DCAP1L_CPSRVB_Pos)                /*!< UTCPD_T::DCAP1L: CPSRVB Mask           */

#define NPD48_DCAP1L_CPSRHV_Pos          (1)                                               /*!< UTCPD_T::DCAP1L: CPSRHV Position       */
#define NPD48_DCAP1L_CPSRHV_Msk          (0x1ul << NPD48_DCAP1L_CPSRHV_Pos)                /*!< UTCPD_T::DCAP1L: CPSRHV Mask           */

#define NPD48_DCAP1L_CPSKVB_Pos          (2)                                               /*!< UTCPD_T::DCAP1L: CPSKVB Position       */
#define NPD48_DCAP1L_CPSKVB_Msk          (0x1ul << NPD48_DCAP1L_CPSKVB_Pos)                /*!< UTCPD_T::DCAP1L: CPSKVB Mask           */

#define NPD48_DCAP1L_CPSRVC_Pos          (3)                                               /*!< UTCPD_T::DCAP1L: CPSRVC Position       */
#define NPD48_DCAP1L_CPSRVC_Msk          (0x1ul << NPD48_DCAP1L_CPSRVC_Pos)                /*!< UTCPD_T::DCAP1L: CPSRVC Mask           */

#define NPD48_DCAP1L_CPSDBG_Pos          (4)                                               /*!< UTCPD_T::DCAP1L: CPSDBG Position       */
#define NPD48_DCAP1L_CPSDBG_Msk          (0x1ul << NPD48_DCAP1L_CPSDBG_Pos)                /*!< UTCPD_T::DCAP1L: CPSDBG Mask           */

#define NPD48_DCAP1L_CPROL_Pos           (5)                                               /*!< UTCPD_T::DCAP1L: CPROL Position        */
#define NPD48_DCAP1L_CPROL_Msk           (0x7ul << NPD48_DCAP1L_CPROL_Pos)                 /*!< UTCPD_T::DCAP1L: CPROL Mask            */

#define NPD48_DCAP1H_CPSRRE_Pos          (0)                                               /*!< UTCPD_T::DCAP1H: CPSRRE Position       */
#define NPD48_DCAP1H_CPSRRE_Msk          (0x3ul << NPD48_DCAP1H_CPSRRE_Pos)                /*!< UTCPD_T::DCAP1H: CPSRRE Mask           */

#define NPD48_DCAP1H_CPVBAM_Pos          (2)                                               /*!< UTCPD_T::DCAP1H: CPVBAM Position       */
#define NPD48_DCAP1H_CPVBAM_Msk          (0x1ul << NPD48_DCAP1H_CPVBAM_Pos)                /*!< UTCPD_T::DCAP1H: CPVBAM Mask           */

#define NPD48_DCAP1H_CPFDG_Pos           (3)                                               /*!< UTCPD_T::DCAP1H: CPFDG Position        */
#define NPD48_DCAP1H_CPFDG_Msk           (0x1ul << NPD48_DCAP1H_CPFDG_Pos)                 /*!< UTCPD_T::DCAP1H: CPFDG Mask            */

#define NPD48_DCAP1H_CPBDG_Pos           (4)                                               /*!< UTCPD_T::DCAP1H: CPBDG Position        */
#define NPD48_DCAP1H_CPBDG_Msk           (0x1ul << NPD48_DCAP1H_CPBDG_Pos)                 /*!< UTCPD_T::DCAP1H: CPBDG Mask            */

#define NPD48_DCAP1H_CPVBOVP_Pos         (5)                                               /*!< UTCPD_T::DCAP1H: CPVBOVP Position      */
#define NPD48_DCAP1H_CPVBOVP_Msk         (0x1ul << NPD48_DCAP1H_CPVBOVP_Pos)               /*!< UTCPD_T::DCAP1H: CPVBOVP Mask          */

#define NPD48_DCAP1H_CPVBOCP_Pos         (6)                                               /*!< UTCPD_T::DCAP1H: CPVBOCP Position      */
#define NPD48_DCAP1H_CPVBOCP_Msk         (0x1ul << NPD48_DCAP1H_CPVBOCP_Pos)               /*!< UTCPD_T::DCAP1H: CPVBOCP Mask          */

#define NPD48_DCAP2L_CPVCOC_Pos          (0)                                               /*!< UTCPD_T::DCAP2L: CPVCOC Position       */
#define NPD48_DCAP2L_CPVCOC_Msk          (0x1ul << NPD48_DCAP2L_CPVCOC_Pos)                /*!< UTCPD_T::DCAP2L: CPVCOC Mask           */

#define NPD48_DCAP2L_CPVCPWR_Pos         (1)                                               /*!< UTCPD_T::DCAP2L: CPVCPWR Position      */
#define NPD48_DCAP2L_CPVCPWR_Msk         (0x7ul << NPD48_DCAP2L_CPVCPWR_Pos)               /*!< UTCPD_T::DCAP2L: CPVCPWR Mask          */

#define NPD48_DCAP2L_CPVBAMLS_Pos        (4)                                               /*!< UTCPD_T::DCAP2L: CPVBAMLS Position     */
#define NPD48_DCAP2L_CPVBAMLS_Msk        (0x3ul << NPD48_DCAP2L_CPVBAMLS_Pos)              /*!< UTCPD_T::DCAP2L: CPVBAMLS Mask         */

#define NPD48_DCAP2L_CPSPDGTH_Pos        (6)                                               /*!< UTCPD_T::DCAP2L: CPSPDGTH Position     */
#define NPD48_DCAP2L_CPSPDGTH_Msk        (0x1ul << NPD48_DCAP2L_CPSPDGTH_Pos)              /*!< UTCPD_T::DCAP2L: CPSPDGTH Mask         */

#define NPD48_DCAP2L_CPSKDCDT_Pos        (7)                                               /*!< UTCPD_T::DCAP2L: CPSKDCDT Position     */
#define NPD48_DCAP2L_CPSKDCDT_Msk        (0x1ul << NPD48_DCAP2L_CPSKDCDT_Pos)              /*!< UTCPD_T::DCAP2L: CPSKDCDT Mask         */

#define NPD48_SICAP_CPVCOC_Pos           (0)                                               /*!< UTCPD_T::SICAP: CPVCOC Position        */
#define NPD48_SICAP_CPVCOC_Msk           (0x1ul << NPD48_SICAP_CPVCOC_Pos)                 /*!< UTCPD_T::SICAP: CPVCOC Mask            */

#define NPD48_SICAP_CPVCPWR_Pos          (1)                                               /*!< UTCPD_T::SICAP: CPVCPWR Position       */
#define NPD48_SICAP_CPVCPWR_Msk          (0x1ul << NPD48_SICAP_CPVCPWR_Pos)                /*!< UTCPD_T::SICAP: CPVCPWR Mask           */

#define NPD48_SICAP_VBEOVF_Pos           (2)                                               /*!< UTCPD_T::SICAP: VBEOVF Position        */
#define NPD48_SICAP_VBEOVF_Msk           (0x1ul << NPD48_SICAP_VBEOVF_Pos)                 /*!< UTCPD_T::SICAP: VBEOVF Mask            */

#define NPD48_SOCAP_CONORI_Pos           (0)                                               /*!< UTCPD_T::SOCAP: CONORI Position        */
#define NPD48_SOCAP_CONORI_Msk           (0x1ul << NPD48_SOCAP_CONORI_Pos)                 /*!< UTCPD_T::SOCAP: CONORI Mask            */

#define NPD48_SOCAP_CONRES_Pos           (1)                                               /*!< UTCPD_T::SOCAP: CONRES Position        */
#define NPD48_SOCAP_CONRES_Msk           (0x1ul << NPD48_SOCAP_CONRES_Pos)                 /*!< UTCPD_T::SOCAP: CONRES Mask            */

#define NPD48_SOCAP_MUXCINF_Pos          (2)                                               /*!< UTCPD_T::SOCAP: MUXCINF Position       */
#define NPD48_SOCAP_MUXCINF_Msk          (0x1ul << NPD48_SOCAP_MUXCINF_Pos)                /*!< UTCPD_T::SOCAP: MUXCINF Mask           */

#define NPD48_SOCAP_ACIND_Pos            (3)                                               /*!< UTCPD_T::SOCAP: ACIND Position         */
#define NPD48_SOCAP_ACIND_Msk            (0x1ul << NPD48_SOCAP_ACIND_Pos)                  /*!< UTCPD_T::SOCAP: ACIND Mask             */

#define NPD48_SOCAP_AAAIND_Pos           (4)                                               /*!< UTCPD_T::SOCAP: AAAIND Position        */
#define NPD48_SOCAP_AAAIND_Msk           (0x1ul << NPD48_SOCAP_AAAIND_Pos)                 /*!< UTCPD_T::SOCAP: AAAIND Mask            */

#define NPD48_SOCAP_VPMON_Pos            (5)                                               /*!< UTCPD_T::SOCAP: VPMON Position         */
#define NPD48_SOCAP_VPMON_Msk            (0x1ul << NPD48_SOCAP_VPMON_Pos)                  /*!< UTCPD_T::SOCAP: VPMON Mask             */

#define NPD48_SOCAP_DAIND_Pos            (6)                                               /*!< UTCPD_T::SOCAP: DAIND Position         */
#define NPD48_SOCAP_DAIND_Msk            (0x1ul << NPD48_SOCAP_DAIND_Pos)                  /*!< UTCPD_T::SOCAP: DAIND Mask             */

#define NPD48_MHINFO_PROLE_Pos           (0)                                               /*!< UTCPD_T::MHINFO: PROLE Position        */
#define NPD48_MHINFO_PROLE_Msk           (0x1ul << NPD48_MHINFO_PROLE_Pos)                 /*!< UTCPD_T::MHINFO: PROLE Mask            */

#define NPD48_MHINFO_PDREV_Pos           (1)                                               /*!< UTCPD_T::MHINFO: PDREV Position        */
#define NPD48_MHINFO_PDREV_Msk           (0x3ul << NPD48_MHINFO_PDREV_Pos)                 /*!< UTCPD_T::MHINFO: PDREV Mask            */

#define NPD48_MHINFO_DROLE_Pos           (3)                                               /*!< UTCPD_T::MHINFO: DROLE Position        */
#define NPD48_MHINFO_DROLE_Msk           (0x1ul << NPD48_MHINFO_DROLE_Pos)                 /*!< UTCPD_T::MHINFO: DROLE Mask            */

#define NPD48_MHINFO_CPLUG_Pos           (4)                                               /*!< UTCPD_T::MHINFO: CPLUG Position        */
#define NPD48_MHINFO_CPLUG_Msk           (0x1ul << NPD48_MHINFO_CPLUG_Pos)                 /*!< UTCPD_T::MHINFO: CPLUG Mask            */

#define NPD48_RDET_SOPEN_Pos             (0)                                               /*!< UTCPD_T::RDET: SOPEN Position          */
#define NPD48_RDET_SOPEN_Msk             (0x1ul << NPD48_RDET_SOPEN_Pos)                   /*!< UTCPD_T::RDET: SOPEN Mask              */

#define NPD48_RDET_SOPPEN_Pos            (1)                                               /*!< UTCPD_T::RDET: SOPPEN Position         */
#define NPD48_RDET_SOPPEN_Msk            (0x1ul << NPD48_RDET_SOPPEN_Pos)                  /*!< UTCPD_T::RDET: SOPPEN Mask             */

#define NPD48_RDET_SOPPPEN_Pos           (2)                                               /*!< UTCPD_T::RDET: SOPPPEN Position        */
#define NPD48_RDET_SOPPPEN_Msk           (0x1ul << NPD48_RDET_SOPPPEN_Pos)                 /*!< UTCPD_T::RDET: SOPPPEN Mask            */

#define NPD48_RDET_SDBGPEN_Pos           (3)                                               /*!< UTCPD_T::RDET: SDBGPEN Position        */
#define NPD48_RDET_SDBGPEN_Msk           (0x1ul << NPD48_RDET_SDBGPEN_Pos)                 /*!< UTCPD_T::RDET: SDBGPEN Mask            */

#define NPD48_RDET_SDBGPPEN_Pos          (4)                                               /*!< UTCPD_T::RDET: SDBGPPEN Position       */
#define NPD48_RDET_SDBGPPEN_Msk          (0x1ul << NPD48_RDET_SDBGPPEN_Pos)                /*!< UTCPD_T::RDET: SDBGPPEN Mask           */

#define NPD48_RDET_HRSTEN_Pos            (5)                                               /*!< UTCPD_T::RDET: HRSTEN Position         */
#define NPD48_RDET_HRSTEN_Msk            (0x1ul << NPD48_RDET_HRSTEN_Pos)                  /*!< UTCPD_T::RDET: HRSTEN Mask             */

#define NPD48_RDET_CABRSTEN_Pos          (6)                                               /*!< UTCPD_T::RDET: CABRSTEN Position       */
#define NPD48_RDET_CABRSTEN_Msk          (0x1ul << NPD48_RDET_CABRSTEN_Pos)                /*!< UTCPD_T::RDET: CABRSTEN Mask           */

#define NPD48_RBCNT_RXBCNT_Pos           (0)                                               /*!< UTCPD_T::RBCNT: RXBCNT Position        */
#define NPD48_RBCNT_RXBCNT_Msk           (0xfful << NPD48_RBCNT_RXBCNT_Pos)                /*!< UTCPD_T::RBCNT: RXBCNT Mask            */

#define NPD48_RBFTYP_RXFTYPE_Pos         (0)                                               /*!< UTCPD_T::RBFTYP: RXFTYPE Position      */
#define NPD48_RBFTYP_RXFTYPE_Msk         (0x7ul << NPD48_RBFTYP_RXFTYPE_Pos)               /*!< UTCPD_T::RBFTYP: RXFTYPE Mask          */

#define NPD48_RXBHEADL_RXHEAD_Pos        (0)                                               /*!< UTCPD_T::RXBHEADL: RXHEAD Position     */
#define NPD48_RXBHEADL_RXHEAD_Msk        (0xfful << NPD48_RXBHEADL_RXHEAD_Pos)             /*!< UTCPD_T::RXBHEADL: RXHEAD Mask         */

#define NPD48_RXBHEADH_RXHEAD_Pos        (0)                                               /*!< UTCPD_T::RXBHEADH: RXHEAD Position     */
#define NPD48_RXBHEADH_RXHEAD_Msk        (0xfful << NPD48_RXBHEADH_RXHEAD_Pos)             /*!< UTCPD_T::RXBHEADH: RXHEAD Mask         */

#define NPD48_RXBUF0_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF0: RXBUF Position        */
#define NPD48_RXBUF0_RXBUF_Msk           (0xfful << NPD48_RXBUF0_RXBUF_Pos)                /*!< UTCPD_T::RXBUF0: RXBUF Mask            */

#define NPD48_RXBUF1_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF1: RXBUF Position        */
#define NPD48_RXBUF1_RXBUF_Msk           (0xfful << NPD48_RXBUF1_RXBUF_Pos)                /*!< UTCPD_T::RXBUF1: RXBUF Mask            */

#define NPD48_RXBUF2_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF2: RXBUF Position        */
#define NPD48_RXBUF2_RXBUF_Msk           (0xfful << NPD48_RXBUF2_RXBUF_Pos)                /*!< UTCPD_T::RXBUF2: RXBUF Mask            */

#define NPD48_RXBUF3_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF3: RXBUF Position        */
#define NPD48_RXBUF3_RXBUF_Msk           (0xfful << NPD48_RXBUF3_RXBUF_Pos)                /*!< UTCPD_T::RXBUF3: RXBUF Mask            */

#define NPD48_RXBUF4_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF4: RXBUF Position        */
#define NPD48_RXBUF4_RXBUF_Msk           (0xfful << NPD48_RXBUF4_RXBUF_Pos)                /*!< UTCPD_T::RXBUF4: RXBUF Mask            */

#define NPD48_RXBUF5_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF5: RXBUF Position        */
#define NPD48_RXBUF5_RXBUF_Msk           (0xfful << NPD48_RXBUF5_RXBUF_Pos)                /*!< UTCPD_T::RXBUF5: RXBUF Mask            */

#define NPD48_RXBUF6_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF6: RXBUF Position        */
#define NPD48_RXBUF6_RXBUF_Msk           (0xfful << NPD48_RXBUF6_RXBUF_Pos)                /*!< UTCPD_T::RXBUF6: RXBUF Mask            */

#define NPD48_RXBUF7_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF7: RXBUF Position        */
#define NPD48_RXBUF7_RXBUF_Msk           (0xfful << NPD48_RXBUF7_RXBUF_Pos)                /*!< UTCPD_T::RXBUF7: RXBUF Mask            */

#define NPD48_RXBUF8_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF8: RXBUF Position        */
#define NPD48_RXBUF8_RXBUF_Msk           (0xfful << NPD48_RXBUF8_RXBUF_Pos)                /*!< UTCPD_T::RXBUF8: RXBUF Mask            */

#define NPD48_RXBUF9_RXBUF_Pos           (0)                                               /*!< UTCPD_T::RXBUF9: RXBUF Position        */
#define NPD48_RXBUF9_RXBUF_Msk           (0xfful << NPD48_RXBUF9_RXBUF_Pos)                /*!< UTCPD_T::RXBUF9: RXBUF Mask            */

#define NPD48_RXBUF10_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF10: RXBUF Position       */
#define NPD48_RXBUF10_RXBUF_Msk          (0xfful << NPD48_RXBUF10_RXBUF_Pos)               /*!< UTCPD_T::RXBUF10: RXBUF Mask           */

#define NPD48_RXBUF11_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF11: RXBUF Position       */
#define NPD48_RXBUF11_RXBUF_Msk          (0xfful << NPD48_RXBUF11_RXBUF_Pos)               /*!< UTCPD_T::RXBUF11: RXBUF Mask           */

#define NPD48_RXBUF12_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF12: RXBUF Position       */
#define NPD48_RXBUF12_RXBUF_Msk          (0xfful << NPD48_RXBUF12_RXBUF_Pos)               /*!< UTCPD_T::RXBUF12: RXBUF Mask           */

#define NPD48_RXBUF13_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF13: RXBUF Position       */
#define NPD48_RXBUF13_RXBUF_Msk          (0xfful << NPD48_RXBUF13_RXBUF_Pos)               /*!< UTCPD_T::RXBUF13: RXBUF Mask           */

#define NPD48_RXBUF14_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF14: RXBUF Position       */
#define NPD48_RXBUF14_RXBUF_Msk          (0xfful << NPD48_RXBUF14_RXBUF_Pos)               /*!< UTCPD_T::RXBUF14: RXBUF Mask           */

#define NPD48_RXBUF15_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF15: RXBUF Position       */
#define NPD48_RXBUF15_RXBUF_Msk          (0xfful << NPD48_RXBUF15_RXBUF_Pos)               /*!< UTCPD_T::RXBUF15: RXBUF Mask           */

#define NPD48_RXBUF16_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF16: RXBUF Position       */
#define NPD48_RXBUF16_RXBUF_Msk          (0xfful << NPD48_RXBUF16_RXBUF_Pos)               /*!< UTCPD_T::RXBUF16: RXBUF Mask           */

#define NPD48_RXBUF17_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF17: RXBUF Position       */
#define NPD48_RXBUF17_RXBUF_Msk          (0xfful << NPD48_RXBUF17_RXBUF_Pos)               /*!< UTCPD_T::RXBUF17: RXBUF Mask           */

#define NPD48_RXBUF18_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF18: RXBUF Position       */
#define NPD48_RXBUF18_RXBUF_Msk          (0xfful << NPD48_RXBUF18_RXBUF_Pos)               /*!< UTCPD_T::RXBUF18: RXBUF Mask           */

#define NPD48_RXBUF19_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF19: RXBUF Position       */
#define NPD48_RXBUF19_RXBUF_Msk          (0xfful << NPD48_RXBUF19_RXBUF_Pos)               /*!< UTCPD_T::RXBUF19: RXBUF Mask           */

#define NPD48_RXBUF20_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF20: RXBUF Position       */
#define NPD48_RXBUF20_RXBUF_Msk          (0xfful << NPD48_RXBUF20_RXBUF_Pos)               /*!< UTCPD_T::RXBUF20: RXBUF Mask           */

#define NPD48_RXBUF21_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF21: RXBUF Position       */
#define NPD48_RXBUF21_RXBUF_Msk          (0xfful << NPD48_RXBUF21_RXBUF_Pos)               /*!< UTCPD_T::RXBUF21: RXBUF Mask           */

#define NPD48_RXBUF22_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF22: RXBUF Position       */
#define NPD48_RXBUF22_RXBUF_Msk          (0xfful << NPD48_RXBUF22_RXBUF_Pos)               /*!< UTCPD_T::RXBUF22: RXBUF Mask           */

#define NPD48_RXBUF23_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF23: RXBUF Position       */
#define NPD48_RXBUF23_RXBUF_Msk          (0xfful << NPD48_RXBUF23_RXBUF_Pos)               /*!< UTCPD_T::RXBUF23: RXBUF Mask           */

#define NPD48_RXBUF24_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF24: RXBUF Position       */
#define NPD48_RXBUF24_RXBUF_Msk          (0xfful << NPD48_RXBUF24_RXBUF_Pos)               /*!< UTCPD_T::RXBUF24: RXBUF Mask           */

#define NPD48_RXBUF25_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF25: RXBUF Position       */
#define NPD48_RXBUF25_RXBUF_Msk          (0xfful << NPD48_RXBUF25_RXBUF_Pos)               /*!< UTCPD_T::RXBUF25: RXBUF Mask           */

#define NPD48_RXBUF26_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF26: RXBUF Position       */
#define NPD48_RXBUF26_RXBUF_Msk          (0xfful << NPD48_RXBUF26_RXBUF_Pos)               /*!< UTCPD_T::RXBUF26: RXBUF Mask           */

#define NPD48_RXBUF27_RXBUF_Pos          (0)                                               /*!< UTCPD_T::RXBUF27: RXBUF Position       */
#define NPD48_RXBUF27_RXBUF_Msk          (0xfful << NPD48_RXBUF27_RXBUF_Pos)               /*!< UTCPD_T::RXBUF27: RXBUF Mask           */

#define NPD48_TRANSMIT_TXSTYPE_Pos       (0)                                               /*!< UTCPD_T::TRANSMIT: TXSTYPE Position    */
#define NPD48_TRANSMIT_TXSTYPE_Msk       (0x7ul << NPD48_TRANSMIT_TXSTYPE_Pos)             /*!< UTCPD_T::TRANSMIT: TXSTYPE Mask        */

#define NPD48_TRANSMIT_RXBCNT_Pos        (4)                                               /*!< UTCPD_T::TRANSMIT: RXBCNT Position     */
#define NPD48_TRANSMIT_RXBCNT_Msk        (0x3ul << NPD48_TRANSMIT_RXBCNT_Pos)              /*!< UTCPD_T::TRANSMIT: RXBCNT Mask         */

#define NPD48_TBCNT_TBCNT_Pos            (0)                                               /*!< UTCPD_T::TBCNT: TBCNT Position         */
#define NPD48_TBCNT_TBCNT_Msk            (0xfful << NPD48_TBCNT_TBCNT_Pos)                 /*!< UTCPD_T::TBCNT: TBCNT Mask             */

#define NPD48_TXBHEADL_TXHEAD_Pos        (0)                                               /*!< UTCPD_T::TXBHEADL: TXHEAD Position     */
#define NPD48_TXBHEADL_TXHEAD_Msk        (0xfful << NPD48_TXBHEADL_TXHEAD_Pos)             /*!< UTCPD_T::TXBHEADL: TXHEAD Mask         */

#define NPD48_TXBHEADH_TXHEAD_Pos        (0)                                               /*!< UTCPD_T::TXBHEADH: TXHEAD Position     */
#define NPD48_TXBHEADH_TXHEAD_Msk        (0xfful << NPD48_TXBHEADH_TXHEAD_Pos)             /*!< UTCPD_T::TXBHEADH: TXHEAD Mask         */

#define NPD48_TXBUF0_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF0: TXBUF Position        */
#define NPD48_TXBUF0_TXBUF_Msk           (0xfful << NPD48_TXBUF0_TXBUF_Pos)                /*!< UTCPD_T::TXBUF0: TXBUF Mask            */

#define NPD48_TXBUF1_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF1: TXBUF Position        */
#define NPD48_TXBUF1_TXBUF_Msk           (0xfful << NPD48_TXBUF1_TXBUF_Pos)                /*!< UTCPD_T::TXBUF1: TXBUF Mask            */

#define NPD48_TXBUF2_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF2: TXBUF Position        */
#define NPD48_TXBUF2_TXBUF_Msk           (0xfful << NPD48_TXBUF2_TXBUF_Pos)                /*!< UTCPD_T::TXBUF2: TXBUF Mask            */

#define NPD48_TXBUF3_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF3: TXBUF Position        */
#define NPD48_TXBUF3_TXBUF_Msk           (0xfful << NPD48_TXBUF3_TXBUF_Pos)                /*!< UTCPD_T::TXBUF3: TXBUF Mask            */

#define NPD48_TXBUF4_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF4: TXBUF Position        */
#define NPD48_TXBUF4_TXBUF_Msk           (0xfful << NPD48_TXBUF4_TXBUF_Pos)                /*!< UTCPD_T::TXBUF4: TXBUF Mask            */

#define NPD48_TXBUF5_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF5: TXBUF Position        */
#define NPD48_TXBUF5_TXBUF_Msk           (0xfful << NPD48_TXBUF5_TXBUF_Pos)                /*!< UTCPD_T::TXBUF5: TXBUF Mask            */

#define NPD48_TXBUF6_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF6: TXBUF Position        */
#define NPD48_TXBUF6_TXBUF_Msk           (0xfful << NPD48_TXBUF6_TXBUF_Pos)                /*!< UTCPD_T::TXBUF6: TXBUF Mask            */

#define NPD48_TXBUF7_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF7: TXBUF Position        */
#define NPD48_TXBUF7_TXBUF_Msk           (0xfful << NPD48_TXBUF7_TXBUF_Pos)                /*!< UTCPD_T::TXBUF7: TXBUF Mask            */

#define NPD48_TXBUF8_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF8: TXBUF Position        */
#define NPD48_TXBUF8_TXBUF_Msk           (0xfful << NPD48_TXBUF8_TXBUF_Pos)                /*!< UTCPD_T::TXBUF8: TXBUF Mask            */

#define NPD48_TXBUF9_TXBUF_Pos           (0)                                               /*!< UTCPD_T::TXBUF9: TXBUF Position        */
#define NPD48_TXBUF9_TXBUF_Msk           (0xfful << NPD48_TXBUF9_TXBUF_Pos)                /*!< UTCPD_T::TXBUF9: TXBUF Mask            */

#define NPD48_TXBUF10_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF10: TXBUF Position       */
#define NPD48_TXBUF10_TXBUF_Msk          (0xfful << NPD48_TXBUF10_TXBUF_Pos)               /*!< UTCPD_T::TXBUF10: TXBUF Mask           */

#define NPD48_TXBUF11_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF11: TXBUF Position       */
#define NPD48_TXBUF11_TXBUF_Msk          (0xfful << NPD48_TXBUF11_TXBUF_Pos)               /*!< UTCPD_T::TXBUF11: TXBUF Mask           */

#define NPD48_TXBUF12_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF12: TXBUF Position       */
#define NPD48_TXBUF12_TXBUF_Msk          (0xfful << NPD48_TXBUF12_TXBUF_Pos)               /*!< UTCPD_T::TXBUF12: TXBUF Mask           */

#define NPD48_TXBUF13_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF13: TXBUF Position       */
#define NPD48_TXBUF13_TXBUF_Msk          (0xfful << NPD48_TXBUF13_TXBUF_Pos)               /*!< UTCPD_T::TXBUF13: TXBUF Mask           */

#define NPD48_TXBUF14_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF14: TXBUF Position       */
#define NPD48_TXBUF14_TXBUF_Msk          (0xfful << NPD48_TXBUF14_TXBUF_Pos)               /*!< UTCPD_T::TXBUF14: TXBUF Mask           */

#define NPD48_TXBUF15_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF15: TXBUF Position       */
#define NPD48_TXBUF15_TXBUF_Msk          (0xfful << NPD48_TXBUF15_TXBUF_Pos)               /*!< UTCPD_T::TXBUF15: TXBUF Mask           */

#define NPD48_TXBUF16_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF16: TXBUF Position       */
#define NPD48_TXBUF16_TXBUF_Msk          (0xfful << NPD48_TXBUF16_TXBUF_Pos)               /*!< UTCPD_T::TXBUF16: TXBUF Mask           */

#define NPD48_TXBUF17_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF17: TXBUF Position       */
#define NPD48_TXBUF17_TXBUF_Msk          (0xfful << NPD48_TXBUF17_TXBUF_Pos)               /*!< UTCPD_T::TXBUF17: TXBUF Mask           */

#define NPD48_TXBUF18_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF18: TXBUF Position       */
#define NPD48_TXBUF18_TXBUF_Msk          (0xfful << NPD48_TXBUF18_TXBUF_Pos)               /*!< UTCPD_T::TXBUF18: TXBUF Mask           */

#define NPD48_TXBUF19_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF19: TXBUF Position       */
#define NPD48_TXBUF19_TXBUF_Msk          (0xfful << NPD48_TXBUF19_TXBUF_Pos)               /*!< UTCPD_T::TXBUF19: TXBUF Mask           */

#define NPD48_TXBUF20_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF20: TXBUF Position       */
#define NPD48_TXBUF20_TXBUF_Msk          (0xfful << NPD48_TXBUF20_TXBUF_Pos)               /*!< UTCPD_T::TXBUF20: TXBUF Mask           */

#define NPD48_TXBUF21_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF21: TXBUF Position       */
#define NPD48_TXBUF21_TXBUF_Msk          (0xfful << NPD48_TXBUF21_TXBUF_Pos)               /*!< UTCPD_T::TXBUF21: TXBUF Mask           */

#define NPD48_TXBUF22_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF22: TXBUF Position       */
#define NPD48_TXBUF22_TXBUF_Msk          (0xfful << NPD48_TXBUF22_TXBUF_Pos)               /*!< UTCPD_T::TXBUF22: TXBUF Mask           */

#define NPD48_TXBUF23_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF23: TXBUF Position       */
#define NPD48_TXBUF23_TXBUF_Msk          (0xfful << NPD48_TXBUF23_TXBUF_Pos)               /*!< UTCPD_T::TXBUF23: TXBUF Mask           */

#define NPD48_TXBUF24_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF24: TXBUF Position       */
#define NPD48_TXBUF24_TXBUF_Msk          (0xfful << NPD48_TXBUF24_TXBUF_Pos)               /*!< UTCPD_T::TXBUF24: TXBUF Mask           */

#define NPD48_TXBUF25_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF25: TXBUF Position       */
#define NPD48_TXBUF25_TXBUF_Msk          (0xfful << NPD48_TXBUF25_TXBUF_Pos)               /*!< UTCPD_T::TXBUF25: TXBUF Mask           */

#define NPD48_TXBUF26_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF26: TXBUF Position       */
#define NPD48_TXBUF26_TXBUF_Msk          (0xfful << NPD48_TXBUF26_TXBUF_Pos)               /*!< UTCPD_T::TXBUF26: TXBUF Mask           */

#define NPD48_TXBUF27_TXBUF_Pos          (0)                                               /*!< UTCPD_T::TXBUF27: TXBUF Position       */
#define NPD48_TXBUF27_TXBUF_Msk          (0xfful << NPD48_TXBUF27_TXBUF_Pos)               /*!< UTCPD_T::TXBUF27: TXBUF Mask           */

#define NPD48_VBUSVOL_VBVOL_Pos          (0)                                               /*!< UTCPD_T::VBUSVOL: VBVOL Position       */
#define NPD48_VBUSVOL_VBVOL_Msk          (0xfful << NPD48_VBUSVOL_VBVOL_Pos)               /*!< UTCPD_T::VBUSVOL: VBVOL Mask           */

#define NPD48_VBUSVOH_VBVOL_Pos          (0)                                               /*!< UTCPD_T::VBUSVOH: VBVOL Position       */
#define NPD48_VBUSVOH_VBVOL_Msk          (0x3ul << NPD48_VBUSVOH_VBVOL_Pos)                /*!< UTCPD_T::VBUSVOH: VBVOL Mask           */

#define NPD48_VBUSVOH_VBSCALE_Pos        (2)                                               /*!< UTCPD_T::VBUSVOH: VBSCALE Position     */
#define NPD48_VBUSVOH_VBSCALE_Msk        (0x3ul << NPD48_VBUSVOH_VBSCALE_Pos)              /*!< UTCPD_T::VBUSVOH: VBSCALE Mask         */

#define NPD48_VBUSDTL_SKVBDCTH_Pos       (0)                                               /*!< UTCPD_T::VBUSDTL: SKVBDCTH Position    */
#define NPD48_VBUSDTL_SKVBDCTH_Msk       (0xfful << NPD48_VBUSDTL_SKVBDCTH_Pos)            /*!< UTCPD_T::VBUSDTL: SKVBDCTH Mask        */

#define NPD48_VBUSDTH_SKVBDCTH_Pos       (0)                                               /*!< UTCPD_T::VBUSDTH: SKVBDCTH Position    */
#define NPD48_VBUSDTH_SKVBDCTH_Msk       (0x3ul << NPD48_VBUSDTH_SKVBDCTH_Pos)             /*!< UTCPD_T::VBUSDTH: SKVBDCTH Mask        */

#define NPD48_VBUSTPDTL_SPGDTH_Pos       (0)                                               /*!< UTCPD_T::VBUSTPDTL: SPGDTH Position    */
#define NPD48_VBUSTPDTL_SPGDTH_Msk       (0xfful << NPD48_VBUSTPDTL_SPGDTH_Pos)            /*!< UTCPD_T::VBUSTPDTL: SPGDTH Mask        */

#define NPD48_VBUSTPDTH_SPGDTH_Pos       (0)                                               /*!< UTCPD_T::VBUSTPDTH: SPGDTH Position    */
#define NPD48_VBUSTPDTH_SPGDTH_Msk       (0x3ul << NPD48_VBUSTPDTH_SPGDTH_Pos)             /*!< UTCPD_T::VBUSTPDTH: SPGDTH Mask        */

#define NPD48_VBUSOVTL_VBAMTH_Pos        (0)                                               /*!< UTCPD_T::VBUSOVTL: VBAMTH Position     */
#define NPD48_VBUSOVTL_VBAMTH_Msk        (0xfful << NPD48_VBUSOVTL_VBAMTH_Pos)             /*!< UTCPD_T::VBUSOVTL: VBAMTH Mask         */

#define NPD48_VBUSOVTH_VBAMTH_Pos        (0)                                               /*!< UTCPD_T::VBUSOVTH: VBAMTH Position     */
#define NPD48_VBUSOVTH_VBAMTH_Msk        (0x3ul << NPD48_VBUSOVTH_VBAMTH_Pos)              /*!< UTCPD_T::VBUSOVTH: VBAMTH Mask         */

#define NPD48_VBUSUVTL_VBAMTL_Pos        (0)                                               /*!< UTCPD_T::VBUSUVTL: VBAMTL Position     */
#define NPD48_VBUSUVTL_VBAMTL_Msk        (0xfful << NPD48_VBUSUVTL_VBAMTL_Pos)             /*!< UTCPD_T::VBUSUVTL: VBAMTL Mask         */

#define NPD48_VBUSUVTH_VBAMTL_Pos        (0)                                               /*!< UTCPD_T::VBUSUVTH: VBAMTL Position     */
#define NPD48_VBUSUVTH_VBAMTL_Msk        (0x3ul << NPD48_VBUSUVTH_VBAMTL_Pos)              /*!< UTCPD_T::VBUSUVTH: VBAMTL Mask         */

#define NPD48_FSASTS_FSDISRXSTS_Pos      (0)                                               /*!< UTCPD_T::FSASTS: FSDISRXSTS Position   */
#define NPD48_FSASTS_FSDISRXSTS_Msk      (0x1ul << NPD48_FSASTS_FSDISRXSTS_Pos)            /*!< UTCPD_T::FSASTS: FSDISRXSTS Mask       */

#define NPD48_FSASTS_FSDISTXSTS_Pos      (1)                                               /*!< UTCPD_T::FSASTS: FSDISTXSTS Position   */
#define NPD48_FSASTS_FSDISTXSTS_Msk      (0x1ul << NPD48_FSASTS_FSDISTXSTS_Pos)            /*!< UTCPD_T::FSASTS: FSDISTXSTS Mask       */

#define NPD48_FSASTS_WDTSTS_Pos          (2)                                               /*!< UTCPD_T::FSASTS: WDTSTS Position       */
#define NPD48_FSASTS_WDTSTS_Msk          (0x1ul << NPD48_FSASTS_WDTSTS_Pos)                /*!< UTCPD_T::FSASTS: WDTSTS Mask           */

#define NPD48_FSASTS_CRCESTS_Pos         (3)                                               /*!< UTCPD_T::FSASTS: PDCRC Error Position   */
#define NPD48_FSASTS_CRCESTS_Msk         (0x1ul << NPD48_FSASTS_CRCESTS_Pos)               /*!< UTCPD_T::FSASTS: PDCRC Error Mask       */

#define NPD48_FSASTS_EHWRSTK_Pos         (4)                                               /*!< UTCPD_T::FSASTS: EHWRSTK Position      */
#define NPD48_FSASTS_EHWRSTK_Msk         (0x1ul << NPD48_FSASTS_EHWRSTK_Pos)               /*!< UTCPD_T::FSASTS: EHWRSTK Mask          */

#define NPD48_FSASTS_VCUSTS_Pos          (5)                                               /*!< UTCPD_T::FSASTS: VCUSTS Position       */
#define NPD48_FSASTS_VCUSTS_Msk          (0x1ul << NPD48_FSASTS_VCUSTS_Pos)                /*!< UTCPD_T::FSASTS: VCUSTS Mask           */

#define NPD48_FSASTS_VCOTPSTS_Pos        (6)                                               /*!< UTCPD_T::FSASTS: VCOTPSTS Position     */
#define NPD48_FSASTS_VCOTPSTS_Msk        (0x1ul << NPD48_FSASTS_VCOTPSTS_Pos)              /*!< UTCPD_T::FSASTS: VCOTPSTS Mask         */

#define NPD48_DRPTRCTL_DRPRATIO_Pos      (0)                                               /*!< UTCPD_T::DRPTRCTL: DRPRATIO Position   */
#define NPD48_DRPTRCTL_DRPRATIO_Msk      (0x7ul << NPD48_DRPTRCTL_DRPRATIO_Pos)            /*!< UTCPD_T::DRPTRCTL: DRPRATIO Mask       */

#define NPD48_FSAMASK_FSDISRXEN_Pos      (0)                                               /*!< UTCPD_T::FSAMASK: FSDISRXEN Position   */
#define NPD48_FSAMASK_FSDISRXEN_Msk      (0x1ul << NPD48_FSAMASK_FSDISRXEN_Pos)            /*!< UTCPD_T::FSAMASK: FSDISRXEN Mask       */

#define NPD48_FSAMASK_FSDISTXEN_Pos      (1)                                               /*!< UTCPD_T::FSAMASK: FSDISTXEN Position   */
#define NPD48_FSAMASK_FSDISTXEN_Msk      (0x1ul << NPD48_FSAMASK_FSDISTXEN_Pos)            /*!< UTCPD_T::FSAMASK: FSDISTXEN Mask       */

#define NPD48_FSAMASK_WDTEN_Pos          (2)                                               /*!< UTCPD_T::FSAMASK: WDTEN Position       */
#define NPD48_FSAMASK_WDTEN_Msk          (0x1ul << NPD48_FSAMASK_WDTEN_Pos)                /*!< UTCPD_T::FSAMASK: WDTEN Mask           */

#define NPD48_FSAMASK_CRCEN_Pos          (3)                                               /*!< UTCPD_T::FSAMASK: CRCEN Position       */
#define NPD48_FSAMASK_CRCEN_Msk          (0x1ul << NPD48_FSAMASK_CRCEN_Pos)                /*!< UTCPD_T::FSAMASK: CRCEN Mask           */

#define NPD48_FSAMASK_EHWRSTKEN_Pos      (4)                                               /*!< UTCPD_T::FSAMASK: EHWRSTKEN Position   */
#define NPD48_FSAMASK_EHWRSTKEN_Msk      (0x1ul << NPD48_FSAMASK_EHWRSTKEN_Pos)            /*!< UTCPD_T::FSAMASK: EHWRSTKEN Mask       */

#define NPD48_FSAMASK_VCUEN_Pos          (5)                                               /*!< UTCPD_T::FSAMASK: VCUEN Position       */
#define NPD48_FSAMASK_VCUEN_Msk          (0x1ul << NPD48_FSAMASK_VCUEN_Pos)                /*!< UTCPD_T::FSAMASK: VCUEN Mask           */

#define NPD48_FSAMASK_VCOTPEN_Pos        (6)                                               /*!< UTCPD_T::FSAMASK: VCOTPEN Position     */
#define NPD48_FSAMASK_VCOTPEN_Msk        (0x1ul << NPD48_FSAMASK_VCOTPEN_Pos)              /*!< UTCPD_T::FSAMASK: VCOTPEN Mask         */

#define NPD48_BMCSCTL_SLICEL_Pos         (0)                                               /*!< UTCPD_T::BMCSCTL: SLICEL Position      */
#define NPD48_BMCSCTL_SLICEL_Msk         (0x3ul << NPD48_BMCSCTL_SLICEL_Pos)               /*!< UTCPD_T::BMCSCTL: SLICEL Mask          */

#define NPD48_BMCSCTL_SLICEH_Pos         (2)                                               /*!< UTCPD_T::BMCSCTL: SLICEH Position      */
#define NPD48_BMCSCTL_SLICEH_Msk         (0x3ul << NPD48_BMCSCTL_SLICEH_Pos)               /*!< UTCPD_T::BMCSCTL: SLICEH Mask          */

#define NPD48_BMCSCTL_SLICEM_Pos         (5)                                               /*!< UTCPD_T::BMCSCTL: SLICEM Position      */
#define NPD48_BMCSCTL_SLICEM_Msk         (0x3ul << NPD48_BMCSCTL_SLICEM_Pos)               /*!< UTCPD_T::BMCSCTL: SLICEM Mask          */

#define NPD48_VCDISCTL_VCONDISEN_Pos     (0)                                               /*!< UTCPD_T::VCDISCTL: VCONDISEN Position  */
#define NPD48_VCDISCTL_VCONDISEN_Msk     (0x1ul << NPD48_VCDISCTL_VCONDISEN_Pos)           /*!< UTCPD_T::VCDISCTL: VCONDISEN Mask      */

#define NPD48_VCDISCTL_VBSTPWT_Pos       (1)                                               /*!< UTCPD_T::VCDISCTL: VBSTPWT Position    */
#define NPD48_VCDISCTL_VBSTPWT_Msk       (0x7ul << NPD48_VCDISCTL_VBSTPWT_Pos)             /*!< UTCPD_T::VCDISCTL: VBSTPWT Mask        */

#define NPD48_VCDISCTL_VBOPNWT_Pos       (4)                                               /*!< UTCPD_T::VCDISCTL: VBOPNWT Position    */
#define NPD48_VCDISCTL_VBOPNWT_Msk       (0x7ul << NPD48_VCDISCTL_VBOPNWT_Pos)             /*!< UTCPD_T::VCDISCTL: VBOPNWT Mask        */

#define NPD48_VCDISCTL_VCDISACT_Pos      (7)                                               /*!< UTCPD_T::VCDISCTL: VBOPNWT Position    */
#define NPD48_VCDISCTL_VCDISACT_Msk      (0x7ul << NPD48_VCDISCTL_VCDISACT_Pos)            /*!< UTCPD_T::VCDISCTL: VBOPNWT Mask        */

#define NPD48_CCTRIM_CCTRIMR_Pos         (0)                                               /*!< UTCPD_T::CCTRIM: CCTRIMR Position      */
#define NPD48_CCTRIM_CCTRIMR_Msk         (0x7ul << NPD48_CCTRIM_CCTRIMR_Pos)               /*!< UTCPD_T::CCTRIM: CCTRIMR Mask          */

#define NPD48_CCTRIM_CCTRIMF_Pos         (3)                                               /*!< UTCPD_T::CCTRIM: CCTRIMF Position      */
#define NPD48_CCTRIM_CCTRIMF_Msk         (0x7ul << NPD48_CCTRIM_CCTRIMF_Pos)               /*!< UTCPD_T::CCTRIM: CCTRIMF Mask          */

#define NPD48_CCTRIM_BIDLSEL_Pos         (6)                                               /*!< UTCPD_T::CCTRIM: BIDLSEL Position      */
#define NPD48_CCTRIM_BIDLSEL_Msk         (0x3ul << NPD48_CCTRIM_BIDLSEL_Pos)               /*!< UTCPD_T::CCTRIM: BIDLSEL Mask          */

#define NPD48_AFESCTL_AFETST_Pos         (0)                                               /*!< UTCPD_T::AFESCTL: AFETST Position      */
#define NPD48_AFESCTL_AFETST_Msk         (0xful << NPD48_AFESCTL_AFETST_Pos)               /*!< UTCPD_T::AFESCTL: AFETST Mask          */

#define NPD48_AFESCTL_TSMIDEN_Pos        (4)                                               /*!< UTCPD_T::AFESCTL: TSMIDEN Position     */
#define NPD48_AFESCTL_TSMIDEN_Msk        (0x1ul << NPD48_AFESCTL_TSMIDEN_Pos)              /*!< UTCPD_T::AFESCTL: TSMIDEN Mask         */

#define NPD48_AFESCTL_TSALLEN_Pos        (5)                                               /*!< UTCPD_T::AFESCTL: TSALLEN Position     */
#define NPD48_AFESCTL_TSALLEN_Msk        (0x1ul << NPD48_AFESCTL_TSALLEN_Pos)              /*!< UTCPD_T::AFESCTL: TSALLEN Mask         */

#define NPD48_ADTIME_ADGTM_Pos           (0)                                               /*!< UTCPD_T::ADTIME: ADGTM Position        */
#define NPD48_ADTIME_ADGTM_Msk           (0xfful << NPD48_ADTIME_ADGTM_Pos)                /*!< UTCPD_T::ADTIME: ADGTM Mask            */

#define NPD48_ADVSAFE_VSAFE0V_Pos        (0)                                               /*!< UTCPD_T::ADVSAFE: VSAFE0V Position     */
#define NPD48_ADVSAFE_VSAFE0V_Msk        (0xfful << NPD48_ADVSAFE_VSAFE0V_Pos)             /*!< UTCPD_T::ADVSAFE: VSAFE0V Mask         */

#define NPD48_FRSVSAFE_VSAFE5V_Pos       (0)                                               /*!< UTCPD_T::FRSVSAFE: VSAFE5V Position    */
#define NPD48_FRSVSAFE_VSAFE5V_Msk       (0xfful << NPD48_FRSVSAFE_VSAFE5V_Pos)            /*!< UTCPD_T::FRSVSAFE: VSAFE5V Mask        */

#define NPD48_CCDEBTM_CCDEBTM_Pos        (0)                                               /*!< UTCPD_T::CCDEBTM: CCDEBTM Position     */
#define NPD48_CCDEBTM_CCDEBTM_Msk        (0xfful << NPD48_CCDEBTM_CCDEBTM_Pos)             /*!< UTCPD_T::CCDEBTM: CCDEBTM Mask         */

#define NPD48_CCFILTM_CCFILTERTM_Pos     (0)                                               /*!< UTCPD_T::CCFILTM: CCFILTERTM Position  */
#define NPD48_CCFILTM_CCFILTERTM_Msk     (0xfful << NPD48_CCFILTM_CCFILTERTM_Pos)          /*!< UTCPD_T::CCFILTM: CCFILTERTM Mask      */

#define NPD48_CCDRPTTM_CCDRPTTM_Pos      (0)                                               /*!< UTCPD_T::CCDRPTTM: CCDRPTTM Position   */
#define NPD48_CCDRPTTM_CCDRPTTM_Msk      (0xfful << NPD48_CCDRPTTM_CCDRPTTM_Pos)           /*!< UTCPD_T::CCDRPTTM: CCDRPTTM Mask       */

#define NPD48_DRPTCTL_CCDRPTR_Pos        (0)                                               /*!< UTCPD_T::DRPTCTL: CCDRPTR Position     */
#define NPD48_DRPTCTL_CCDRPTR_Msk        (0x7ul << NPD48_DRPTCTL_CCDRPTR_Pos)              /*!< UTCPD_T::DRPTCTL: CCDRPTR Mask         */

#define NPD48_DRPTCTL_VBUSVSEL_Pos       (3)                                               /*!< UTCPD_T::DRPTCTL: VBUSVSEL Position    */
#define NPD48_DRPTCTL_VBUSVSEL_Msk       (0x1ul << NPD48_DRPTCTL_VBUSVSEL_Pos)             /*!< UTCPD_T::DRPTCTL: VBUSVSEL Mask        */

#define NPD48_DRPTCTL_CLKSTOEN_Pos       (4)                                               /*!< UTCPD_T::DRPTCTL: CLKSTOEN Position    */
#define NPD48_DRPTCTL_CLKSTOEN_Msk       (0x1ul << NPD48_DRPTCTL_CLKSTOEN_Pos)             /*!< UTCPD_T::DRPTCTL: CLKSTOEN Mask        */

#define NPD48_DRPTCTL_FSTPEN_Pos         (5)                                               /*!< UTCPD_T::DRPTCTL: FSTPEN Position      */
#define NPD48_DRPTCTL_FSTPEN_Msk         (0x1ul << NPD48_DRPTCTL_FSTPEN_Pos)               /*!< UTCPD_T::DRPTCTL: FSTPEN Mask          */

#define NPD48_DRPTCTL_VBUSMEN_Pos        (6)                                               /*!< UTCPD_T::DRPTCTL: VBUSMEN Position     */
#define NPD48_DRPTCTL_VBUSMEN_Msk        (0x1ul << NPD48_DRPTCTL_VBUSMEN_Pos)              /*!< UTCPD_T::DRPTCTL: VBUSMEN Mask         */

#define NPD48_DRPTCTL_TESTMEN_Pos        (7)                                               /*!< UTCPD_T::DRPTCTL: TESTMEN Position     */
#define NPD48_DRPTCTL_TESTMEN_Msk        (0x1ul << NPD48_DRPTCTL_TESTMEN_Pos)              /*!< UTCPD_T::DRPTCTL: TESTMEN Mask         */

#define NPD48_INTFRMG_INTFRMG_Pos        (0)                                               /*!< UTCPD_T::INTFRMG: INTFRMG Position     */
#define NPD48_INTFRMG_INTFRMG_Msk        (0xfful << NPD48_INTFRMG_INTFRMG_Pos)             /*!< UTCPD_T::INTFRMG: INTFRMG Mask         */

#define NPD48_EHWRSTK_CTLRSTS_Pos        (0)                                               /*!< UTCPD_T::EHWRSTK: CTLRSTS Position     */
#define NPD48_EHWRSTK_CTLRSTS_Msk        (0x1ul << NPD48_EHWRSTK_CTLRSTS_Pos)              /*!< UTCPD_T::EHWRSTK: CTLRSTS Mask         */

#define NPD48_SOPNGCRC_SOPNGCRC_Pos      (0)                                               /*!< UTCPD_T::SOPNGCRC: SOPNGCRC Position   */
#define NPD48_SOPNGCRC_SOPNGCRC_Msk      (0x1ful << NPD48_SOPNGCRC_SOPNGCRC_Pos)           /*!< UTCPD_T::SOPNGCRC: SOPNGCRC Mask       */

#define NPD48_VBDISSTS_SRCOPNLVL_Pos     (0)                                               /*!< UTCPD_T::VBDISSTS: SRCOPNLVL Position  */
#define NPD48_VBDISSTS_SRCOPNLVL_Msk     (0x1ul << NPD48_VBDISSTS_SRCOPNLVL_Pos)           /*!< UTCPD_T::VBDISSTS: SRCOPNLVL Mask      */

#define NPD48_VBDISSTS_SNKOPNLVL_Pos     (1)                                               /*!< UTCPD_T::VBDISSTS: SNKOPNLVL Position  */
#define NPD48_VBDISSTS_SNKOPNLVL_Msk     (0x1ul << NPD48_VBDISSTS_SNKOPNLVL_Pos)           /*!< UTCPD_T::VBDISSTS: SNKOPNLVL Mask      */

#define NPD48_VBDISSTS_PWRDONPHY_Pos     (3)                                               /*!< UTCPD_T::VBDISSTS: PWRDONPHY Position  */
#define NPD48_VBDISSTS_PWRDONPHY_Msk     (0x1ul << NPD48_VBDISSTS_PWRDONPHY_Pos)           /*!< UTCPD_T::VBDISSTS: PWRDONPHY Mask      */

#define NPD48_VBDISSTS_BISTMALLIN_Pos    (5)                                               /*!< UTCPD_T::VBDISSTS: BISTMALLIN Position */
#define NPD48_VBDISSTS_BISTMALLIN_Msk    (0x1ul << NPD48_VBDISSTS_BISTMALLIN_Pos)          /*!< UTCPD_T::VBDISSTS: BISTMALLIN Mask     */

#define NPD48_VBDISSTS_BISTMTDIN_Pos     (6)                                               /*!< UTCPD_T::VBDISSTS: BISTMTDIN Position  */
#define NPD48_VBDISSTS_BISTMTDIN_Msk     (0x1ul << NPD48_VBDISSTS_BISTMTDIN_Pos)           /*!< UTCPD_T::VBDISSTS: BISTMTDIN Mask      */

#define NPD48_VBDISSTS_BISTM2IN_Pos      (7)                                               /*!< UTCPD_T::VBDISSTS: BISTM2IN Position   */
#define NPD48_VBDISSTS_BISTM2IN_Msk      (0x1ul << NPD48_VBDISSTS_BISTM2IN_Pos)            /*!< UTCPD_T::VBDISSTS: BISTM2IN Mask       */

#define NPD48_PICFG_CFG1ID_Pos           (0)                                               /*!< UTCPD_T::PICFG: CFG1ID Position        */
#define NPD48_PICFG_CFG1ID_Msk           (0x1ful << NPD48_PICFG_CFG1ID_Pos)                /*!< UTCPD_T::PICFG: CFG1ID Mask            */

#define NPD48_PICFG_CFG1RSVD_Pos         (5)                                               /*!< UTCPD_T::PICFG: CFG1RSVD Position      */
#define NPD48_PICFG_CFG1RSVD_Msk         (0x3ul << NPD48_PICFG_CFG1RSVD_Pos)               /*!< UTCPD_T::PICFG: CFG1RSVD Mask          */

#define NPD48_PICFG_CFG1IDFILL_Pos       (7)                                               /*!< UTCPD_T::PICFG: CFG1IDFILL Position    */
#define NPD48_PICFG_CFG1IDFILL_Msk       (0x1ul << NPD48_PICFG_CFG1IDFILL_Pos)             /*!< UTCPD_T::PICFG: CFG1IDFILL Mask        */

#define NPD48_BMCCTL_BMCEXT_Pos          (0)                                               /*!< UTCPD_T::BMCCTL: BMCEXT Position       */
#define NPD48_BMCCTL_BMCEXT_Msk          (0x1ul << NPD48_BMCCTL_BMCEXT_Pos)                /*!< UTCPD_T::BMCCTL: BMCEXT Mask           */

#define NPD48_BMCCTL_HRINTDSCRD_Pos      (1)                                               /*!< UTCPD_T::BMCCTL: HRINTDSCRD Position   */
#define NPD48_BMCCTL_HRINTDSCRD_Msk      (0x1ul << NPD48_BMCCTL_HRINTDSCRD_Pos)            /*!< UTCPD_T::BMCCTL: HRINTDSCRD Mask       */

#define NPD48_BMCCTL_CRINTDSCRD_Pos      (2)                                               /*!< UTCPD_T::BMCCTL: CRINTDSCRD Position   */
#define NPD48_BMCCTL_CRINTDSCRD_Msk      (0x1ul << NPD48_BMCCTL_CRINTDSCRD_Pos)            /*!< UTCPD_T::BMCCTL: CRINTDSCRD Mask       */

#define NPD48_BMCCTL_CTL13PDDIS_Pos      (3)                                               /*!< UTCPD_T::BMCCTL: CTL13PDDIS Position   */
#define NPD48_BMCCTL_CTL13PDDIS_Msk      (0x1ul << NPD48_BMCCTL_CTL13PDDIS_Pos)            /*!< UTCPD_T::BMCCTL: CTL13PDDIS Mask       */

#define NPD48_BMCCTL_CTL13DETDIS_Pos     (4)                                               /*!< UTCPD_T::BMCCTL: CTL13DETDIS Position  */
#define NPD48_BMCCTL_CTL13DETDIS_Msk     (0x1ul << NPD48_BMCCTL_CTL13DETDIS_Pos)           /*!< UTCPD_T::BMCCTL: CTL13DETDIS Mask      */

#define NPD48_BMCCTL_CTL13LB_Pos         (5)                                               /*!< UTCPD_T::BMCCTL: CTL13LB Position      */
#define NPD48_BMCCTL_CTL13LB_Msk         (0x1ul << NPD48_BMCCTL_CTL13LB_Pos)               /*!< UTCPD_T::BMCCTL: CTL13LB Mask          */

#define NPD48_BMCCTL_CTL13IGNVB_Pos      (6)                                               /*!< UTCPD_T::BMCCTL: CTL13IGNVB Position   */
#define NPD48_BMCCTL_CTL13IGNVB_Msk      (0x1ul << NPD48_BMCCTL_CTL13IGNVB_Pos)            /*!< UTCPD_T::BMCCTL: CTL13IGNVB Mask       */

#define NPD48_BMCCTL_CTL13TM_Pos         (7)                                               /*!< UTCPD_T::BMCCTL: CTL13TM Position      */
#define NPD48_BMCCTL_CTL13TM_Msk         (0x1ul << NPD48_BMCCTL_CTL13TM_Pos)               /*!< UTCPD_T::BMCCTL: CTL13TM Mask          */

#define NPD48_AFEPWRSTE_PHYPWR_Pos       (0)                                               /*!< UTCPD_T::AFEPWRSTE: PHYPWR Position    */
#define NPD48_AFEPWRSTE_PHYPWR_Msk       (0x1ul << NPD48_AFEPWRSTE_PHYPWR_Pos)             /*!< UTCPD_T::AFEPWRSTE: PHYPWR Mask        */

#define NPD48_AFEPWRSTE_AFEPWR_Pos       (1)                                               /*!< UTCPD_T::AFEPWRSTE: AFEPWR Position    */
#define NPD48_AFEPWRSTE_AFEPWR_Msk       (0x7ul << NPD48_AFEPWRSTE_AFEPWR_Pos)             /*!< UTCPD_T::AFEPWRSTE: AFEPWR Mask        */

#define NPD48_IDLETM_IDLETM_Pos          (0)                                               /*!< UTCPD_T::IDLETM: IDLETM Position       */
#define NPD48_IDLETM_IDLETM_Msk          (0xfful << NPD48_IDLETM_IDLETM_Pos)               /*!< UTCPD_T::IDLETM: IDLETM Mask           */

#define NPD48_CCSTATE_CCFSM_Pos          (0)                                               /*!< UTCPD_T::CCSTATE: CCFSM Position       */
#define NPD48_CCSTATE_CCFSM_Msk          (0x1ful << NPD48_CCSTATE_CCFSM_Pos)               /*!< UTCPD_T::CCSTATE: CCFSM Mask           */

#define NPD48_CCSTATE_CCSTATE_Pos        (5)                                               /*!< UTCPD_T::CCSTATE: CCSTATE Position     */
#define NPD48_CCSTATE_CCSTATE_Msk        (0x7ul << NPD48_CCSTATE_CCSTATE_Pos)              /*!< UTCPD_T::CCSTATE: CCSTATE Mask         */

#define NPD48_CCSTS1_CC1STSSRC_Pos       (0)                                               /*!< UTCPD_T::CCSTS1: CC1STSSRC Position    */
#define NPD48_CCSTS1_CC1STSSRC_Msk       (0x3ul << NPD48_CCSTS1_CC1STSSRC_Pos)             /*!< UTCPD_T::CCSTS1: CC1STSSRC Mask        */

#define NPD48_CCSTS1_CC1STSSNK_Pos       (2)                                               /*!< UTCPD_T::CCSTS1: CC1STSSNK Position    */
#define NPD48_CCSTS1_CC1STSSNK_Msk       (0x3ul << NPD48_CCSTS1_CC1STSSNK_Pos)             /*!< UTCPD_T::CCSTS1: CC1STSSNK Mask        */

#define NPD48_CCSTS1_CC2STSSRC_Pos       (4)                                               /*!< UTCPD_T::CCSTS1: CC2STSSRC Position    */
#define NPD48_CCSTS1_CC2STSSRC_Msk       (0x3ul << NPD48_CCSTS1_CC2STSSRC_Pos)             /*!< UTCPD_T::CCSTS1: CC2STSSRC Mask        */

#define NPD48_CCSTS1_CC2STSSNK_Pos       (6)                                               /*!< UTCPD_T::CCSTS1: CC2STSSNK Position    */
#define NPD48_CCSTS1_CC2STSSNK_Msk       (0x3ul << NPD48_CCSTS1_CC2STSSNK_Pos)             /*!< UTCPD_T::CCSTS1: CC2STSSNK Mask        */

#define NPD48_FSTXCTL_FSDISCTX8_Pos      (0)                                               /*!< UTCPD_T::FSTXCTL: FSDISCTX8 Position   */
#define NPD48_FSTXCTL_FSDISCTX8_Msk      (0x1ul << NPD48_FSTXCTL_FSDISCTX8_Pos)            /*!< UTCPD_T::FSTXCTL: FSDISCTX8 Mask       */

#define NPD48_FSTXCTL_FSTXEN_Pos         (1)                                               /*!< UTCPD_T::FSTXCTL: FSTXEN Position      */
#define NPD48_FSTXCTL_FSTXEN_Msk         (0x1ul << NPD48_FSTXCTL_FSTXEN_Pos)               /*!< UTCPD_T::FSTXCTL: FSTXEN Mask          */

#define NPD48_FSTXCTL_FSRXDETEN_Pos      (2)                                               /*!< UTCPD_T::FSTXCTL: FSRXDETEN Position   */
#define NPD48_FSTXCTL_FSRXDETEN_Msk      (0x1ul << NPD48_FSTXCTL_FSRXDETEN_Pos)            /*!< UTCPD_T::FSTXCTL: FSRXDETEN Mask       */

#define NPD48_FSTXCTL_FSRXDET_Pos        (3)                                               /*!< UTCPD_T::FSTXCTL: FSRXDET Position     */
#define NPD48_FSTXCTL_FSRXDET_Msk        (0x1ul << NPD48_FSTXCTL_FSRXDET_Pos)              /*!< UTCPD_T::FSTXCTL: FSRXDET Mask         */

#define NPD48_FSTXCTL_CC1OVDET_Pos       (4)                                               /*!< UTCPD_T::FSTXCTL: CC1OVDET Position    */
#define NPD48_FSTXCTL_CC1OVDET_Msk       (0x1ul << NPD48_FSTXCTL_CC1OVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC1OVDET Mask        */

#define NPD48_FSTXCTL_CC2OVDET_Pos       (5)                                               /*!< UTCPD_T::FSTXCTL: CC2OVDET Position    */
#define NPD48_FSTXCTL_CC2OVDET_Msk       (0x1ul << NPD48_FSTXCTL_CC2OVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC2OVDET Mask        */

#define NPD48_FSTXCTL_CC1UVVDET_Pos      (6)                                               /*!< UTCPD_T::FSTXCTL: CC1UVVDET Position   */
#define NPD48_FSTXCTL_CC1UVVDET_Msk      (0x1ul << NPD48_FSTXCTL_CC1UVVDET_Pos)            /*!< UTCPD_T::FSTXCTL: CC1UVVDET Mask       */

#define NPD48_FSTXCTL_CC2UVDET_Pos       (7)                                               /*!< UTCPD_T::FSTXCTL: CC2UVDET Position    */
#define NPD48_FSTXCTL_CC2UVDET_Msk       (0x1ul << NPD48_FSTXCTL_CC2UVDET_Pos)             /*!< UTCPD_T::FSTXCTL: CC2UVDET Mask        */

#define NPD48_EARLYTM_EARTMLO_Pos        (0)                                               /*!< UTCPD_T::EARLYTM: EARTMLO Position     */
#define NPD48_EARLYTM_EARTMLO_Msk        (0xful << NPD48_EARLYTM_EARTMLO_Pos)              /*!< UTCPD_T::EARLYTM: EARTMLO Mask         */

#define NPD48_EARLYTM_EARTMHI_Pos        (4)                                               /*!< UTCPD_T::EARLYTM: EARTMHI Position     */
#define NPD48_EARLYTM_EARTMHI_Msk        (0xful << NPD48_EARLYTM_EARTMHI_Pos)              /*!< UTCPD_T::EARLYTM: EARTMHI Mask         */

#define NPD48_DISCARD_DISCARDTM_Pos      (0)                                               /*!< UTCPD_T::DISCARD: DISCARDTM Position   */
#define NPD48_DISCARD_DISCARDTM_Msk      (0xfful << NPD48_DISCARD_DISCARDTM_Pos)           /*!< UTCPD_T::DISCARD: DISCARDTM Mask       */

#define NPD48_CCPDSTS_PUEN_Pos           (0)                                               /*!< UTCPD_T::CCPDSTS: PUEN Position        */
#define NPD48_CCPDSTS_PUEN_Msk           (0x3ul << NPD48_CCPDSTS_PUEN_Pos)                 /*!< UTCPD_T::CCPDSTS: PUEN Mask            */

#define NPD48_CCPDSTS_PUDWN_Pos          (3)                                               /*!< UTCPD_T::CCPDSTS: PUDWN Position       */
#define NPD48_CCPDSTS_PUDWN_Msk          (0x3ul << NPD48_CCPDSTS_PUDWN_Pos)                /*!< UTCPD_T::CCPDSTS: PUDWN Mask           */

#define NPD48_BMCSTS_TXSTS_Pos           (0)                                               /*!< UTCPD_T::BMCSTS: TXSTS Position        */
#define NPD48_BMCSTS_TXSTS_Msk           (0xful << NPD48_BMCSTS_TXSTS_Pos)                 /*!< UTCPD_T::BMCSTS: TXSTS Mask            */

#define NPD48_BMCSTS_RXSTS_Pos           (4)                                               /*!< UTCPD_T::BMCSTS: RXSTS Position        */
#define NPD48_BMCSTS_RXSTS_Msk           (0xful << NPD48_BMCSTS_RXSTS_Pos)                 /*!< UTCPD_T::BMCSTS: RXSTS Mask            */

#define NPD48_PWRASTS_SRCENLVL_Pos       (0)                                               /*!< UTCPD_T::PWRASTS: SRCENLVL Position    */
#define NPD48_PWRASTS_SRCENLVL_Msk       (0x1ul << NPD48_PWRASTS_SRCENLVL_Pos)             /*!< UTCPD_T::PWRASTS: SRCENLVL Mask        */

#define NPD48_PWRASTS_SNKENLVL_Pos       (1)                                               /*!< UTCPD_T::PWRASTS: SNKENLVL Position    */
#define NPD48_PWRASTS_SNKENLVL_Msk       (0x1ul << NPD48_PWRASTS_SNKENLVL_Pos)             /*!< UTCPD_T::PWRASTS: SNKENLVL Mask        */

#define NPD48_PWRASTS_FCDENLVL_Pos       (2)                                               /*!< UTCPD_T::PWRASTS: FCDENLVL Position    */
#define NPD48_PWRASTS_FCDENLVL_Msk       (0x1ul << NPD48_PWRASTS_FCDENLVL_Pos)             /*!< UTCPD_T::PWRASTS: FCDENLVL Mask        */

#define NPD48_PWRASTS_BDCLVL_Pos         (3)                                               /*!< UTCPD_T::PWRASTS: BDCLVL Position      */
#define NPD48_PWRASTS_BDCLVL_Msk         (0x1ul << NPD48_PWRASTS_BDCLVL_Pos)               /*!< UTCPD_T::PWRASTS: BDCLVL Mask          */

#define NPD48_PCACTL_SRCENLVL_Pos       (0)                                                /*!< UTCPD_T::PWRASTS: SRCENLVL Position    */
#define NPD48_PCACTL_SRCENLVL_Msk       (0x1ul << NPD48_PCACTL_SRCENLVL_Pos)               /*!< UTCPD_T::PWRASTS: SRCENLVL Mask        */

#define NPD48_PCACTL_SNKENLVL_Pos       (1)                                                /*!< UTCPD_T::PWRASTS: SNKENLVL Position    */
#define NPD48_PCACTL_SNKENLVL_Msk       (0x1ul << NPD48_PCACTL_SNKENLVL_Pos)               /*!< UTCPD_T::PWRASTS: SNKENLVL Mask        */

#define NPD48_PCACTL_FCDENLVL_Pos       (2)                                                /*!< UTCPD_T::PWRASTS: FCDENLVL Position    */
#define NPD48_PCACTL_FCDENLVL_Msk       (0x1ul << NPD48_PCACTL_FCDENLVL_Pos)               /*!< UTCPD_T::PWRASTS: FCDENLVL Mask        */

#define NPD48_PCACTL_BDCLVL_Pos         (3)                                                /*!< UTCPD_T::PWRASTS: BDCLVL Position      */
#define NPD48_PCACTL_BDCLVL_Msk         (0x1ul << NPD48_PCACTL_BDCLVL_Pos)                 /*!< UTCPD_T::PWRASTS: BDCLVL Mask          */

#define NPD48_PHYCC1CMP_CC1DETRA_Pos     (0)                                               /*!< UTCPD_T::PHYCC1CMP: CC1DETRA Position  */
#define NPD48_PHYCC1CMP_CC1DETRA_Msk     (0x1ul << NPD48_PHYCC1CMP_CC1DETRA_Pos)           /*!< UTCPD_T::PHYCC1CMP: CC1DETRA Mask      */

#define NPD48_PHYCC1CMP_CC1DETRD_Pos     (1)                                               /*!< UTCPD_T::PHYCC1CMP: CC1DETRD Position  */
#define NPD48_PHYCC1CMP_CC1DETRD_Msk     (0x1ul << NPD48_PHYCC1CMP_CC1DETRD_Pos)           /*!< UTCPD_T::PHYCC1CMP: CC1DETRD Mask      */

#define NPD48_PHYCC1CMP_CC1DETDEF_Pos    (2)                                               /*!< UTCPD_T::PHYCC1CMP: CC1DETDEF Position */
#define NPD48_PHYCC1CMP_CC1DETDEF_Msk    (0x1ul << NPD48_PHYCC1CMP_CC1DETDEF_Pos)          /*!< UTCPD_T::PHYCC1CMP: CC1DETDEF Mask     */

#define NPD48_PHYCC1CMP_CC1DET15_Pos     (3)                                               /*!< UTCPD_T::PHYCC1CMP: CC1DET15 Position  */
#define NPD48_PHYCC1CMP_CC1DET15_Msk     (0x1ul << NPD48_PHYCC1CMP_CC1DET15_Pos)           /*!< UTCPD_T::PHYCC1CMP: CC1DET15 Mask      */

#define NPD48_PHYCC1CMP_CC1DET3A_Pos     (4)                                               /*!< UTCPD_T::PHYCC1CMP: CC1DET3A Position  */
#define NPD48_PHYCC1CMP_CC1DET3A_Msk     (0x1ul << NPD48_PHYCC1CMP_CC1DET3A_Pos)           /*!< UTCPD_T::PHYCC1CMP: CC1DET3A Mask      */

#define NPD48_PHYCC1CMP_CC1STS_Pos       (5)                                               /*!< UTCPD_T::PHYCC1CMP: CC1STS Position    */
#define NPD48_PHYCC1CMP_CC1STS_Msk       (0x7ul << NPD48_PHYCC1CMP_CC1STS_Pos)             /*!< UTCPD_T::PHYCC1CMP: CC1STS Mask        */

#define NPD48_PHYCC2CMP_CC2DETRA_Pos     (0)                                               /*!< UTCPD_T::PHYCC2CMP: CC2DETRA Position  */
#define NPD48_PHYCC2CMP_CC2DETRA_Msk     (0x1ul << NPD48_PHYCC2CMP_CC2DETRA_Pos)           /*!< UTCPD_T::PHYCC2CMP: CC2DETRA Mask      */

#define NPD48_PHYCC2CMP_CC2DETRD_Pos     (1)                                               /*!< UTCPD_T::PHYCC2CMP: CC2DETRD Position  */
#define NPD48_PHYCC2CMP_CC2DETRD_Msk     (0x1ul << NPD48_PHYCC2CMP_CC2DETRD_Pos)           /*!< UTCPD_T::PHYCC2CMP: CC2DETRD Mask      */

#define NPD48_PHYCC2CMP_CC2DETDEF_Pos    (2)                                               /*!< UTCPD_T::PHYCC2CMP: CC2DETDEF Position */
#define NPD48_PHYCC2CMP_CC2DETDEF_Msk    (0x1ul << NPD48_PHYCC2CMP_CC2DETDEF_Pos)          /*!< UTCPD_T::PHYCC2CMP: CC2DETDEF Mask     */

#define NPD48_PHYCC2CMP_CC2DET15_Pos     (3)                                               /*!< UTCPD_T::PHYCC2CMP: CC2DET15 Position  */
#define NPD48_PHYCC2CMP_CC2DET15_Msk     (0x1ul << NPD48_PHYCC2CMP_CC2DET15_Pos)           /*!< UTCPD_T::PHYCC2CMP: CC2DET15 Mask      */

#define NPD48_PHYCC2CMP_CC2DET3A_Pos     (4)                                               /*!< UTCPD_T::PHYCC2CMP: CC2DET3A Position  */
#define NPD48_PHYCC2CMP_CC2DET3A_Msk     (0x1ul << NPD48_PHYCC2CMP_CC2DET3A_Pos)           /*!< UTCPD_T::PHYCC2CMP: CC2DET3A Mask      */

#define NPD48_PHYCC2CMP_CC2STS_Pos       (5)                                               /*!< UTCPD_T::PHYCC2CMP: CC2STS Position    */
#define NPD48_PHYCC2CMP_CC2STS_Msk       (0x7ul << NPD48_PHYCC2CMP_CC2STS_Pos)             /*!< UTCPD_T::PHYCC2CMP: CC2STS Mask        */

#define NPD48_VDRINIT_PDVER_Pos          (1)                                               /*!< UTCPD_T::VDRINIT: PDVER Position       */
#define NPD48_VDRINIT_PDVER_Msk          (0x1ul << NPD48_VDRINIT_PDVER_Pos)                /*!< UTCPD_T::VDRINIT: PDVER Mask           */

#define NPD48_VDRINIT_DEVCAPDEF_Pos      (3)                                               /*!< UTCPD_T::VDRINIT: DEVCAPDEF Position   */
#define NPD48_VDRINIT_DEVCAPDEF_Msk      (0x1ul << NPD48_VDRINIT_DEVCAPDEF_Pos)            /*!< UTCPD_T::VDRINIT: DEVCAPDEF Mask       */

#define NPD48_VDSNKDIS_VDSNKDISRC_Pos    (0)                                               /*!< UTCPD_T::VDSNKDIS: VDSNKDISRC Position */
#define NPD48_VDSNKDIS_VDSNKDISRC_Msk    (0x1ul << NPD48_VDSNKDIS_VDSNKDISRC_Pos)          /*!< UTCPD_T::VDSNKDIS: VDSNKDISRC Mask     */

#define NPD48_VDSNKDIS_CCSNKOPNEN_Pos    (1)                                               /*!< UTCPD_T::VDSNKDIS: CCSNKOPNEN Position */
#define NPD48_VDSNKDIS_CCSNKOPNEN_Msk    (0x1ul << NPD48_VDSNKDIS_CCSNKOPNEN_Pos)          /*!< UTCPD_T::VDSNKDIS: CCSNKOPNEN Mask     */

#define NPD48_VDSNKDIS_DISSNKOPNEN_Pos   (2)                                               /*!< UTCPD_T::VDSNKDIS: DISSNKOPNEN Position*/
#define NPD48_VDSNKDIS_DISSNKOPNEN_Msk   (0x1ul << NPD48_VDSNKDIS_DISSNKOPNEN_Pos)         /*!< UTCPD_T::VDSNKDIS: DISSNKOPNEN Mask    */

#define NPD48_VDSNKDIS_CONRESEN_Pos      (3)                                               /*!< UTCPD_T::VDSNKDIS: CONRESEN Position   */
#define NPD48_VDSNKDIS_CONRESEN_Msk      (0x1ul << NPD48_VDSNKDIS_CONRESEN_Pos)            /*!< UTCPD_T::VDSNKDIS: CONRESEN Mask       */

#define NPD48_VDSNKDIS_CCSRCOPNEN_Pos    (5)                                               /*!< UTCPD_T::VDSNKDIS: CCSRCOPNEN Position */
#define NPD48_VDSNKDIS_CCSRCOPNEN_Msk    (0x3ul << NPD48_VDSNKDIS_CCSRCOPNEN_Pos)          /*!< UTCPD_T::VDSNKDIS: CCSRCOPNEN Mask     */

#define NPD48_VS0VDISLO_VBSNKDCSTS_Pos   (0)                                               /*!< UTCPD_T::VS0VDISLO: VBSNKDCSTS Position*/
#define NPD48_VS0VDISLO_VBSNKDCSTS_Msk   (0x7ul << NPD48_VS0VDISLO_VBSNKDCSTS_Pos)         /*!< UTCPD_T::VS0VDISLO: VBSNKDCSTS Mask    */

#define NPD48_VS0VDISLO_VBSRCDCSTS_Pos   (3)                                               /*!< UTCPD_T::VS0VDISLO: VBSRCDCSTS Position*/
#define NPD48_VS0VDISLO_VBSRCDCSTS_Msk   (0x7ul << NPD48_VS0VDISLO_VBSRCDCSTS_Pos)         /*!< UTCPD_T::VS0VDISLO: VBSRCDCSTS Mask    */

#define NPD48_VS0VDISLO_VBSNKDISCTHL_Pos (6)                                               /*!< UTCPD_T::VS0VDISLO: VBSNKDISCTHL Position*/
#define NPD48_VS0VDISLO_VBSNKDISCTHL_Msk (0x1ul << NPD48_VS0VDISLO_VBSNKDISCTHL_Pos)       /*!< UTCPD_T::VS0VDISLO: VBSNKDISCTHL Mask  */

#define NPD48_VS0VDISLO_VBSTPDCTHL_Pos   (7)                                               /*!< UTCPD_T::VS0VDISLO: VBSTPDCTHL Position*/
#define NPD48_VS0VDISLO_VBSTPDCTHL_Msk   (0x1ul << NPD48_VS0VDISLO_VBSTPDCTHL_Pos)         /*!< UTCPD_T::VS0VDISLO: VBSTPDCTHL Mask    */

#define NPD48_VBRTOCTL_VBRTOEN_Pos       (0)                                               /*!< UTCPD_T::VBRTOCTL: VBRTOEN Position    */
#define NPD48_VBRTOCTL_VBRTOEN_Msk       (0x1ul << NPD48_VBRTOCTL_VBRTOEN_Pos)             /*!< UTCPD_T::VBRTOCTL: VBRTOEN Mask        */

#define NPD48_ITNERTSTS_VBADCDONE_Pos    (3)                                               /*!< UTCPD_T::ITNERTSTS: VBADCDONE Position */
#define NPD48_ITNERTSTS_VBADCDONE_Msk    (0x1ul << NPD48_ITNERTSTS_VBADCDONE_Pos)          /*!< UTCPD_T::ITNERTSTS: VBADCDONE Mask     */

#define NPD48_ITNERTSTS_VBS0VDET_Pos     (4)                                               /*!< UTCPD_T::ITNERTSTS: VBS0VDET Position  */
#define NPD48_ITNERTSTS_VBS0VDET_Msk     (0x1ul << NPD48_ITNERTSTS_VBS0VDET_Pos)           /*!< UTCPD_T::ITNERTSTS: VBS0VDET Mask      */

#define NPD48_ITNERTSTS_ADCVUD_Pos       (5)                                               /*!< UTCPD_T::ITNERTSTS: ADCVUD Position    */
#define NPD48_ITNERTSTS_ADCVUD_Msk       (0x1ul << NPD48_ITNERTSTS_ADCVUD_Pos)             /*!< UTCPD_T::ITNERTSTS: ADCVUD Mask        */

#define NPD48_ITNERTSTS_VCONDET_Pos      (6)                                               /*!< UTCPD_T::ITNERTSTS: VCONDET Position   */
#define NPD48_ITNERTSTS_VCONDET_Msk      (0x1ul << NPD48_ITNERTSTS_VCONDET_Pos)            /*!< UTCPD_T::ITNERTSTS: VCONDET Mask       */

#define NPD48_ITNERTSTS_VBDET_Pos        (7)                                               /*!< UTCPD_T::ITNERTSTS: VBDET Position     */
#define NPD48_ITNERTSTS_VBDET_Msk        (0x1ul << NPD48_ITNERTSTS_VBDET_Pos)              /*!< UTCPD_T::ITNERTSTS: VBDET Mask         */

#define NPD48_CTLSTS_BISTM2_Pos          (0)                                               /*!< UTCPD_T::CTLSTS: BISTM2 Position       */
#define NPD48_CTLSTS_BISTM2_Msk          (0x1ul << NPD48_CTLSTS_BISTM2_Pos)                /*!< UTCPD_T::CTLSTS: BISTM2 Mask           */

#define NPD48_CTLSTS_FOROFFVB_Pos        (1)                                               /*!< UTCPD_T::CTLSTS: FOROFFVB Position     */
#define NPD48_CTLSTS_FOROFFVB_Msk        (0x1ul << NPD48_CTLSTS_FOROFFVB_Pos)              /*!< UTCPD_T::CTLSTS: FOROFFVB Mask         */

#define NPD48_CTLSTS_CHTCKTDET_Pos       (2)                                               /*!< UTCPD_T::CTLSTS: CHTCKTDET Position    */
#define NPD48_CTLSTS_CHTCKTDET_Msk       (0x1ul << NPD48_CTLSTS_CHTCKTDET_Pos)             /*!< UTCPD_T::CTLSTS: CHTCKTDET Mask        */

#define NPD48_CTLSTS_OVTMPDET_Pos        (3)                                               /*!< UTCPD_T::CTLSTS: OVTMPDET Position     */
#define NPD48_CTLSTS_OVTMPDET_Msk        (0x1ul << NPD48_CTLSTS_OVTMPDET_Pos)              /*!< UTCPD_T::CTLSTS: OVTMPDET Mask         */

#define NPD48_CTLSTS_RXBUFSTS_Pos        (4)                                               /*!< UTCPD_T::CTLSTS: RXBUFSTS Position     */
#define NPD48_CTLSTS_RXBUFSTS_Msk        (0x7ul << NPD48_CTLSTS_RXBUFSTS_Pos)              /*!< UTCPD_T::CTLSTS: RXBUFSTS Mask         */

#define NPD48_CTLSTS_TXBUFRDY_Pos        (7)                                               /*!< UTCPD_T::CTLSTS: TXBUFRDY Position     */
#define NPD48_CTLSTS_TXBUFRDY_Msk        (0x1ul << NPD48_CTLSTS_TXBUFRDY_Pos)              /*!< UTCPD_T::CTLSTS: TXBUFRDY Mask         */

#define NPD48_CTLSTS2_TCACTM_Pos         (0)                                               /*!< UTCPD_T::CTLSTS2: TCACTM Position      */
#define NPD48_CTLSTS2_TCACTM_Msk         (0x1ul << NPD48_CTLSTS2_TCACTM_Pos)               /*!< UTCPD_T::CTLSTS2: TCACTM Mask          */

#define NPD48_CTLSTS2_TXPCKSTS_Pos       (1)                                               /*!< UTCPD_T::CTLSTS2: TXPCKSTS Position    */
#define NPD48_CTLSTS2_TXPCKSTS_Msk       (0x7ul << NPD48_CTLSTS2_TXPCKSTS_Pos)             /*!< UTCPD_T::CTLSTS2: TXPCKSTS Mask        */

#define NPD48_CTLSTS2_RXPCKSTS_Pos       (4)                                               /*!< UTCPD_T::CTLSTS2: RXPCKSTS Position    */
#define NPD48_CTLSTS2_RXPCKSTS_Msk       (0x7ul << NPD48_CTLSTS2_RXPCKSTS_Pos)             /*!< UTCPD_T::CTLSTS2: RXPCKSTS Mask        */

#define NPD48_CTLSTS2_RXPCKINBUF_Pos     (7)                                               /*!< UTCPD_T::CTLSTS2: RXPCKINBUF Position  */
#define NPD48_CTLSTS2_RXPCKINBUF_Msk     (0x1ul << NPD48_CTLSTS2_RXPCKINBUF_Pos)           /*!< UTCPD_T::CTLSTS2: RXPCKINBUF Mask      */

#define NPD48_BMCTXBP_BITPRD_Pos         (0)                                               /*!< UTCPD_T::BMCTXBP: BITPRD Position      */
#define NPD48_BMCTXBP_BITPRD_Msk         (0xfful << NPD48_BMCTXBP_BITPRD_Pos)              /*!< UTCPD_T::BMCTXBP: BITPRD Mask          */

#define NPD48_BMCTXDR_DUTYSET2_Pos       (0)                                               /*!< UTCPD_T::BMCTXDR: DUTYSET2 Position    */
#define NPD48_BMCTXDR_DUTYSET2_Msk       (0x7ful << NPD48_BMCTXDR_DUTYSET2_Pos)            /*!< UTCPD_T::BMCTXDR: DUTYSET2 Mask        */

#define NPD48_BMCTXDR_DUTYSET1_Pos       (7)                                               /*!< UTCPD_T::BMCTXDR: DUTYSET1 Position    */
#define NPD48_BMCTXDR_DUTYSET1_Msk       (0x1ul << NPD48_BMCTXDR_DUTYSET1_Pos)             /*!< UTCPD_T::BMCTXDR: DUTYSET1 Mask        */

#define NPD48_SLICECR_SLICELS_Pos        (0)                                               /*!< UTCPD_T::SLICECR: SLICELS Position     */
#define NPD48_SLICECR_SLICELS_Msk        (0x3ul << NPD48_SLICECR_SLICELS_Pos)              /*!< UTCPD_T::SLICECR: SLICELS Mask         */

#define NPD48_SLICECR_SLICEHS_Pos        (2)                                               /*!< UTCPD_T::SLICECR: SLICEHS Position     */
#define NPD48_SLICECR_SLICEHS_Msk        (0x3ul << NPD48_SLICECR_SLICEHS_Pos)              /*!< UTCPD_T::SLICECR: SLICEHS Mask         */

#define NPD48_SLICECR_SLICEMS_Pos        (4)                                               /*!< UTCPD_T::SLICECR: SLICEMS Position     */
#define NPD48_SLICECR_SLICEMS_Msk        (0x7ul << NPD48_SLICECR_SLICEMS_Pos)              /*!< UTCPD_T::SLICECR: SLICEMS Mask         */

///////
#define NPD48_PEIF_OVPIF_Pos             (0)                                               /*!< UTCPD_T::PEIF:CCOVPIF Poition          */
#define NPD48_PEIF_OVPIF_Msk             (0x1ul << NPD48_PEIF_OVPIF_Pos)                   /*!< UTCPD_T::PEIF:CCOVPIF Mask             */

#define NPD48_PEIF_UVPIF_Pos             (1)                                               /*!< UTCPD_T::PEIF:VCOCPIF Poition          */
#define NPD48_PEIF_UVPIF_Msk             (0x1ul << NPD48_PEIF_UVPIF_Pos)                   /*!< UTCPD_T::PEIF:VCOCPIF Mask             */

#define NPD48_PEIF_OCPIF_Pos             (2)                                               /*!< UTCPD_T::PEIF:CCOVPIF Poition          */
#define NPD48_PEIF_OCPIF_Msk             (0x1ul << NPD48_PEIF_OCPIF_Pos)                   /*!< UTCPD_T::PEIF:CCOVPIF Mask             */

#define NPD48_PEIF_NTCPIF_Pos            (3)                                               /*!< UTCPD_T::PEIF:VCOCPIF Poition          */
#define NPD48_PEIF_NTCPIF_Msk            (0x1ul << NPD48_PEIF_NTCPIF_Pos)                 /*!< UTCPD_T::PEIF:VCOCPIF Mask             */

#define NPD48_PEIF_CCOVPIF_Pos           (4)                                               /*!< UTCPD_T::PEIF:CCOVPIF Poition          */
#define NPD48_PEIF_CCOVPIF_Msk           (0x1ul << NPD48_PEIF_CCOVPIF_Pos)                 /*!< UTCPD_T::PEIF:CCOVPIF Mask             */

#define NPD48_PEIF_VCOCPIF_Pos           (5)                                               /*!< UTCPD_T::PEIF:VCOCPIF Poition          */
#define NPD48_PEIF_VCOCPIF_Msk           (0x1ul << NPD48_PEIF_VCOCPIF_Pos)                 /*!< UTCPD_T::PEIF:VCOCPIF Mask             */

#define NPD48_PEIF_COVPIF_Pos            (6)                                               /*!< UTCPD_T::PEIF:CCOVPIF Poition          */
#define NPD48_PEIF_COVPIF_Msk            (0x1ul << NPD48_PEIF_COVPIF_Pos)                  /*!< UTCPD_T::PEIF:CCOVPIF Mask             */

#define NPD48_PEIF_ITPIF_Pos             (7)                                               /*!< UTCPD_T::PEIF:VCOCPIF Poition          */
#define NPD48_PEIF_ITPIF_Msk             (0x1ul << NPD48_PEIF_ITPIF_Pos)                   /*!< UTCPD_T::PEIF:VCOCPIF Mask             */

#define NPD48_PEIE_OVPIE_Pos             (0)                                               /*!< UTCPD_T::PEIE:OVPIE Poition            */
#define NPD48_PEIE_OVPIE_Msk             (0x1ul << NPD48_PEIE_OVPIE_Pos)                   /*!< UTCPD_T::PEIE:OVPIE Mask               */

#define NPD48_PEIE_UVPIE_Pos             (1)                                               /*!< UTCPD_T::PEIE:UVPIE Poition            */
#define NPD48_PEIE_UVPIE_Msk             (0x1ul << NPD48_PEIE_UVPIE_Pos)                   /*!< UTCPD_T::PEIE:UVPIE Mask               */

#define NPD48_PEIE_OCPIE_Pos             (2)                                               /*!< UTCPD_T::PEIE:OCPIE Poition            */
#define NPD48_PEIE_OCPIE_Msk             (0x1ul << NPD48_PEIE_OCPIE_Pos)                   /*!< UTCPD_T::PEIE:OCPIE Mask               */

#define NPD48_PEIE_NTCPIE_Pos            (3)                                               /*!< UTCPD_T::PEIE:NTCPIE Poition           */
#define NPD48_PEIE_NTCPIE_Msk            (0x1ul << NPD48_PEIE_NTCPIE_Pos)                  /*!< UTCPD_T::PEIE:NTCPIE Mask              */

#define NPD48_PEIE_CCOVPIE_Pos           (4)                                               /*!< UTCPD_T::PEIE:CCOVPIE Poition          */
#define NPD48_PEIE_CCOVPIE_Msk           (0x1ul << NPD48_PEIE_CCOVPIE_Pos)                 /*!< UTCPD_T::PEIE:CCOVPIE Mask             */

#define NPD48_PEIE_VCOCPIE_Pos           (5)                                               /*!< UTCPD_T::PEIE:VCOCPIE Poition          */
#define NPD48_PEIE_VCOCPIE_Msk           (0x1ul << NPD48_PEIE_VCOCPIE_Pos)                 /*!< UTCPD_T::PEIE:VCOCPIE Mask             */

#define NPD48_PECTL_CCOVPEN_Pos           (4)                                              /*!< UTCPD_T::PECTL:CCOVPEN Poition          */
#define NPD48_PECTL_CCOVPEN_Msk           (0x1ul << NPD48_PECTL_CCOVPEN_Pos)               /*!< UTCPD_T::PECTL:CCOVPEN Mask             */

#define NPD48_PECTL_VCOCPEN_Pos           (5)                                              /*!< UTCPD_T::PECTL:VCOCPEN Poition          */
#define NPD48_PECTL_VCOCPEN_Msk           (0x1ul << NPD48_PECTL_VCOCPEN_Pos)               /*!< UTCPD_T::PECTL:VCOCPEN Mask             */

#define NPD48_DISCCTL_VBDISC1_Pos         (0)                                              /*!< UTCPD_T::DISCCTL:VBDISC1 Position       */		
#define NPD48_DISCCTL_VBDISC1_Msk         (0x1ul << NPD48_DISCCTL_VBDISC1_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC1 Mask           */

#define NPD48_DISCCTL_VBDISC2_Pos         (1)                                              /*!< UTCPD_T::DISCCTL:VBDISC2 Position       */		
#define NPD48_DISCCTL_VBDISC2_Msk         (0x1ul << NPD48_DISCCTL_VBDISC2_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC2 Mask           */

#define NPD48_DISCCTL_VBDISC3_Pos         (2)                                              /*!< UTCPD_T::DISCCTL:VBDISC3 Position       */		
#define NPD48_DISCCTL_VBDISC3_Msk         (0x1ul << NPD48_DISCCTL_VBDISC3_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC3 Mask           */

#define NPD48_DISCCTL_VBDISC4_Pos         (3)                                              /*!< UTCPD_T::DISCCTL:VBDISC4 Position       */		
#define NPD48_DISCCTL_VBDISC4_Msk         (0x1ul << NPD48_DISCCTL_VBDISC4_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC4 Mask           */

#define NPD48_DISCCTL_VINBDEN_Pos         (4)                                              /*!< UTCPD_T::DISCCTL:VINBDEN Position       */		
#define NPD48_DISCCTL_VINBDEN_Msk         (0x1ul << NPD48_DISCCTL_VINBDEN_Pos)             /*!< UTCPD_T::DISCCTL:VINBDEN Mask           */

#define NPD48_DISCCTL_VINBDLVL_Pos        (5)                                              /*!< UTCPD_T::DISCCTL:VINBDLVL Position      */		
#define NPD48_DISCCTL_VINBDLVL_Msk        (0x1ul << NPD48_DISCCTL_VINBDLVL_Pos)            /*!< UTCPD_T::DISCCTL:VINBDLVL Mask          */

#define NPD48_DISCSTS_VBDISC1_Pos         (0)                                              /*!< UTCPD_T::DISCCTL:VBDISC1 Position       */		
#define NPD48_DISCSTS_VBDISC1_Msk         (0x1ul << NPD48_DISCSTS_VBDISC1_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC1 Mask           */

#define NPD48_DISCSTS_VBDISC2_Pos         (1)                                              /*!< UTCPD_T::DISCCTL:VBDISC2 Position       */		
#define NPD48_DISCSTS_VBDISC2_Msk         (0x1ul << NPD48_DISCSTS_VBDISC2_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC2 Mask           */

#define NPD48_DISCSTS_VBDISC3_Pos         (2)                                              /*!< UTCPD_T::DISCCTL:VBDISC3 Position       */		
#define NPD48_DISCSTS_VBDISC3_Msk         (0x1ul << NPD48_DISCSTS_VBDISC3_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC3 Mask           */

#define NPD48_DISCSTS_VBDISC4_Pos         (3)                                              /*!< UTCPD_T::DISCCTL:VBDISC4 Position       */		
#define NPD48_DISCSTS_VBDISC4_Msk         (0x1ul << NPD48_DISCSTSL_VBDISC4_Pos)             /*!< UTCPD_T::DISCCTL:VBDISC4 Mask           */

#define NPD48_DISCSTS_VINBDEN_Pos         (4)                                              /*!< UTCPD_T::DISCCTL:VINBDEN Position       */		
#define NPD48_DISCSTS_VINBDEN_Msk         (0x1ul << NPD48_DISCSTS_VINBDEN_Pos)             /*!< UTCPD_T::DISCCTL:VINBDEN Mask           */

#define NPD48_DISCSTS_VINBDLVL_Pos        (5)                                              /*!< UTCPD_T::DISCCTL:VINBDLVL Position      */		
#define NPD48_DISCSTS_VINBDLVL_Msk        (0x1ul << NPD48_DISCSTS_VINBDLVL_Pos)            /*!< UTCPD_T::DISCCTL:VINBDLVL Mask          */

/**@}*/ /* UTCPD_CONST */
/**@}*/ /* end of UTCPD register group */
