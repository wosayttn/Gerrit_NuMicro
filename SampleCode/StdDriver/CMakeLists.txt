create_project(ACMP_CompareVBG ${CMAKE_CURRENT_LIST_DIR}/ACMP_CompareVBG/main.c)
create_project(ACMP_Wakeup ${CMAKE_CURRENT_LIST_DIR}/ACMP_Wakeup/main.c)
create_project(ACMP_WindowCompare ${CMAKE_CURRENT_LIST_DIR}/ACMP_WindowCompare/main.c)

create_project(CANFD_CANFD_Loopback ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_Loopback/main.c)
create_project(CANFD_CANFD_TxRx ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_TxRx/main.c)
create_project(CANFD_CANFD_TxRxINT ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_TxRxINT/main.c)

create_project(CANFD_CAN_Loopback ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_Loopback/main.c)
create_project(CANFD_CAN_TxRx ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_TxRx/main.c)
create_project(CANFD_CAN_TxRxINT ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_TxRxINT/main.c)
create_project(CLK_ClockDetector ${CMAKE_CURRENT_LIST_DIR}/CLK_ClockDetector/main.c)

# create_project(CLK_ClockDetectorWakeup ${CMAKE_CURRENT_LIST_DIR}/CLK_ClockDetectorWakeup/main.c)
create_project(FMC_APWPROT ${CMAKE_CURRENT_LIST_DIR}/FMC_APWPROT/main.c)
create_project(FMC_CRC32 ${CMAKE_CURRENT_LIST_DIR}/FMC_CRC32/main.c)
# create_project(FMC_DualBank ${CMAKE_CURRENT_LIST_DIR}/FMC_DualBank/main.c)

add_subdirectory(FMC_DualBankFwUpdate)
create_project(FMC_ExeInSRAM ${CMAKE_CURRENT_LIST_DIR}/FMC_ExeInSRAM/main.c ${CMAKE_CURRENT_LIST_DIR}/FMC_ExeInSRAM/exeinsram.c)

add_subdirectory(FMC_FwUpdateApplication)
# add_subdirectory(FMC_IAP)
create_project(FMC_MultiBoot ${CMAKE_CURRENT_LIST_DIR}/FMC_MultiBoot/main.c)

# create_project(FMC_MultiWordProgram ${CMAKE_CURRENT_LIST_DIR}/FMC_MultiWordProgram/main.c)
create_project(FMC_ReadAllOne ${CMAKE_CURRENT_LIST_DIR}/FMC_ReadAllOne/main.c)
create_project(FMC_RW ${CMAKE_CURRENT_LIST_DIR}/FMC_RW/main.c)

# create_project(FMC_SPROM ${CMAKE_CURRENT_LIST_DIR}/FMC_SPROM/main.c ${CMAKE_CURRENT_LIST_DIR}/FMC_SPROM/sprom.c)
create_project(GPIO_EINTAndDebounce ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTAndDebounce/main.c)

create_project(GPIO_EINTTriggerPDMA ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerPDMA/main.c)
# create_project(GPIO_EINTTriggerPWM ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerPWM/main.c)
create_project(GPIO_INT ${CMAKE_CURRENT_LIST_DIR}/GPIO_INT/main.c)
create_project(GPIO_OutputInput ${CMAKE_CURRENT_LIST_DIR}/GPIO_OutputInput/main.c)
create_project(GPIO_PowerDown ${CMAKE_CURRENT_LIST_DIR}/GPIO_PowerDown/main.c)

create_project(I2S_MP3PLAYER ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/main.c
                ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/audio_codec.c
                ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/isr.c
                ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/mp3.c
                ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/mp3headerparser.c
                ${CMAKE_CURRENT_LIST_DIR}/I2S_MP3PLAYER/pseudofile.c
                )
target_link_libraries(I2S_MP3PLAYER libmad)

# I3C_Controller
create_project(I3C_Controller
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Controller/main.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Controller/i3c_ControllerRole.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Controller/i3c_DeviceFunc.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Controller/i3c_TargetRole.c
)

# I3C_Target
create_project(I3C_Target
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Target/main.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Target/i3c_ControllerRole.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Target/i3c_DeviceFunc.c
    ${CMAKE_CURRENT_LIST_DIR}/I3C_Target/i3c_TargetRole.c
)


create_project(LLSI_Marquee ${CMAKE_CURRENT_LIST_DIR}/LLSI_Marquee/main.c)
create_project(LLSI_PDMA_Marquee ${CMAKE_CURRENT_LIST_DIR}/LLSI_PDMA_Marquee/main.c)
create_project(PDMA_BasicMode ${CMAKE_CURRENT_LIST_DIR}/PDMA_BasicMode/main.c)
create_project(PDMA_ScatterGather ${CMAKE_CURRENT_LIST_DIR}/PDMA_ScatterGather/main.c)
create_project(PDMA_ScatterGather_PingPongBuffer ${CMAKE_CURRENT_LIST_DIR}/PDMA_ScatterGather_PingPongBuffer/main.c)

create_project(SPI_Loopback ${CMAKE_CURRENT_LIST_DIR}/SPI_Loopback/main.c)
create_project(SPI_MasterFIFOMode ${CMAKE_CURRENT_LIST_DIR}/SPI_MasterFIFOMode/main.c)
create_project(SPI_SlaveFIFOMode ${CMAKE_CURRENT_LIST_DIR}/SPI_SlaveFIFOMode/main.c)
create_project(SYS_BODWakeup ${CMAKE_CURRENT_LIST_DIR}/SYS_BODWakeup/main.c)
create_project(SYS_PLLClockOutput ${CMAKE_CURRENT_LIST_DIR}/SYS_PLLClockOutput/main.c)

create_project(SYS_TrimHIRC ${CMAKE_CURRENT_LIST_DIR}/SYS_TrimHIRC/main.c)
create_project(UART_AutoBaudRate ${CMAKE_CURRENT_LIST_DIR}/UART_AutoBaudRate/main.c)
create_project(UART_AutoFlow ${CMAKE_CURRENT_LIST_DIR}/UART_AutoFlow/main.c)
create_project(UART_IrDA ${CMAKE_CURRENT_LIST_DIR}/UART_IrDA/main.c)
create_project(UART_LIN ${CMAKE_CURRENT_LIST_DIR}/UART_LIN/main.c)

# create_project(UART_LIN_Wakeup ${CMAKE_CURRENT_LIST_DIR}/UART_LIN_Wakeup/main.c)
create_project(UART_PDMA ${CMAKE_CURRENT_LIST_DIR}/UART_PDMA/main.c)
create_project(UART_RS485 ${CMAKE_CURRENT_LIST_DIR}/UART_RS485/main.c)
create_project(UART_SingleWire ${CMAKE_CURRENT_LIST_DIR}/UART_SingleWire/main.c)
create_project(UART_TxRxFunction ${CMAKE_CURRENT_LIST_DIR}/UART_TxRxFunction/main.c)
create_project(UART_Wakeup ${CMAKE_CURRENT_LIST_DIR}/UART_Wakeup/main.c)
create_project(USCI_SPI_Loopback ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_Loopback/main.c)
create_project(USCI_SPI_MasterMode ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_MasterMode/main.c)

# create_project(USCI_SPI_PDMA_LoopTest ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_PDMA_LoopTest/main.c)
create_project(USCI_SPI_SlaveMode ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_SlaveMode/main.c)
create_project(USCI_UART_AutoBaudRate ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_AutoBaudRate/main.c)
create_project(USCI_UART_AutoFlow ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_AutoFlow/main.c)
create_project(USCI_UART_PDMA ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_PDMA/main.c)
create_project(USCI_UART_RS485 ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_RS485/main.c)
create_project(USCI_UART_TxRxFunction ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_TxRxFunction/main.c)
create_project(USCI_UART_Wakeup ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_Wakeup/main.c)
create_project(QSPI_DualMode_Flash ${CMAKE_CURRENT_LIST_DIR}/QSPI_DualMode_Flash/main.c)
create_project(QSPI_QuadMode_Flash ${CMAKE_CURRENT_LIST_DIR}/QSPI_QuadMode_Flash/main.c)
create_project(QSPI_Slave3Wire ${CMAKE_CURRENT_LIST_DIR}/QSPI_Slave3Wire/main.c)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSOTG_Dual_Role_UMAS)

# --- HSOTG / HSUSBH projects ---
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSOTG_HNP)

create_project(HSUSBH_USBH_AudioClass ${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_AudioClass/main.c)
target_link_libraries(HSUSBH_USBH_AudioClass usbhost_uac)

create_project(HSUSBH_USBH_DEV_CONN ${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_DEV_CONN/main.c)
target_link_libraries(HSUSBH_USBH_DEV_CONN usbhost_core)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_Firmware_Update)

create_project(HSUSBH_USBH_HID ${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_HID/main.c)
target_link_libraries(HSUSBH_USBH_HID usbhost_hid)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_HID_Keyboard)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_HID_Mouse_Keyboard)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_MassStorage)

create_project(HSUSBH_USBH_UAC_HID ${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_UAC_HID/main.c)
target_link_libraries(HSUSBH_USBH_UAC_HID usbhost_uac usbhost_hid)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_UAC_LoopBack)

create_project(HSUSBH_USBH_VCOM ${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_VCOM/main.c)
target_link_libraries(HSUSBH_USBH_VCOM usbhost_cdc)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/HSUSBH_USBH_VENDOR_LBK)

# --- PDCI projects ---
create_project(PDCI_Output ${CMAKE_CURRENT_LIST_DIR}/PDCI_Output/main.c)
create_project(PDCI_PDMA_Output ${CMAKE_CURRENT_LIST_DIR}/PDCI_PDMA_Output/main.c)

# create_project(USBD_VCOM_MultiPort ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/main.c ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/descriptors.c ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/vcom_serial.c)
# --- Additional samples (auto-added) ---
create_project(BPWM_Capture
    ${CMAKE_CURRENT_LIST_DIR}/BPWM_Capture/main.c
)
target_include_directories(BPWM_Capture PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BPWM_Capture)
create_project(BPWM_DoubleBuffer
    ${CMAKE_CURRENT_LIST_DIR}/BPWM_DoubleBuffer/main.c
)
target_include_directories(BPWM_DoubleBuffer PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BPWM_DoubleBuffer)
create_project(BPWM_OutputWaveform
    ${CMAKE_CURRENT_LIST_DIR}/BPWM_OutputWaveform/main.c
)
target_include_directories(BPWM_OutputWaveform PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BPWM_OutputWaveform)
create_project(BPWM_SwitchDuty
    ${CMAKE_CURRENT_LIST_DIR}/BPWM_SwitchDuty/main.c
)
target_include_directories(BPWM_SwitchDuty PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BPWM_SwitchDuty)
create_project(BPWM_SyncStart
    ${CMAKE_CURRENT_LIST_DIR}/BPWM_SyncStart/main.c
)
target_include_directories(BPWM_SyncStart PRIVATE ${CMAKE_CURRENT_LIST_DIR}/BPWM_SyncStart)
create_project(CANFD_CAN_MonitorMode
    ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_MonitorMode/main.c
)
target_include_directories(CANFD_CAN_MonitorMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_MonitorMode)
create_project(CANFD_CANFD_MonitorMode
    ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_MonitorMode/main.c
)
target_include_directories(CANFD_CANFD_MonitorMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_MonitorMode)
create_project(CRC_CCITT
    ${CMAKE_CURRENT_LIST_DIR}/CRC_CCITT/main.c
)
target_include_directories(CRC_CCITT PRIVATE ${CMAKE_CURRENT_LIST_DIR}/CRC_CCITT)
create_project(CRC_CRC32
    ${CMAKE_CURRENT_LIST_DIR}/CRC_CRC32/main.c
)
target_include_directories(CRC_CRC32 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/CRC_CRC32)
create_project(CRC_CRC8
    ${CMAKE_CURRENT_LIST_DIR}/CRC_CRC8/main.c
)
target_include_directories(CRC_CRC8 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/CRC_CRC8)
create_project(EADC_Accumulate
    ${CMAKE_CURRENT_LIST_DIR}/EADC_Accumulate/main.c
)
target_include_directories(EADC_Accumulate PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_Accumulate)
create_project(EADC_ADINT_Trigger
    ${CMAKE_CURRENT_LIST_DIR}/EADC_ADINT_Trigger/main.c
)
target_include_directories(EADC_ADINT_Trigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_ADINT_Trigger)
create_project(EADC_Average
    ${CMAKE_CURRENT_LIST_DIR}/EADC_Average/main.c
)
target_include_directories(EADC_Average PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_Average)
create_project(EADC_BandGap
    ${CMAKE_CURRENT_LIST_DIR}/EADC_BandGap/main.c
)
target_include_directories(EADC_BandGap PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_BandGap)
create_project(EADC_EPWM_Trigger
    ${CMAKE_CURRENT_LIST_DIR}/EADC_EPWM_Trigger/main.c
)
target_include_directories(EADC_EPWM_Trigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_EPWM_Trigger)
create_project(EADC_PDMA_EPWM_Trigger
    ${CMAKE_CURRENT_LIST_DIR}/EADC_PDMA_EPWM_Trigger/main.c
)
target_include_directories(EADC_PDMA_EPWM_Trigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_PDMA_EPWM_Trigger)
create_project(EADC_Pending_Priority
    ${CMAKE_CURRENT_LIST_DIR}/EADC_Pending_Priority/main.c
)
target_include_directories(EADC_Pending_Priority PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_Pending_Priority)
create_project(EADC_ResultMonitor
    ${CMAKE_CURRENT_LIST_DIR}/EADC_ResultMonitor/main.c
)
target_include_directories(EADC_ResultMonitor PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_ResultMonitor)
create_project(EADC_SWTRG_Trigger
    ${CMAKE_CURRENT_LIST_DIR}/EADC_SWTRG_Trigger/main.c
)
target_include_directories(EADC_SWTRG_Trigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_SWTRG_Trigger)
create_project(EADC_TempSensor
    ${CMAKE_CURRENT_LIST_DIR}/EADC_TempSensor/main.c
)
target_include_directories(EADC_TempSensor PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_TempSensor)
create_project(EADC_Timer_Trigger
    ${CMAKE_CURRENT_LIST_DIR}/EADC_Timer_Trigger/main.c
)
target_include_directories(EADC_Timer_Trigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EADC_Timer_Trigger)
create_project(EBI_NOR
    ${CMAKE_CURRENT_LIST_DIR}/EBI_NOR/main.c
    ${CMAKE_CURRENT_LIST_DIR}/EBI_NOR/nor_MX29LV320T.c
)
target_include_directories(EBI_NOR PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EBI_NOR)
create_project(EBI_SRAM
    ${CMAKE_CURRENT_LIST_DIR}/EBI_SRAM/ebi_sram.c
    ${CMAKE_CURRENT_LIST_DIR}/EBI_SRAM/main.c
)
target_include_directories(EBI_SRAM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EBI_SRAM)
create_project(ECAP_GetEQEIFreq
    ${CMAKE_CURRENT_LIST_DIR}/ECAP_GetEQEIFreq/main.c
)
target_include_directories(ECAP_GetEQEIFreq PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ECAP_GetEQEIFreq)
create_project(ECAP_GetInputFreq
    ${CMAKE_CURRENT_LIST_DIR}/ECAP_GetInputFreq/main.c
)
target_include_directories(ECAP_GetInputFreq PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ECAP_GetInputFreq)
create_project(ELLSI_Marquee
    ${CMAKE_CURRENT_LIST_DIR}/ELLSI_Marquee/main.c
)
target_include_directories(ELLSI_Marquee PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ELLSI_Marquee)
create_project(ELLSI_Marquee_Y_Cable
    ${CMAKE_CURRENT_LIST_DIR}/ELLSI_Marquee_Y_Cable/main.c
)
target_include_directories(ELLSI_Marquee_Y_Cable PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ELLSI_Marquee_Y_Cable)
create_project(ELLSI_PDMA_Marquee
    ${CMAKE_CURRENT_LIST_DIR}/ELLSI_PDMA_Marquee/main.c
)
target_include_directories(ELLSI_PDMA_Marquee PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ELLSI_PDMA_Marquee)
create_project(ELLSI_PDMA_Marquee_Y_Cable
    ${CMAKE_CURRENT_LIST_DIR}/ELLSI_PDMA_Marquee_Y_Cable/main.c
)
target_include_directories(ELLSI_PDMA_Marquee_Y_Cable PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ELLSI_PDMA_Marquee_Y_Cable)
create_project(EPWM_AccumulatorINT_TriggerPDMA
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_AccumulatorINT_TriggerPDMA/main.c
)
target_include_directories(EPWM_AccumulatorINT_TriggerPDMA PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_AccumulatorINT_TriggerPDMA)
create_project(EPWM_AccumulatorStopMode
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_AccumulatorStopMode/main.c
)
target_include_directories(EPWM_AccumulatorStopMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_AccumulatorStopMode)
create_project(EPWM_Brake
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_Brake/main.c
)
target_include_directories(EPWM_Brake PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_Brake)
create_project(EPWM_Capture
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_Capture/main.c
)
target_include_directories(EPWM_Capture PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_Capture)
create_project(EPWM_DeadTime
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_DeadTime/main.c
)
target_include_directories(EPWM_DeadTime PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_DeadTime)
create_project(EPWM_DoubleBuffer
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_DoubleBuffer/main.c
)
target_include_directories(EPWM_DoubleBuffer PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_DoubleBuffer)
create_project(EPWM_OutputWaveform
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_OutputWaveform/main.c
)
target_include_directories(EPWM_OutputWaveform PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_OutputWaveform)
create_project(EPWM_PDMA_Capture
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_PDMA_Capture/main.c
)
target_include_directories(EPWM_PDMA_Capture PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_PDMA_Capture)
create_project(EPWM_SwitchDuty
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_SwitchDuty/main.c
)
target_include_directories(EPWM_SwitchDuty PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_SwitchDuty)
create_project(EPWM_SyncStart
    ${CMAKE_CURRENT_LIST_DIR}/EPWM_SyncStart/main.c
)
target_include_directories(EPWM_SyncStart PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EPWM_SyncStart)
create_project(EQEI_CompareMatch
    ${CMAKE_CURRENT_LIST_DIR}/EQEI_CompareMatch/main.c
)
target_include_directories(EQEI_CompareMatch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/EQEI_CompareMatch)
create_project(FMC_ECC
    ${CMAKE_CURRENT_LIST_DIR}/FMC_ECC/main.c
)
target_include_directories(FMC_ECC PRIVATE ${CMAKE_CURRENT_LIST_DIR}/FMC_ECC)
add_subdirectory(FMC_IAP)
create_project(FMC_XOM
    ${CMAKE_CURRENT_LIST_DIR}/FMC_XOM/main.c
    ${CMAKE_CURRENT_LIST_DIR}/FMC_XOM/xom_add.c
)
target_include_directories(FMC_XOM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/FMC_XOM)
create_project(GPIO_EINTTriggerEPWM
    ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerEPWM/main.c
)
target_include_directories(GPIO_EINTTriggerEPWM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerEPWM)
create_project(GPIO_EINTTriggerTPWM
    ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerTPWM/main.c
)
target_include_directories(GPIO_EINTTriggerTPWM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerTPWM)
create_project(HSUSBD_Audio10_Codec
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec/audio_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec/config_pdma.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec/usbd_audio.c
)
target_include_directories(HSUSBD_Audio10_Codec PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Codec)
create_project(HSUSBD_Audio10_Headset
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/audio_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/config_pdma.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/hid.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/resample.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset/usbd_audio.c
)
target_include_directories(HSUSBD_Audio10_Headset PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio10_Headset)
create_project(HSUSBD_Audio20_Codec
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec/audio_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec/config_pdma.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec/usbd_audio.c
)
target_include_directories(HSUSBD_Audio20_Codec PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Codec)
create_project(HSUSBD_Audio20_Headset
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/audio_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/config_pdma.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/hid.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset/usbd_audio.c
)
target_include_directories(HSUSBD_Audio20_Headset PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Audio20_Headset)
create_project(HSUSBD_HID_Keyboard
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Keyboard/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Keyboard/hid_keyboard.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Keyboard/main.c
)
target_include_directories(HSUSBD_HID_Keyboard PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Keyboard)
create_project(HSUSBD_HID_Mouse
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse/hid_mouse.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse/main.c
)
target_include_directories(HSUSBD_HID_Mouse PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse)
create_project(HSUSBD_HID_Mouse_BC12
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse_BC12/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse_BC12/hid_mouse.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse_BC12/main.c
)
target_include_directories(HSUSBD_HID_Mouse_BC12 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Mouse_BC12)
create_project(HSUSBD_HID_MouseKeyboard
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_MouseKeyboard/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_MouseKeyboard/hid_mousekeyboard.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_MouseKeyboard/main.c
)
target_include_directories(HSUSBD_HID_MouseKeyboard PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_MouseKeyboard)
create_project(HSUSBD_HID_Transfer
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer/hid_transfer.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer/main.c
)
target_include_directories(HSUSBD_HID_Transfer PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer)
create_project(HSUSBD_HID_Transfer_And_MSC
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer_And_MSC/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer_And_MSC/hid_transfer_and_msc.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer_And_MSC/main.c
)
target_include_directories(HSUSBD_HID_Transfer_And_MSC PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_HID_Transfer_And_MSC)
create_project(HSUSBD_Mass_Storage_CDROM
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_CDROM/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_CDROM/diskimage.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_CDROM/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_CDROM/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_CDROM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_CDROM)
create_project(HSUSBD_Mass_Storage_Flash
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_Flash/dataflashprog.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_Flash/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_Flash/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_Flash/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_Flash PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_Flash)
create_project(HSUSBD_Mass_Storage_ScatterGather
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ScatterGather/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ScatterGather/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ScatterGather/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_ScatterGather PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ScatterGather)
create_project(HSUSBD_Mass_Storage_SD
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SD/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SD/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SD/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_SD PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SD)
create_project(HSUSBD_Mass_Storage_ShortPacket
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ShortPacket/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ShortPacket/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ShortPacket/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_ShortPacket PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_ShortPacket)
create_project(HSUSBD_Mass_Storage_SRAM
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SRAM/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SRAM/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SRAM/massstorage.c
)
target_include_directories(HSUSBD_Mass_Storage_SRAM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_Mass_Storage_SRAM)
create_project(HSUSBD_VCOM_SerialEmulator
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VCOM_SerialEmulator/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VCOM_SerialEmulator/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VCOM_SerialEmulator/vcom_serial.c
)
target_include_directories(HSUSBD_VCOM_SerialEmulator PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VCOM_SerialEmulator)
create_project(HSUSBD_VENDOR_LBK
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VENDOR_LBK/descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VENDOR_LBK/main.c
    ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VENDOR_LBK/vendor_lbk.c
)
target_include_directories(HSUSBD_VENDOR_LBK PRIVATE ${CMAKE_CURRENT_LIST_DIR}/HSUSBD_VENDOR_LBK)
create_project(I2C_EEPROM
    ${CMAKE_CURRENT_LIST_DIR}/I2C_EEPROM/main.c
)
target_include_directories(I2C_EEPROM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_EEPROM)
create_project(I2C_Loopback
    ${CMAKE_CURRENT_LIST_DIR}/I2C_Loopback/main.c
)
target_include_directories(I2C_Loopback PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_Loopback)
create_project(I2C_Master
    ${CMAKE_CURRENT_LIST_DIR}/I2C_Master/main.c
)
target_include_directories(I2C_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_Master)
create_project(I2C_MultiBytes_Master
    ${CMAKE_CURRENT_LIST_DIR}/I2C_MultiBytes_Master/main.c
)
target_include_directories(I2C_MultiBytes_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_MultiBytes_Master)
create_project(I2C_PDMA_TRX
    ${CMAKE_CURRENT_LIST_DIR}/I2C_PDMA_TRX/main.c
)
target_include_directories(I2C_PDMA_TRX PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_PDMA_TRX)
create_project(I2C_Slave
    ${CMAKE_CURRENT_LIST_DIR}/I2C_Slave/main.c
)
target_include_directories(I2C_Slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_Slave)
create_project(I2C_SMBus
    ${CMAKE_CURRENT_LIST_DIR}/I2C_SMBus/main.c
)
target_include_directories(I2C_SMBus PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_SMBus)
create_project(I2C_Wakeup_Slave
    ${CMAKE_CURRENT_LIST_DIR}/I2C_Wakeup_Slave/main.c
)
target_include_directories(I2C_Wakeup_Slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2C_Wakeup_Slave)
create_project(I2S_Codec
    ${CMAKE_CURRENT_LIST_DIR}/I2S_Codec/isr.c
    ${CMAKE_CURRENT_LIST_DIR}/I2S_Codec/main.c
)
target_include_directories(I2S_Codec PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2S_Codec)
create_project(I2S_Codec_PDMA
    ${CMAKE_CURRENT_LIST_DIR}/I2S_Codec_PDMA/main.c
)
target_include_directories(I2S_Codec_PDMA PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2S_Codec_PDMA)
create_project(I2S_WAVPLAYER
    ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER/diskio.c
    ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER/isr.c
    ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER/main.c
    ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER/sdglue.c
    ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER/wavplayer.c
)
target_include_directories(I2S_WAVPLAYER PRIVATE ${CMAKE_CURRENT_LIST_DIR}/I2S_WAVPLAYER)
target_link_libraries(I2S_WAVPLAYER fatfs_lib)
# Disabled: main.c includes missing 'global_variable.h'
# create_project(LLSI_Y_Cable_Control
#     ${CMAKE_CURRENT_LIST_DIR}/LLSI_Y_Cable_Control/main.c
# )
# target_include_directories(LLSI_Y_Cable_Control PRIVATE ${CMAKE_CURRENT_LIST_DIR}/LLSI_Y_Cable_Control)
create_project(PDMA_StrideRepeat
    ${CMAKE_CURRENT_LIST_DIR}/PDMA_StrideRepeat/main.c
)
target_include_directories(PDMA_StrideRepeat PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PDMA_StrideRepeat)
create_project(PDMA_TimeOut
    ${CMAKE_CURRENT_LIST_DIR}/PDMA_TimeOut/main.c
)
target_include_directories(PDMA_TimeOut PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PDMA_TimeOut)
create_project(RTC_Alarm_Test
    ${CMAKE_CURRENT_LIST_DIR}/RTC_Alarm_Test/main.c
)
target_include_directories(RTC_Alarm_Test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/RTC_Alarm_Test)
create_project(RTC_Alarm_Wakeup
    ${CMAKE_CURRENT_LIST_DIR}/RTC_Alarm_Wakeup/main.c
)
target_include_directories(RTC_Alarm_Wakeup PRIVATE ${CMAKE_CURRENT_LIST_DIR}/RTC_Alarm_Wakeup)
create_project(RTC_Time_Display
    ${CMAKE_CURRENT_LIST_DIR}/RTC_Time_Display/main.c
)
target_include_directories(RTC_Time_Display PRIVATE ${CMAKE_CURRENT_LIST_DIR}/RTC_Time_Display)
create_project(SDH_FATFS
    ${CMAKE_CURRENT_LIST_DIR}/SDH_FATFS/diskio.c
    ${CMAKE_CURRENT_LIST_DIR}/SDH_FATFS/main.c
    ${CMAKE_CURRENT_LIST_DIR}/SDH_FATFS/SDGlue.c
)
target_include_directories(SDH_FATFS PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SDH_FATFS)
target_link_libraries(SDH_FATFS fatfs_lib)
create_project(SPI_Flash
    ${CMAKE_CURRENT_LIST_DIR}/SPI_Flash/main.c
)
target_include_directories(SPI_Flash PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPI_Flash)
create_project(SPI_HalfDuplex
    ${CMAKE_CURRENT_LIST_DIR}/SPI_HalfDuplex/main.c
)
target_include_directories(SPI_HalfDuplex PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPI_HalfDuplex)
create_project(SPI_PDMA_LoopTest
    ${CMAKE_CURRENT_LIST_DIR}/SPI_PDMA_LoopTest/main.c
)
target_include_directories(SPI_PDMA_LoopTest PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPI_PDMA_LoopTest)
create_project(SPII2S_Master
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_Master/main.c
)
target_include_directories(SPII2S_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_Master)
create_project(SPII2S_PDMA_Codec
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Codec/main.c
)
target_include_directories(SPII2S_PDMA_Codec PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Codec)
create_project(SPII2S_PDMA_Play
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Play/main.c
)
target_include_directories(SPII2S_PDMA_Play PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Play)
create_project(SPII2S_PDMA_PlayRecord
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_PlayRecord/main.c
)
target_include_directories(SPII2S_PDMA_PlayRecord PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_PlayRecord)
create_project(SPII2S_PDMA_Record
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Record/main.c
)
target_include_directories(SPII2S_PDMA_Record PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_PDMA_Record)
create_project(SPII2S_Slave
    ${CMAKE_CURRENT_LIST_DIR}/SPII2S_Slave/main.c
)
target_include_directories(SPII2S_Slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/SPII2S_Slave)
create_project(TIMER_ACMPTrigger
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_ACMPTrigger/main.c
)
target_include_directories(TIMER_ACMPTrigger PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_ACMPTrigger)
create_project(TIMER_CaptureCounter
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_CaptureCounter/main.c
)
target_include_directories(TIMER_CaptureCounter PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_CaptureCounter)
create_project(TIMER_Delay
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_Delay/main.c
)
target_include_directories(TIMER_Delay PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_Delay)
create_project(TIMER_EventCounter
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_EventCounter/main.c
)
target_include_directories(TIMER_EventCounter PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_EventCounter)
create_project(TIMER_FreeCountingMode
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_FreeCountingMode/main.c
)
target_include_directories(TIMER_FreeCountingMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_FreeCountingMode)
create_project(TIMER_InterTimerTriggerMode
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_InterTimerTriggerMode/main.c
)
target_include_directories(TIMER_InterTimerTriggerMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_InterTimerTriggerMode)
create_project(TIMER_Periodic
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_Periodic/main.c
)
target_include_directories(TIMER_Periodic PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_Periodic)
create_project(TIMER_PeriodicINT
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PeriodicINT/main.c
)
target_include_directories(TIMER_PeriodicINT PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PeriodicINT)
create_project(TIMER_PWM_AccumulatorINTStopMode
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_AccumulatorINTStopMode/main.c
)
target_include_directories(TIMER_PWM_AccumulatorINTStopMode PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_AccumulatorINTStopMode)
create_project(TIMER_PWM_AccumulatorINTTriggerPDMA
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_AccumulatorINTTriggerPDMA/main.c
)
target_include_directories(TIMER_PWM_AccumulatorINTTriggerPDMA PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_AccumulatorINTTriggerPDMA)
create_project(TIMER_PWM_Brake
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_Brake/main.c
)
target_include_directories(TIMER_PWM_Brake PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_Brake)
create_project(TIMER_PWM_ChangeDuty
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_ChangeDuty/main.c
)
target_include_directories(TIMER_PWM_ChangeDuty PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_ChangeDuty)
create_project(TIMER_PWM_DeadTime
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_DeadTime/main.c
)
target_include_directories(TIMER_PWM_DeadTime PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_DeadTime)
create_project(TIMER_PWM_OutputWaveform
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_OutputWaveform/main.c
)
target_include_directories(TIMER_PWM_OutputWaveform PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_PWM_OutputWaveform)
create_project(TIMER_TimeoutWakeup
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_TimeoutWakeup/main.c
)
target_include_directories(TIMER_TimeoutWakeup PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_TimeoutWakeup)
create_project(TIMER_ToggleOut
    ${CMAKE_CURRENT_LIST_DIR}/TIMER_ToggleOut/main.c
)
target_include_directories(TIMER_ToggleOut PRIVATE ${CMAKE_CURRENT_LIST_DIR}/TIMER_ToggleOut)
create_project(USCI_I2C_EEPROM
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_EEPROM/main.c
)
target_include_directories(USCI_I2C_EEPROM PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_EEPROM)
create_project(USCI_I2C_Master
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Master/main.c
)
target_include_directories(USCI_I2C_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Master)
create_project(USCI_I2C_Master_10bit
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Master_10bit/main.c
)
target_include_directories(USCI_I2C_Master_10bit PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Master_10bit)
create_project(USCI_I2C_Monitor
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Monitor/main.c
)
target_include_directories(USCI_I2C_Monitor PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Monitor)
create_project(USCI_I2C_MultiBytes_Master
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_MultiBytes_Master/main.c
)
target_include_directories(USCI_I2C_MultiBytes_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_MultiBytes_Master)
create_project(USCI_I2C_SingleByte_Master
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_SingleByte_Master/main.c
)
target_include_directories(USCI_I2C_SingleByte_Master PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_SingleByte_Master)
create_project(USCI_I2C_Slave
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Slave/main.c
)
target_include_directories(USCI_I2C_Slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Slave)
create_project(USCI_I2C_Slave_10bit
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Slave_10bit/main.c
)
target_include_directories(USCI_I2C_Slave_10bit PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Slave_10bit)
create_project(USCI_I2C_Wakeup_Slave
    ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Wakeup_Slave/main.c
)
target_include_directories(USCI_I2C_Wakeup_Slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USCI_I2C_Wakeup_Slave)
create_project(WDT_TimeoutWakeupAndReset
    ${CMAKE_CURRENT_LIST_DIR}/WDT_TimeoutWakeupAndReset/main.c
)
target_include_directories(WDT_TimeoutWakeupAndReset PRIVATE ${CMAKE_CURRENT_LIST_DIR}/WDT_TimeoutWakeupAndReset)
create_project(WWDT_CompareINT
    ${CMAKE_CURRENT_LIST_DIR}/WWDT_CompareINT/main.c
)
target_include_directories(WWDT_CompareINT PRIVATE ${CMAKE_CURRENT_LIST_DIR}/WWDT_CompareINT)
