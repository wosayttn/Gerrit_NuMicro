/**************************************************************************//**
 * @file     pdci_reg.h
 * @version  V3.00
 * @brief    PDCI register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PDCI_REG_H__
#define __PDCI_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup PDCI Parallel Dimming Communication Interface (PDCI)
    Memory Mapped Structure for PDCI Controller
@{ */

typedef struct
{

    /**
     * @var PDCI_T::CTL
     * Offset: 0x00  PDCI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDCIEN    |PDCI Enable Bit
     * |        |          |0 = PDCI function is Disabled.
     * |        |          |1 = PDCI function is Enabled.
     * |[1]     |BWADJ     |Bit Width Adjustment
     * |        |          |0 = The bit time period of Logic '0' is same as Logic '1'.
     * |        |          |1 = The bit time period of Logic '0' is 1.5 times as Logic '1'.
     * |        |          |Note: When this bit is set, Users must consider that the packet timing of the 4 channels in each group will have different lengths due to the difference in data content, which may cause the channel data to be written in time, resulting in the channel content underrun
     * |[2]     |PREAM32   |Preamble Bit Number 32
     * |        |          |0 = The bit number of Preamble is defined by PDCI_CTL2[3:0].
     * |        |          |1 = The bit number of Preamble is 32 bits.
     * |[3]     |DUMLVL    |Dummy Bit Level
     * |        |          |0 = The logic level of dummy bit is low.
     * |        |          |1 = The logic level of dummy bit is high.
     * |[4]     |DMAEN     |PDMA Channel Enable Bit
     * |        |          |0 = PDMA function Disabled.
     * |        |          |1 = PDMA function Enabled.
     * |[7]     |TAILVL    |Tail Bit Level
     * |        |          |0 = The logic level of tail bit is low.
     * |        |          |1 = The logic level of tail bit is high.
     * |[8]     |G0CHEN    |PDCI Group 0 Channel Enable Bit
     * |        |          |0 = PDCI Channel 0~3 Disabled.
     * |        |          |1 = PDCI Channel 0~3 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[9]     |G1CHEN    |PDCI Group 1 Channel Enable Bit
     * |        |          |0 = PDCI Channel 4~7 Disabled.
     * |        |          |1 = PDCI Channel 4~7 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[10]    |G2CHEN    |PDCI Group 2 Channel Enable Bit
     * |        |          |0 = PDCI Channel 8~11 Disabled.
     * |        |          |1 = PDCI Channel 8~11 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[11]    |G3CHEN    |PDCI Group 3 Channel Enable Bit
     * |        |          |0 = PDCI Channel 12~15 Disabled.
     * |        |          |1 = PDCI Channel 12~15 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[12]    |G4CHEN    |PDCI Group 4 Channel Enable Bit
     * |        |          |0 = PDCI Channel 16~19 Disabled.
     * |        |          |1 = PDCI Channel 16~19 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[13]    |G5CHEN    |PDCI Group 5 Channel Enable Bit
     * |        |          |0 = PDCI Channel 20~23 Disabled.
     * |        |          |1 = PDCI Channel 20~23 Enabled.
     * |        |          |Note: This bit is cleared after the group channel transfer done.
     * |[24:16] |BTDIV     |Bit Time Divider
     * |        |          |These bit field indicates the bit time divider for PDCI bit.
     * |        |          |For example, if the HCLK is 200 MHz, the divider can be set as 0x64
     * |        |          |It will generate 2 MHz reference clock and the PDCI transmitting data is sent according to the reference divided clock.
     * |[31:25] |NBTDIV    |Narrow Bit Time Divider
     * |        |          |These bit field indicates the 1st narrow width of data u201C1u201D time divider
     * |        |          |It shall refer to the value of BTDIV.
     * |        |          |For example, if the HCLK is 200 MHz, the NBDIV can be set as 0x1E
     * |        |          |It will generate about 30 % of the PDCI bit if the BTDIV is set as 0x64
     * |        |          |(0x1E/0x64 ~ 30%)
     * @var PDCI_T::DNUM0
     * Offset: 0x04  PDCI Dummy Number Group 0~3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DNUMG0    |Dummy Number Group for Channel 0~3
     * |        |          |The bit field defines the dummy bit number + 1 + one bit data time for the group of channel 0~3
     * |        |          |Each dummy bit equals 8 bits data period.
     * |        |          |Note: The minimum value of DNUMG* shall greater than 1.
     * |[15:8]  |DNUMG1    |Dummy Number Group for Channel 4~7
     * |        |          |The bit field defines the dummy bit number + 1 + one bit data time for the group of channel 4~7
     * |        |          |Each dummy bit equals 8 bits data period.
     * |        |          |Note: The minimum value of DNUMG* shall greater than 1.
     * |[23:16] |DNUMG2    |Dummy Number Group for Channel 8~11
     * |        |          |The bit field defines the dummy bit number+ 1 + one bit data time for the group of channel 8~11
     * |        |          |Each dummy bit equals 8 bits data period.
     * |        |          |Note: The minimum value of DNUMG* shall greater than 1.
     * |[31:24] |DNUMG3    |Dummy Number Group for Channel 12~15
     * |        |          |The bit field defines the dummy bit number + 1 + one bit data time for the group of channel 12~15
     * |        |          |Each dummy bit equals 8 bits data period.
     * |        |          |Note: The minimum value of DNUMG* shall greater than 1.
     * @var PDCI_T::DNUM1
     * Offset: 0x08  PDCI Dummy Number Group 4~5 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DNUMG4    |Dummy Number Group for Channel 16~19
     * |        |          |The bit field defines the dummy bit number for the group of channel 16~19
     * |        |          |Each dummy bit equals 8 bit data period.
     * |        |          |Note: 1. The minimum value of DNUMG* shall greater than 1.
     * |[15:8]  |DNUMG5    |Dummy Number Group for Channel 20~23
     * |        |          |The bit field defines the dummy bit number for the group of channel 20~23
     * |        |          |Each dummy bit equals 8 bit data period.
     * |        |          |Note: The minimum value of DNUMG* shall greater than 1.
     * @var PDCI_T::INTEN
     * Offset: 0x0C  PDCI Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FTXDIEN   |Frame Transmit Done Interrupt Enable Bit
     * |        |          |0 = Frame transmit done interrupt Disabled.
     * |        |          |1 = Frame transmit done interrupt Enabled.
     * |[1]     |TXUNDIEN  |Transmit Data Underrun Interrupt Enable Bit
     * |        |          |0 = Transmit data register underrun interrupt Disabled.
     * |        |          |1 = Transmit data register underrun interrupt Enabled.
     * |[2]     |FEMPTIEN  |FIFO Empty interrupt Enable Bit
     * |        |          |0 = Indicates the channel FIFO empty interrupt Disabled.
     * |        |          |1 = Indicates the channel FIFO empty underrun interrupt Enabled.
     * @var PDCI_T::INTSTS
     * Offset: 0x10  PDCI Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FTXDIF    |Frame Transmit Done Interrupt Flag
     * |        |          |0 = No frame transmit done interrupt flag.
     * |        |          |1 = Frame transmit done interrupt flag. After G*TXDF be clear, this bit also be clear to 0.
     * |[1]     |TXUNDIF   |Transmit Data Register Underrun Interrupt Flag
     * |        |          |0 = No transmit data register underrun interrupt flag.
     * |        |          |1 = Transmit data register under interrupt flag. This bit is the OR function of PDCI_INTSTS[15:8]. 
     * |[2]     |FEMPTIF   |FIFO Empty Interrupt Flag
     * |        |          |0 = No FIFO empty interrupt flag.
     * |        |          |1 = FIFO empty interrupt flag. This bit is the OR function of PDCI_CHEMPTY[31:0]
     * |[8]     |G0TXUND   |Channel 0~3 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 0~3.
     * |        |          |1 = Transmit data underrun active in one of channel 0~3. Write 1 to clear.
     * |[9]     |G1TXUND   |Channel 4~7 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 4~7.
     * |        |          |1 = Transmit data underrun active in one of channel 4~7. Write 1 to clear.
     * |[10]    |G2TXUND   |Channel 8~11 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 8~11.
     * |        |          |1 = Transmit data underrun active in one of channel 8~11. Write 1 to clear.
     * |[11]    |G3TXUND   |Channel 12~15 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 12~15.
     * |        |          |1 = Transmit data underrun active in one of channel 12~15. Write 1 to clear.
     * |[12]    |G4TXUND   |Channel 16~19 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 16~19.
     * |        |          |1 = Transmit data underrun active in one of channel 16~19. Write 1 to clear.
     * |[13]    |G5TXUND   |Channel 20~23 Transmit Data Underrun
     * |        |          |0 = No Transmit data underrun active in one of channel 20~23.
     * |        |          |1 = Transmit data underrun active in one of channel 20~23. Write 1 to clear.
     * |[16]    |G0TXDF    |Channel 0~3 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 0~3.
     * |        |          |1 = Transmit done in one of channel 0~3. Write 1 to clear.
     * |[17]    |G1TXDF    |Channel 4~7 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 4~7.
     * |        |          |1 = Transmit done in one of channel 4~7. Write 1 to clear.
     * |[18]    |G2TXDF    |Channel 8~11 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 8~11.
     * |        |          |1 = Transmit done in one of channel 8~11. Write 1 to clear.
     * |[19]    |G3TXDF    |Channel 12~15 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 12~15.
     * |        |          |1 = Transmit done in one of channel 12~15. Write 1 to clear.
     * |[20]    |G4TXDF    |Channel 16~19 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 16~19.
     * |        |          |1 = Transmit done in one of channel 16~19. Write 1 to clear.
     * |[21]    |G5TXDF    |Channel 20~23 Transmit Done
     * |        |          |0 = No Transmit done yet in one of channel 20~23.
     * |        |          |1 = Transmit done in one of channel 20~23. Write 1 to clear.
     * @var PDCI_T::CHEMPTY
     * Offset: 0x14  PDCI Channel Buffer Empty Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CH0EPT    |PDCI Channel 0 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[1]     |CH1EPT    |PDCI Channel 1 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[2]     |CH2EPT    |PDCI Channel 2 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[3]     |CH3EPT    |PDCI Channel 3 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[4]     |CH4EPT    |PDCI Channel 4 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[5]     |CH5EPT    |PDCI Channel 5 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[6]     |CH6EPT    |PDCI Channel 6 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[7]     |CH7EPT    |PDCI Channel 7 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[8]     |CH8EPT    |PDCI Channel 8 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[9]     |CH9EPT    |PDCI Channel 9 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[10]    |CH10EPT   |PDCI Channel 10 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[11]    |CH11EPT   |PDCI Channel 11 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[12]    |CH12EPT   |PDCI Channel 12 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[13]    |CH13EPT   |PDCI Channel 13 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[14]    |CH14EPT   |PDCI Channel 14 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[15]    |CH15EPT   |PDCI Channel 15 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[16]    |CH16EPT   |PDCI Channel 16 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[17]    |CH17EPT   |PDCI Channel 17 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[18]    |CH18EPT   |PDCI Channel 18 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[19]    |CH19EPT   |PDCI Channel 19 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[20]    |CH20EPT   |PDCI Channel 20 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[21]    |CH21EPT   |PDCI Channel 21 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[22]    |CH22EPT   |PDCI Channel 22 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * |[23]    |CH23EPT   |PDCI Channel 23 Current FIFO Empty
     * |        |          |0 = The current transmitted FIFO is not empty.
     * |        |          |1 = The current transmitted FIFO is empty.
     * |        |          |Note: This bit will be cleared automatically by writing the relative channel data (byte).
     * @var PDCI_T::TXDATG0
     * Offset: 0x18  PDCI Transmit Data Group 0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH0_TXDAT |PDCI Channel 0 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 0.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 0 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH1_TXDAT |PDCI Channel 1 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 1.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 1 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH2_TXDAT |PDCI Channel 2 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 2.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 2 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH3_TXDAT |PDCI Channel 3 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 3.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 3 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::TXDATG1
     * Offset: 0x1C  PDCI Transmit Data Group 1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH4_TXDAT |PDCI Channel 4 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 4.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 4 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH5_TXDAT |PDCI Channel 5 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 5.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 5 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH6_TXDAT |PDCI Channel 6 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 6.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 6 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH7_TXDAT |PDCI Channel 7 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 7.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 7 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::TXDATG2
     * Offset: 0x20  PDCI Transmit Data Group 2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH8_TXDAT |PDCI Channel 8 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 8.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 8 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH9_TXDAT |PDCI Channel 9 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 9.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 9 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH10_TXDAT|PDCI Channel 10 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 10.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 10 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH11_TXDAT|PDCI Channel 11 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 11.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 11 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::TXDATG3
     * Offset: 0x24  PDCI Transmit Data Group 3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH12_TXDAT|PDCI Channel 12 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 12.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 12 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH13_TXDAT|PDCI Channel 13 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 13.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 13 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH14_TXDAT|PDCI Channel 14 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 14.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 14 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH15_TXDAT|PDCI Channel 15 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 15.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 15 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::TXDATG4
     * Offset: 0x28  PDCI Transmit Data Group 4 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH16_TXDAT|PDCI Channel 16 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 16.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 16 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH17_TXDAT|PDCI Channel 17 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 17.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 17 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH18_TXDAT|PDCI Channel 18 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 18.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 18 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH19_TXDAT|PDCI Channel 19 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 19.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 19 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::TXDATG5
     * Offset: 0x2C  PDCI Transmit Data Group 5 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH20_TXDAT|PDCI Channel 20 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 20.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 20 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[15:8]  |CH21_TXDAT|PDCI Channel 21 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 21.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 21 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[23:16] |CH22_TXDAT|PDCI Channel 22 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 22.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 22 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * |[31:24] |CH23_TXDAT|PDCI Channel 23 Transmit Data
     * |        |          |The bits field indicates the transmit data buffer for channel 23.
     * |        |          |If MODES (PDCI_CTL2[17:16]) = 2'b00, it indicates the 5-bit BMC data.
     * |        |          |If MODES (PDCI_CTL2[17:16]) != 2'b00, it indicates two 4-bit nibble data for the PDCI channel 23 and its will be converted to 5B if TWOCHEN (PDCI_CTL2[12]) is set to 0.
     * @var PDCI_T::CTL2
     * Offset: 0x38  PDCI Control 2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PRENUM    |Preamble Number
     * |        |          |The field indicates the bit number of preamble of the packet.
     * |        |          |0000 = Indicates 64 bits of preamble.
     * |        |          |0001 = Indicates 4 bits of preamble.
     * |        |          |0010 = Indicates 5 bits of preamble.
     * |        |          |0011 = Indicates 8 bits of preamble.
     * |        |          |0100 = Indicates 10 bits of preamble.
     * |        |          |0101 = Indicates 16 bits of preamble.
     * |        |          |0110 = Indicates 20 bits of preamble.
     * |        |          |0111 = Indicates 32 bits of preamble.
     * |        |          |1000 = Indicates 40 bits of preamble.
     * |        |          |1001 = Indicates 48 bits of preamble.
     * |        |          |1010 = Indicates 60 bits of preamble.
     * |        |          |Others = Reserved.
     * |[7:5]   |TAILNUM   |Tail Bit Number of Packet
     * |        |          |The field indicates the bit number plus 1 of tail of packet after EOP and the ZERO3B (PDCI_CTL2[14]) shall be set to 0.
     * |        |          |Note: This is support in MODES = 2'b01 only and the minimum value is 1.
     * |[8]     |KCODES    |KCODE Source Bit
     * |        |          |0 = The content of K-Code in PDCI packet stored in SRAM.
     * |        |          |1 = The content of K-Code in PDCI packet stored in PDCI_KCODE register.
     * |        |          |Note: In MODES = 2'b01, the bit shall be set to 1. In MODES = 2'b00, the bit shall be set to 0.
     * |[9]     |CRCEN     |CRC function Enable Bit
     * |        |          |0 = CRC function is Disabled.
     * |        |          |1 = CRC function is Enabled.
     * |        |          |Note: If CRC is calculated by hardware, the data length must be limited to an even byte number.
     * |[11]    |B4B5EN    |B4 to B5 Transfer Enable Bit
     * |        |          |0 = Four bits transfer to five bits function is Disabled.
     * |        |          |1 = Four bits transfer to five bits function is Enabled.
     * |        |          |Note: This bit is only valid when TWOCHEN is set to 1 and in MODES = 2'b10.
     * |[12]    |TWOCHEN   |Two Channel Line Mode Enable Bit
     * |        |          |0 = Two channel lines mode is Disabled.
     * |        |          |1 = Two channel lines mode is Enabled.
     * |        |          |Note 1: When this bit is set to 1, there is no preamble period and the PDCI clock will output at the same time.
     * |        |          |Note 2: The dummy length is fixed to 2 when this bit is Enabled -> The dummy period is ((2+1) * 8 + 1) bit time.
     * |[13]    |MSB       |Most Significant Bit Enable
     * |        |          |0 = MSB first is Disabled.
     * |        |          |1 = MSB first is Enabled.
     * |        |          |Note: For MODES = 2'b00, this bit shall set as 1'b0 else this bit shall set as 1'b1.
     * |[14]    |ZERO3B    |Zero 3-Bit After EOP
     * |        |          |0 = Zero 3-Bit after EOP is Disabled.
     * |        |          |1 = Zero 3-Bit after EOP is Enabled.
     * |        |          |Note: If MODES no equal 2'b00, this bit shall be set to 0.
     * |[17:16] |MODES     |Mode Selection
     * |        |          |00 = 5-Bit data mode for entire packet (default).
     * |        |          |01 = 4 Bit to 5 Bit function is Enable and the SOP content comes from KCODEm (m = 1~4) in PDCI_KCODE register and it is work in DMAEN (PDCI_CTL[4]) = 1 only.
     * |        |          |10 = 4 Bit to 5 Bit function is Disable and 4 Bit data mode for entire packet.
     * |        |          |11 = There is not any preamble, SOP, CRC and EOP content. It is data only and transmit by DMA.
     * |        |          |Note 1: For MODES = 2'b00, the SOP and TX data shall be converted by the transfer table.
     * |        |          |Note 2: For MODES = 2'b01 or 2'b10, the SOP is latched for PDCI_KCODE register and the data is 4+4 bits in the TXDATA registers. The MSB (PDCI_CTL2[13]) shall be set to 1.
     * |        |          |Note 3: In MODE = 2'b10, the mapping of 4-bit to 5-bit BMC transfer is disabled unless the B4B5EN and TWOCHEN bits be set to 1.
     * |        |          |Note 4: This is also used in PDCI RX. But for only MODES = 2'b01 or 2'b10.
     * |[20]    |HDMODE    |Half Duplex Mode
     * |        |          |0 = Half Duplex Mode is Disabled.
     * |        |          |1 = Half Duplex Mode is Enabled.
     * @var PDCI_T::KCODE
     * Offset: 0x3C  PDCI K-Code Pattern Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |KCODE1    |Packet 1st K-Code Data
     * |        |          |The bit field indicates the 1st SOP pattern if the KCODES (PDCI_CTL2[8]) is set to 1.
     * |        |          |Note: In MODES = 2'b10, the four bits in LSB is used only when B4B5EN (PDCI_CTL2[11]) = 0.
     * |[12:8]  |KCODE2    |Packet 2nd K-Code Data
     * |        |          |The bit field indicates the 2nd SOP pattern if the KCODES (PDCI_CTL2[8]) is set to 1.
     * |        |          |Note: In MODES = 2'b10, the four bits in LSB is used only when B4B5EN (PDCI_CTL2[11]) = 0.
     * |[20:16] |KCODE3    |Packet 3rd K-Code Data
     * |        |          |The bit field indicates the 3rd SOP pattern if the KCODES (PDCI_CTL2[8]) is set to 1.
     * |        |          |Note: In MODES = 2'b10, the four bits in LSB is used only when B4B5EN (PDCI_CTL2[11]) = 0.
     * |[28:24] |KCODE4    |Packet 4th K-Code Data
     * |        |          |The bit field indicates the 4th SOP pattern if the KCODES (PDCI_CTL2[8]) is set to 1.
     * |        |          |Note: In MODES = 2'b10, the four bits in LSB is used only when B4B5EN (PDCI_CTL2[11]) = 0.
     * @var PDCI_T::CONFIGIO
     * Offset: 0x40  PDCI Configured I/O Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |G0TGION   |Group 0 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 0.
     * |        |          |000 = Indicates the destination of group 0 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 0 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 0 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 0 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 0 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 0 to I/O group 5.
     * |[6:4]   |G1TGION   |Group 1 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 1.
     * |        |          |000 = Indicates the destination of group 1 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 1 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 1 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 1 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 1 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 1 to I/O group 5.
     * |[10:8]  |G2TGION   |Group 2 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 2.
     * |        |          |000 = Indicates the destination of group 2 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 2 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 2 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 2 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 2 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 2 to I/O group 5.
     * |[14:12] |G3TGION   |Group 3 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 3.
     * |        |          |000 = Indicates the destination of group 3 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 3 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 3 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 3 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 3 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 3 to I/O group 5.
     * |[18:16] |G4TGION   |Group 4 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 4.
     * |        |          |000 = Indicates the destination of group 4 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 4 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 4 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 4 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 4 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 4 to I/O group 5.
     * |[22:20] |G5TGION   |Group 5 to Group I/O N
     * |        |          |The field indicates the destination I/O group of Group 5.
     * |        |          |000 = Indicates the destination of group 5 to I/O group 0.
     * |        |          |001 = Indicates the destination of group 5 to I/O group 1.
     * |        |          |010 = Indicates the destination of group 5 to I/O group 2.
     * |        |          |011 = Indicates the destination of group 5 to I/O group 3.
     * |        |          |100 = Indicates the destination of group 5 to I/O group 4.
     * |        |          |101 = Indicates the destination of group 5 to I/O group 5.
     * @var PDCI_T::RXCTL0
     * Offset: 0x48  PDCI Receiver Control 0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXPDMAEN  |RX PDMA Channel Enable Bit
     * |        |          |0 = Disable PDMA function.
     * |        |          |1 = Enable PDMA function.
     * |[1]     |RXCRCIEN  |PDCI Receiver CRC Error Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver CRC Error interrupt.
     * |        |          |1 = Enable PDCI Receiver CRC Error interrupt.
     * |[2]     |RXCRCAIEN |PDCI Receiver CRC Address Error Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver CRC Address Error interrupt.
     * |        |          |1 = Enable PDCI Receiver CRC Address Error interrupt.
     * |[4]     |RXFIEN    |PDCI Receiver Data FIFO Full Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver Data FIFO full interrupt.
     * |        |          |1 = Enable PDCI Receiver Data FIFO full interrupt.
     * |[5]     |RXDIEN    |PDCI Receiver Done Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver transfer done interrupt.
     * |        |          |1 = Enable PDCI Receiver transfer done interrupt.
     * |[6]     |RXOVIEN   |PDCI Receiver Overflow Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver Overflow Interrupt.
     * |        |          |1 = Enable PDCI Receiver Overflow Interrupt.
     * |[7]     |RXFAILIEN |PDCI Receiver Fail Interrupt Enable Bit
     * |        |          |0 = Disable PDCI Receiver transfer fail interrupt.
     * |        |          |1 = Enable PDCI Receiver transfer fail interrupt.
     * |[15:8]  |RXTHRES   |Receive Data Threshold
     * |        |          |Defines the bit time duty threshold (not 50/50). Edges with duty below this threshold will be ignored.
     * |        |          |Note: It is recommended to set this value to 15~20% of BTDIV minus 1 (i.e., RXTHRES = BTDIV * 20% - 1).
     * |[24]    |OSPBMODE  |OSPBMODE
     * |        |          |Set this bit when using the OSPB protocol. It enables the calculation of data length in LSB format for proper handling of OSPB data format.
     * @var PDCI_T::RXCTL1
     * Offset: 0x4C  PDCI Receiver Control 1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX0EN     |PDCI RX Channel 0 Enable Bit
     * |        |          |0 = PDCI RX Channel 0 Disabled.
     * |        |          |1 = PDCI RX Channel 0 Enabled.
     * |[1]     |RX1EN     |PDCI RX Channel 1 Enable Bit
     * |        |          |0 = PDCI RX Channel 1 Disabled.
     * |        |          |1 = PDCI RX Channel 1 Enabled.
     * |[2]     |RX2EN     |PDCI RX Channel 2 Enable Bit
     * |        |          |0 = PDCI RX Channel 2 Disabled.
     * |        |          |1 = PDCI RX Channel 2 Enabled.
     * |[3]     |RX3EN     |PDCI RX Channel 3 Enable Bit
     * |        |          |0 = PDCI RX Channel 3 Disabled.
     * |        |          |1 = PDCI RX Channel 3 Enabled.
     * |[4]     |RX4EN     |PDCI RX Channel 4 Enable Bit
     * |        |          |0 = PDCI RX Channel 4 Disabled.
     * |        |          |1 = PDCI RX Channel 4 Enabled.
     * |[5]     |RX5EN     |PDCI RX Channel 5 Enable Bit
     * |        |          |0 = PDCI RX Channel 5 Disabled.
     * |        |          |1 = PDCI RX Channel 5 Enabled.
     * |[6]     |RX6EN     |PDCI RX Channel 6 Enable Bit
     * |        |          |0 = PDCI RX Channel 6 Disabled.
     * |        |          |1 = PDCI RX Channel 6 Enabled.
     * |[7]     |RX7EN     |PDCI RX Channel 7 Enable Bit
     * |        |          |0 = PDCI RX Channel 7 Disabled.
     * |        |          |1 = PDCI RX Channel 7 Enabled.
     * |[8]     |RX8EN     |PDCI RX Channel 8 Enable Bit
     * |        |          |0 = PDCI RX Channel 8 Disabled.
     * |        |          |1 = PDCI RX Channel 8 Enabled.
     * |[9]     |RX9EN     |PDCI RX Channel 9 Enable Bit
     * |        |          |0 = PDCI RX Channel 9 Disabled.
     * |        |          |1 = PDCI RX Channel 9 Enabled.
     * |[10]    |RX10EN    |PDCI RX Channel 10 Enable Bit
     * |        |          |0 = PDCI RX Channel 10 Disabled.
     * |        |          |1 = PDCI RX Channel 10 Enabled.
     * |[11]    |RX11EN    |PDCI RX Channel 11 Enable Bit
     * |        |          |0 = PDCI RX Channel 11 Disabled.
     * |        |          |1 = PDCI RX Channel 11 Enabled.
     * |[12]    |RX12EN    |PDCI RX Channel 12 Enable Bit
     * |        |          |0 = PDCI RX Channel 12 Disabled.
     * |        |          |1 = PDCI RX Channel 12 Enabled.
     * |[13]    |RX13EN    |PDCI RX Channel 13 Enable Bit
     * |        |          |0 = PDCI RX Channel 13 Disabled.
     * |        |          |1 = PDCI RX Channel 13 Enabled.
     * |[14]    |RX14EN    |PDCI RX Channel 14 Enable Bit
     * |        |          |0 = PDCI RX Channel 14 Disabled.
     * |        |          |1 = PDCI RX Channel 14 Enabled.
     * |[15]    |RX15EN    |PDCI RX Channel 15 Enable Bit
     * |        |          |0 = PDCI RX Channel 15 Disabled.
     * |        |          |1 = PDCI RX Channel 15 Enabled.
     * |[16]    |RX16EN    |PDCI RX Channel 16 Enable Bit
     * |        |          |0 = PDCI RX Channel 16 Disabled.
     * |        |          |1 = PDCI RX Channel 16 Enabled.
     * |[17]    |RX17EN    |PDCI RX Channel 17 Enable Bit
     * |        |          |0 = PDCI RX Channel 17 Disabled.
     * |        |          |1 = PDCI RX Channel 17 Enabled.
     * |[18]    |RX18EN    |PDCI RX Channel 18 Enable Bit
     * |        |          |0 = PDCI RX Channel 18 Disabled.
     * |        |          |1 = PDCI RX Channel 18 Enabled.
     * |[19]    |RX19EN    |PDCI RX Channel 19 Enable Bit
     * |        |          |0 = PDCI RX Channel 19 Disabled.
     * |        |          |1 = PDCI RX Channel 19 Enabled.
     * |[20]    |RX20EN    |PDCI RX Channel 20 Enable Bit
     * |        |          |0 = PDCI RX Channel 20 Disabled.
     * |        |          |1 = PDCI RX Channel 20 Enabled.
     * |[21]    |RX21EN    |PDCI RX Channel 21 Enable Bit
     * |        |          |0 = PDCI RX Channel 21 Disabled.
     * |        |          |1 = PDCI RX Channel 21 Enabled.
     * |[22]    |RX22EN    |PDCI RX Channel 22 Enable Bit
     * |        |          |0 = PDCI RX Channel 22 Disabled.
     * |        |          |1 = PDCI RX Channel 22 Enabled.
     * |[23]    |RX23EN    |PDCI RX Channel 23 Enable Bit
     * |        |          |0 = PDCI RX Channel 23 Disabled.
     * |        |          |1 = PDCI RX Channel 23 Enabled.
     * @var PDCI_T::RXCNT
     * Offset: 0x50  PDCI Receiver Data Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |RXADDRC   |Receive Address Count
     * |        |          |Indicates the number of subaddress to be received. The maximum RXADDRC value is 8.
     * @var PDCI_T::RXG01STS
     * Offset: 0x54  PDCI Receiver Group 0/1 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXDAT0F   |PDCI Receiver Data FIFO 0 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 0 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[1]     |RXDAT1F   |PDCI Receiver Data FIFO 1 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 1 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[2]     |RXDAT2F   |PDCI Receiver Data FIFO 2 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 2 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[3]     |RXDAT3F   |PDCI Receiver Data FIFO 3 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 3 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[4]     |RXDAT4F   |PDCI Receiver Data FIFO 4 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 4 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[5]     |RXDAT5F   |PDCI Receiver Data FIFO 5 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 5 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[6]     |RXDAT6F   |PDCI Receiver Data FIFO 6 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 6 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[7]     |RXDAT7F   |PDCI Receiver Data FIFO 7 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 7 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[8]     |RX0DONE   |PDCI Receiver 0 Done
     * |        |          |Indicates that the PDCI receiver 0 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[9]     |RX1DONE   |PDCI Receiver 1 Done
     * |        |          |Indicates that the PDCI receiver 1 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[10]    |RX2DONE   |PDCI Receiver 2 Done
     * |        |          |Indicates that the PDCI receiver 2 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[11]    |RX3DONE   |PDCI Receiver 3 Done
     * |        |          |Indicates that the PDCI receiver 3 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[12]    |RX4DONE   |PDCI Receiver 4 Done
     * |        |          |Indicates that the PDCI receiver 4 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[13]    |RX5DONE   |PDCI Receiver 5 Done
     * |        |          |Indicates that the PDCI receiver 5 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[14]    |RX6DONE   |PDCI Receiver 6 Done
     * |        |          |Indicates that the PDCI receiver 6 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[15]    |RX7DONE   |PDCI Receiver 7 Done
     * |        |          |Indicates that the PDCI receiver 7 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[16]    |RX0OV     |PDCI Receiver 0 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 0.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[17]    |RX1OV     |PDCI Receiver 1 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 1.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[18]    |RX2OV     |PDCI Receiver 2 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 2.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[19]    |RX3OV     |PDCI Receiver 3 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 3.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[20]    |RX4OV     |PDCI Receiver 4 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 4.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[21]    |RX5OV     |PDCI Receiver 5 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 5.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[22]    |RX6OV     |PDCI Receiver 6 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 6.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[23]    |RX7OV     |PDCI Receiver 7 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 7.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[24]    |RX0FAIL   |PDCI Receiver 0 Fail
     * |        |          |Indicates a failure in PDCI receiver 0.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[25]    |RX1FAIL   |PDCI Receiver 1 Fail
     * |        |          |Indicates a failure in PDCI receiver 1.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[26]    |RX2FAIL   |PDCI Receiver 2 Fail
     * |        |          |Indicates a failure in PDCI receiver 2.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[27]    |RX3FAIL   |PDCI Receiver 3 Fail
     * |        |          |Indicates a failure in PDCI receiver 3.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[28]    |RX4FAIL   |PDCI Receiver 4 Fail
     * |        |          |Indicates a failure in PDCI receiver 4.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[29]    |RX5FAIL   |PDCI Receiver 5 Fail
     * |        |          |Indicates a failure in PDCI receiver 5.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[30]    |RX6FAIL   |PDCI Receiver 6 Fail
     * |        |          |Indicates a failure in PDCI receiver 6.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[31]    |RX7FAIL   |PDCI Receiver 7 Fail
     * |        |          |Indicates a failure in PDCI receiver 7.
     * |        |          |This bit can be cleared by writing 1 to it.
     * @var PDCI_T::RXG23STS
     * Offset: 0x58  PDCI Receiver Group 2/3 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXDAT8F   |PDCI Receiver Data FIFO 8 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 8 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[1]     |RXDAT9F   |PDCI Receiver Data FIFO 9 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 9 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[2]     |RXDAT10F  |PDCI Receiver Data FIFO 10 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 10 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[3]     |RXDAT11F  |PDCI Receiver Data FIFO 11 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 11 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[4]     |RXDAT12F  |PDCI Receiver Data FIFO 12 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 12 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[5]     |RXDAT13F  |PDCI Receiver Data FIFO 13 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 13 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[6]     |RXDAT14F  |PDCI Receiver Data FIFO 14 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 14 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[7]     |RXDAT15F  |PDCI Receiver Data FIFO 15 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 15 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[8]     |RX8DONE   |PDCI Receiver 8 Done
     * |        |          |Indicates that the PDCI receiver 8 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[9]     |RX9DONE   |PDCI Receiver 9 Done
     * |        |          |Indicates that the PDCI receiver 9 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[10]    |RX10DONE  |PDCI Receiver 10 Done
     * |        |          |Indicates that the PDCI receiver 10 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[11]    |RX11DONE  |PDCI Receiver 11 Done
     * |        |          |Indicates that the PDCI receiver 11 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[12]    |RX12DONE  |PDCI Receiver 12 Done
     * |        |          |Indicates that the PDCI receiver 12 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[13]    |RX13DONE  |PDCI Receiver 13 Done
     * |        |          |Indicates that the PDCI receiver 13 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[14]    |RX14DONE  |PDCI Receiver 14 Done
     * |        |          |Indicates that the PDCI receiver 14 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[15]    |RX15DONE  |PDCI Receiver 15 Done
     * |        |          |Indicates that the PDCI receiver 15 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[16]    |RX8OV     |PDCI Receiver 8 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 8.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[17]    |RX9OV     |PDCI Receiver 9 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 9.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[18]    |RX10OV    |PDCI Receiver 10 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 10.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[19]    |RX11OV    |PDCI Receiver 11 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 11.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[20]    |RX12OV    |PDCI Receiver 12 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 12.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[21]    |RX13OV    |PDCI Receiver 13 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 13.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[22]    |RX14OV    |PDCI Receiver 14 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 14.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[23]    |RX15OV    |PDCI Receiver 15 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 15.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[24]    |RX8FAIL   |PDCI Receiver 8 Fail
     * |        |          |Indicates a failure in PDCI receiver 8.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[25]    |RX9FAIL   |PDCI Receiver 9 Fail
     * |        |          |Indicates a failure in PDCI receiver 9.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[26]    |RX10FAIL  |PDCI Receiver 10 Fail
     * |        |          |Indicates a failure in PDCI receiver 10.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[27]    |RX11FAIL  |PDCI Receiver 11 Fail
     * |        |          |Indicates a failure in PDCI receiver 11.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[28]    |RX12FAIL  |PDCI Receiver 12 Fail
     * |        |          |Indicates a failure in PDCI receiver 12.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[29]    |RX13FAIL  |PDCI Receiver 13 Fail
     * |        |          |Indicates a failure in PDCI receiver 13.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[30]    |RX14FAIL  |PDCI Receiver 14 Fail
     * |        |          |Indicates a failure in PDCI receiver 14.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[31]    |RX15FAIL  |PDCI Receiver 15 Fail
     * |        |          |Indicates a failure in PDCI receiver 15.
     * |        |          |This bit can be cleared by writing 1 to it.
     * @var PDCI_T::RXG45STS
     * Offset: 0x5C  PDCI Receiver Group 4/5 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXDAT16F  |PDCI Receiver Data FIFO 16 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 16 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[1]     |RXDAT17F  |PDCI Receiver Data FIFO 17 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 17 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[2]     |RXDAT18F  |PDCI Receiver Data FIFO 18 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 18 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[3]     |RXDAT19F  |PDCI Receiver Data FIFO 19 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 19 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[4]     |RXDAT20F  |PDCI Receiver Data FIFO 20 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 20 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[5]     |RXDAT21F  |PDCI Receiver Data FIFO 21 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 21 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[6]     |RXDAT22F  |PDCI Receiver Data FIFO 22 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 22 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[7]     |RXDAT23F  |PDCI Receiver Data FIFO 23 Full
     * |        |          |Indicates that the PDCI receiver data FIFO 23 is full.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[8]     |RX16DONE  |PDCI Receiver 16 Done
     * |        |          |Indicates that the PDCI receiver 16 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[9]     |RX17DONE  |PDCI Receiver 17 Done
     * |        |          |Indicates that the PDCI receiver 17 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[10]    |RX18DONE  |PDCI Receiver 18 Done
     * |        |          |Indicates that the PDCI receiver 18 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[11]    |RX19DONE  |PDCI Receiver 19 Done
     * |        |          |Indicates that the PDCI receiver 19 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[12]    |RX20DONE  |PDCI Receiver 20 Done
     * |        |          |Indicates that the PDCI receiver 20 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[13]    |RX21DONE  |PDCI Receiver 21 Done
     * |        |          |Indicates that the PDCI receiver 21 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[14]    |RX22DONE  |PDCI Receiver 22 Done
     * |        |          |Indicates that the PDCI receiver 22 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[15]    |RX23DONE  |PDCI Receiver 23 Done
     * |        |          |Indicates that the PDCI receiver 23 transfer is complete.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[16]    |RX16OV    |PDCI Receiver 16 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 16.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[17]    |RX17OV    |PDCI Receiver 17 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 17.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[18]    |RX18OV    |PDCI Receiver 18 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 18.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[19]    |RX19OV    |PDCI Receiver 19 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 19.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[20]    |RX20OV    |PDCI Receiver 20 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 20.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[21]    |RX21OV    |PDCI Receiver 21 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 21.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[22]    |RX22OV    |PDCI Receiver 22 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 22.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[23]    |RX23OV    |PDCI Receiver 23 FIFO Overflow
     * |        |          |Indicates that a FIFO overflow has occurred in the PDCI receiver 23.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[24]    |RX16FAIL  |PDCI Receiver 16 Fail
     * |        |          |Indicates a failure in PDCI receiver 16.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[25]    |RX17FAIL  |PDCI Receiver 17 Fail
     * |        |          |Indicates a failure in PDCI receiver 17.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[26]    |RX18FAIL  |PDCI Receiver 18 Fail
     * |        |          |Indicates a failure in PDCI receiver 18.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[27]    |RX19FAIL  |PDCI Receiver 19 Fail
     * |        |          |Indicates a failure in PDCI receiver 19.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[28]    |RX20FAIL  |PDCI Receiver 20 Fail
     * |        |          |Indicates a failure in PDCI receiver 20.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[29]    |RX21FAIL  |PDCI Receiver 21 Fail
     * |        |          |Indicates a failure in PDCI receiver 21.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[30]    |RX22FAIL  |PDCI Receiver 22 Fail
     * |        |          |Indicates a failure in PDCI receiver 22.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[31]    |RX23FAIL  |PDCI Receiver 23 Fail
     * |        |          |Indicates a failure in PDCI receiver 23.
     * |        |          |This bit can be cleared by writing 1 to it.
     * @var PDCI_T::RXDATG0
     * Offset: 0x60  PDCI Receiver Data Group 0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH0_RXDAT |PDCI Channel 0 Receive Data
     * |        |          |This field contains the received data for channel 0.
     * |[15:8]  |CH1_RXDAT |PDCI Channel 1 Receive Data
     * |        |          |This field contains the received data for channel 1.
     * |[23:16] |CH2_RXDAT |PDCI Channel 2 Receive Data
     * |        |          |This field contains the received data for channel 2.
     * |[31:24] |CH3_RXDAT |PDCI Channel 3 Receive Data
     * |        |          |This field contains the received data for channel 3.
     * @var PDCI_T::RXDATG1
     * Offset: 0x64  PDCI Receiver Data Group 1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH4_RXDAT |PDCI Channel 4 Receive Data
     * |        |          |This field contains the received data for channel 4.
     * |[15:8]  |CH5_RXDAT |PDCI Channel 5 Receive Data
     * |        |          |This field contains the received data for channel 5.
     * |[23:16] |CH6_RXDAT |PDCI Channel 6 Receive Data
     * |        |          |This field contains the received data for channel 6.
     * |[31:24] |CH7_RXDAT |PDCI Channel 7 Receive Data
     * |        |          |This field contains the received data for channel 7.
     * @var PDCI_T::RXDATG2
     * Offset: 0x68  PDCI Receiver Data Group 2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH8_RXDAT |PDCI Channel 8 Receive Data
     * |        |          |This field contains the received data for channel 8.
     * |[15:8]  |CH9_RXDAT |PDCI Channel 9 Receive Data
     * |        |          |This field contains the received data for channel 9.
     * |[23:16] |CH10_RXDAT|PDCI Channel 10 Receive Data
     * |        |          |This field contains the received data for channel 10.
     * |[31:24] |CH11_RXDAT|PDCI Channel 11 Receive Data
     * |        |          |This field contains the received data for channel 11.
     * @var PDCI_T::RXDATG3
     * Offset: 0x6C  PDCI Receiver Data Group 3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH12_RXDAT|PDCI Channel 12 Receive Data
     * |        |          |This field contains the received data for channel 12.
     * |[15:8]  |CH13_RXDAT|PDCI Channel 13 Receive Data
     * |        |          |This field contains the received data for channel 13.
     * |[23:16] |CH14_RXDAT|PDCI Channel 14 Receive Data
     * |        |          |This field contains the received data for channel 14.
     * |[31:24] |CH15_RXDAT|PDCI Channel 15 Receive Data
     * |        |          |This field contains the received data for channel 15.
     * @var PDCI_T::RXDATG4
     * Offset: 0x70  PDCI Receiver Data Group 4 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH16_RXDAT|PDCI Channel 16 Receive Data
     * |        |          |This field contains the received data for channel 16.
     * |[15:8]  |CH17_RXDAT|PDCI Channel 17 Receive Data
     * |        |          |This field contains the received data for channel 17.
     * |[23:16] |CH18_RXDAT|PDCI Channel 18 Receive Data
     * |        |          |This field contains the received data for channel 18.
     * |[31:24] |CH19_RXDAT|PDCI Channel 19 Receive Data
     * |        |          |This field contains the received data for channel 19.
     * @var PDCI_T::RXDATG5
     * Offset: 0x74  PDCI Receiver Data Group 5 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CH20_RXDAT|PDCI Channel 20 Receive Data
     * |        |          |This field contains the received data for channel 20.
     * |[15:8]  |CH21_RXDAT|PDCI Channel 21 Receive Data
     * |        |          |This field contains the received data for channel 21.
     * |[23:16] |CH22_RXDAT|PDCI Channel 22 Receive Data
     * |        |          |This field contains the received data for channel 22.
     * |[31:24] |CH23_RXDAT|PDCI Channel 23 Receive Data
     * |        |          |This field contains the received data for channel 23.
     * @var PDCI_T::RXCRCSTS0
     * Offset: 0x78  PDCI Receiver CRC Status 0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX0CRCE   |PDCI Receiver 0 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[1]     |RX1CRCE   |PDCI Receiver 1 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[2]     |RX2CRCE   |PDCI Receiver 2 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[3]     |RX3CRCE   |PDCI Receiver 3 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[4]     |RX4CRCE   |PDCI Receiver 4 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[5]     |RX5CRCE   |PDCI Receiver 5 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[6]     |RX6CRCE   |PDCI Receiver 6 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[7]     |RX7CRCE   |PDCI Receiver 7 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[8]     |RX8CRCE   |PDCI Receiver 8 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[9]     |RX9CRCE   |PDCI Receiver 9 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[10]    |RX10CRCE  |PDCI Receiver 10 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[11]    |RX11CRCE  |PDCI Receiver 11 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[12]    |RX12CRCE  |PDCI Receiver 12 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[13]    |RX13CRCE  |PDCI Receiver 13 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[14]    |RX14CRCE  |PDCI Receiver 14 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[15]    |RX15CRCE  |PDCI Receiver 15 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[16]    |RX0CRCAE  |PDCI Receiver 0 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[17]    |RX1CRCAE  |PDCI Receiver 1 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[18]    |RX2CRCAE  |PDCI Receiver 2 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[19]    |RX3CRCAE  |PDCI Receiver 3 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[20]    |RX4CRCAE  |PDCI Receiver 4 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[21]    |RX5CRCAE  |PDCI Receiver 5 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[22]    |RX6CRCAE  |PDCI Receiver 6 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[23]    |RX7CRCAE  |PDCI Receiver 7 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[24]    |RX8CRCAE  |PDCI Receiver 8 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[25]    |RX9CRCAE  |PDCI Receiver 9 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[26]    |RX10CRCAE |PDCI Receiver 10 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[27]    |RX11CRCAE |PDCI Receiver 11 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[28]    |RX12CRCAE |PDCI Receiver 12 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[29]    |RX13CRCAE |PDCI Receiver 13 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[30]    |RX14CRCAE |PDCI Receiver 14 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[31]    |RX15CRCAE |PDCI Receiver 15 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * @var PDCI_T::RXCRCSTS1
     * Offset: 0x7C  PDCI Receiver CRC Status 1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX16CRCE  |PDCI Receiver 16 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[1]     |RX17CRCE  |PDCI Receiver 17 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[2]     |RX18CRCE  |PDCI Receiver 18 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[3]     |RX19CRCE  |PDCI Receiver 19 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[4]     |RX20CRCE  |PDCI Receiver 20 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[5]     |RX21CRCE  |PDCI Receiver 21 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[6]     |RX22CRCE  |PDCI Receiver 22 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[7]     |RX23CRCE  |PDCI Receiver 23 CRC Error
     * |        |          |Indicates that the received CRC code does not match the expected RX CRC output.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[16]    |RX16CRCAE |PDCI Receiver 16 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[17]    |RX17CRCAE |PDCI Receiver 17 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[18]    |RX18CRCAE |PDCI Receiver 18 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[19]    |RX19CRCAE |PDCI Receiver 19 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[20]    |RX20CRCAE |PDCI Receiver 20 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[21]    |RX21CRCAE |PDCI Receiver 21 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[22]    |RX22CRCAE |PDCI Receiver 22 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     * |[23]    |RX23CRCAE |PDCI Receiver 23 CRC Address Error
     * |        |          |Indicates a CRC error on address bit 15 in the PDCI receiver.
     * |        |          |This bit can be cleared by writing 1 to it.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] PDCI Control Register                                            */
    __IO uint32_t DNUM0;                 /*!< [0x0004] PDCI Dummy Number Group 0~3 Register                             */
    __IO uint32_t DNUM1;                 /*!< [0x0008] PDCI Dummy Number Group 4~7 Register                             */
    __IO uint32_t INTEN;                 /*!< [0x000c] PDCI Interrupt Enable Register                                   */
    __IO uint32_t INTSTS;                /*!< [0x0010] PDCI Interrupt Status Register                                   */
    __IO uint32_t CHEMPTY;               /*!< [0x0014] PDCI Channel Buffer Empty Register                               */
    __IO uint32_t TXDATG0;               /*!< [0x0018] PDCI Transmit Data Group 0 Register                              */
    __IO uint32_t TXDATG1;               /*!< [0x001c] PDCI Transmit Data Group 1 Register                              */
    __IO uint32_t TXDATG2;               /*!< [0x0020] PDCI Transmit Data Group 2 Register                              */
    __IO uint32_t TXDATG3;               /*!< [0x0024] PDCI Transmit Data Group 3 Register                              */
    __IO uint32_t TXDATG4;               /*!< [0x0028] PDCI Transmit Data Group 4 Register                              */
    __IO uint32_t TXDATG5;               /*!< [0x002c] PDCI Transmit Data Group 5 Register                              */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CTL2;                  /*!< [0x0038] PDCI Control 2 Register                                          */
    __IO uint32_t KCODE;                 /*!< [0x003c] PDCI K-Code Pattern Register                                     */
    __IO uint32_t CONFIGIO;              /*!< [0x0040] PDCI Configured I/O Register                                     */
    __I  uint32_t CRC16;                 /*!< [0x0044] PDCI CRC16 Result Register                                       */
    __IO uint32_t RXCTL0;                /*!< [0x0048] PDCI Receiver Control 0 Register                                 */
    __IO uint32_t RXCTL1;                /*!< [0x004c] PDCI Receiver Control 1 Register                                 */
    __IO uint32_t RXCNT;                 /*!< [0x0050] PDCI Receiver Data Count Register                                */
    __IO uint32_t RXG01STS;              /*!< [0x0054] PDCI Receiver Group 0/1 Status Register                          */
    __IO uint32_t RXG23STS;              /*!< [0x0058] PDCI Receiver Group 2/3 Status Register                          */
    __IO uint32_t RXG45STS;              /*!< [0x005c] PDCI Receiver Group 4/5 Status Register                          */
    __I  uint32_t RXDATG0;               /*!< [0x0060] PDCI Receiver Data Group 0 Register                              */
    __I  uint32_t RXDATG1;               /*!< [0x0064] PDCI Receiver Data Group 1 Register                              */
    __I  uint32_t RXDATG2;               /*!< [0x0068] PDCI Receiver Data Group 2 Register                              */
    __I  uint32_t RXDATG3;               /*!< [0x006c] PDCI Receiver Data Group 3 Register                              */
    __I  uint32_t RXDATG4;               /*!< [0x0070] PDCI Receiver Data Group 4 Register                              */
    __I  uint32_t RXDATG5;               /*!< [0x0074] PDCI Receiver Data Group 5 Register                              */
    __IO uint32_t RXCRCSTS0;             /*!< [0x0078] PDCI Receiver CRC Status 0 Register                              */
    __IO uint32_t RXCRCSTS1;             /*!< [0x007c] PDCI Receiver CRC Status 1 Register                              */

} PDCI_T;

/**
    @addtogroup PDCI_CONST PDCI Bit Field Definition
    Constant Definitions for PDCI Controller
@{ */

#define PDCI_CTL_PDCIEN_Pos              (0)                                               /*!< PDCI_T::CTL: PDCIEN Position           */
#define PDCI_CTL_PDCIEN_Msk              (0x1ul << PDCI_CTL_PDCIEN_Pos)                    /*!< PDCI_T::CTL: PDCIEN Mask               */

#define PDCI_CTL_BWADJ_Pos               (1)                                               /*!< PDCI_T::CTL: BWADJ Position            */
#define PDCI_CTL_BWADJ_Msk               (0x1ul << PDCI_CTL_BWADJ_Pos)                     /*!< PDCI_T::CTL: BWADJ Mask                */

#define PDCI_CTL_PREAM32_Pos             (2)                                               /*!< PDCI_T::CTL: PREAM32 Position          */
#define PDCI_CTL_PREAM32_Msk             (0x1ul << PDCI_CTL_PREAM32_Pos)                   /*!< PDCI_T::CTL: PREAM32 Mask              */

#define PDCI_CTL_DUMLVL_Pos              (3)                                               /*!< PDCI_T::CTL: DUMLVL Position           */
#define PDCI_CTL_DUMLVL_Msk              (0x1ul << PDCI_CTL_DUMLVL_Pos)                    /*!< PDCI_T::CTL: DUMLVL Mask               */

#define PDCI_CTL_DMAEN_Pos               (4)                                               /*!< PDCI_T::CTL: DMAEN Position            */
#define PDCI_CTL_DMAEN_Msk               (0x1ul << PDCI_CTL_DMAEN_Pos)                     /*!< PDCI_T::CTL: DMAEN Mask                */

#define PDCI_CTL_TAILVL_Pos              (7)                                               /*!< PDCI_T::CTL: TAILVL Position           */
#define PDCI_CTL_TAILVL_Msk              (0x1ul << PDCI_CTL_TAILVL_Pos)                    /*!< PDCI_T::CTL: TAILVL Mask               */

#define PDCI_CTL_G0CHEN_Pos              (8)                                               /*!< PDCI_T::CTL: G0CHEN Position           */
#define PDCI_CTL_G0CHEN_Msk              (0x1ul << PDCI_CTL_G0CHEN_Pos)                    /*!< PDCI_T::CTL: G0CHEN Mask               */

#define PDCI_CTL_G1CHEN_Pos              (9)                                               /*!< PDCI_T::CTL: G1CHEN Position           */
#define PDCI_CTL_G1CHEN_Msk              (0x1ul << PDCI_CTL_G1CHEN_Pos)                    /*!< PDCI_T::CTL: G1CHEN Mask               */

#define PDCI_CTL_G2CHEN_Pos              (10)                                              /*!< PDCI_T::CTL: G2CHEN Position           */
#define PDCI_CTL_G2CHEN_Msk              (0x1ul << PDCI_CTL_G2CHEN_Pos)                    /*!< PDCI_T::CTL: G2CHEN Mask               */

#define PDCI_CTL_G3CHEN_Pos              (11)                                              /*!< PDCI_T::CTL: G3CHEN Position           */
#define PDCI_CTL_G3CHEN_Msk              (0x1ul << PDCI_CTL_G3CHEN_Pos)                    /*!< PDCI_T::CTL: G3CHEN Mask               */

#define PDCI_CTL_G4CHEN_Pos              (12)                                              /*!< PDCI_T::CTL: G4CHEN Position           */
#define PDCI_CTL_G4CHEN_Msk              (0x1ul << PDCI_CTL_G4CHEN_Pos)                    /*!< PDCI_T::CTL: G4CHEN Mask               */

#define PDCI_CTL_G5CHEN_Pos              (13)                                              /*!< PDCI_T::CTL: G5CHEN Position           */
#define PDCI_CTL_G5CHEN_Msk              (0x1ul << PDCI_CTL_G5CHEN_Pos)                    /*!< PDCI_T::CTL: G5CHEN Mask               */

#define PDCI_CTL_BTDIV_Pos               (16)                                              /*!< PDCI_T::CTL: BTDIV Position            */
#define PDCI_CTL_BTDIV_Msk               (0x1fful << PDCI_CTL_BTDIV_Pos)                   /*!< PDCI_T::CTL: BTDIV Mask                */

#define PDCI_CTL_NBTDIV_Pos              (25)                                              /*!< PDCI_T::CTL: NBTDIV Position           */
#define PDCI_CTL_NBTDIV_Msk              (0x7ful << PDCI_CTL_NBTDIV_Pos)                   /*!< PDCI_T::CTL: NBTDIV Mask               */

#define PDCI_DNUM0_DNUMG0_Pos            (0)                                               /*!< PDCI_T::DNUM0: DNUMG0 Position         */
#define PDCI_DNUM0_DNUMG0_Msk            (0xfful << PDCI_DNUM0_DNUMG0_Pos)                 /*!< PDCI_T::DNUM0: DNUMG0 Mask             */

#define PDCI_DNUM0_DNUMG1_Pos            (8)                                               /*!< PDCI_T::DNUM0: DNUMG1 Position         */
#define PDCI_DNUM0_DNUMG1_Msk            (0xfful << PDCI_DNUM0_DNUMG1_Pos)                 /*!< PDCI_T::DNUM0: DNUMG1 Mask             */

#define PDCI_DNUM0_DNUMG2_Pos            (16)                                              /*!< PDCI_T::DNUM0: DNUMG2 Position         */
#define PDCI_DNUM0_DNUMG2_Msk            (0xfful << PDCI_DNUM0_DNUMG2_Pos)                 /*!< PDCI_T::DNUM0: DNUMG2 Mask             */

#define PDCI_DNUM0_DNUMG3_Pos            (24)                                              /*!< PDCI_T::DNUM0: DNUMG3 Position         */
#define PDCI_DNUM0_DNUMG3_Msk            (0xfful << PDCI_DNUM0_DNUMG3_Pos)                 /*!< PDCI_T::DNUM0: DNUMG3 Mask             */

#define PDCI_DNUM1_DNUMG4_Pos            (0)                                               /*!< PDCI_T::DNUM1: DNUMG4 Position         */
#define PDCI_DNUM1_DNUMG4_Msk            (0xfful << PDCI_DNUM1_DNUMG4_Pos)                 /*!< PDCI_T::DNUM1: DNUMG4 Mask             */

#define PDCI_DNUM1_DNUMG5_Pos            (8)                                               /*!< PDCI_T::DNUM1: DNUMG5 Position         */
#define PDCI_DNUM1_DNUMG5_Msk            (0xfful << PDCI_DNUM1_DNUMG5_Pos)                 /*!< PDCI_T::DNUM1: DNUMG5 Mask             */

#define PDCI_INTEN_FTXDIEN_Pos           (0)                                               /*!< PDCI_T::INTEN: FTXDIEN Position        */
#define PDCI_INTEN_FTXDIEN_Msk           (0x1ul << PDCI_INTEN_FTXDIEN_Pos)                 /*!< PDCI_T::INTEN: FTXDIEN Mask            */

#define PDCI_INTEN_TXUNDIEN_Pos          (1)                                               /*!< PDCI_T::INTEN: TXUNDIEN Position       */
#define PDCI_INTEN_TXUNDIEN_Msk          (0x1ul << PDCI_INTEN_TXUNDIEN_Pos)                /*!< PDCI_T::INTEN: TXUNDIEN Mask           */

#define PDCI_INTEN_FEMPTIEN_Pos          (2)                                               /*!< PDCI_T::INTEN: FEMPTIEN Position       */
#define PDCI_INTEN_FEMPTIEN_Msk          (0x1ul << PDCI_INTEN_FEMPTIEN_Pos)                /*!< PDCI_T::INTEN: FEMPTIEN Mask           */

#define PDCI_INTSTS_FTXDIF_Pos           (0)                                               /*!< PDCI_T::INTSTS: FTXDIF Position        */
#define PDCI_INTSTS_FTXDIF_Msk           (0x1ul << PDCI_INTSTS_FTXDIF_Pos)                 /*!< PDCI_T::INTSTS: FTXDIF Mask            */

#define PDCI_INTSTS_TXUNDIF_Pos          (1)                                               /*!< PDCI_T::INTSTS: TXUNDIF Position       */
#define PDCI_INTSTS_TXUNDIF_Msk          (0x1ul << PDCI_INTSTS_TXUNDIF_Pos)                /*!< PDCI_T::INTSTS: TXUNDIF Mask           */

#define PDCI_INTSTS_FEMPTIF_Pos          (2)                                               /*!< PDCI_T::INTSTS: FEMPTIF Position       */
#define PDCI_INTSTS_FEMPTIF_Msk          (0x1ul << PDCI_INTSTS_FEMPTIF_Pos)                /*!< PDCI_T::INTSTS: FEMPTIF Mask           */

#define PDCI_INTSTS_G0TXUND_Pos          (8)                                               /*!< PDCI_T::INTSTS: G0TXUND Position       */
#define PDCI_INTSTS_G0TXUND_Msk          (0x1ul << PDCI_INTSTS_G0TXUND_Pos)                /*!< PDCI_T::INTSTS: G0TXUND Mask           */

#define PDCI_INTSTS_G1TXUND_Pos          (9)                                               /*!< PDCI_T::INTSTS: G1TXUND Position       */
#define PDCI_INTSTS_G1TXUND_Msk          (0x1ul << PDCI_INTSTS_G1TXUND_Pos)                /*!< PDCI_T::INTSTS: G1TXUND Mask           */

#define PDCI_INTSTS_G2TXUND_Pos          (10)                                              /*!< PDCI_T::INTSTS: G2TXUND Position       */
#define PDCI_INTSTS_G2TXUND_Msk          (0x1ul << PDCI_INTSTS_G2TXUND_Pos)                /*!< PDCI_T::INTSTS: G2TXUND Mask           */

#define PDCI_INTSTS_G3TXUND_Pos          (11)                                              /*!< PDCI_T::INTSTS: G3TXUND Position       */
#define PDCI_INTSTS_G3TXUND_Msk          (0x1ul << PDCI_INTSTS_G3TXUND_Pos)                /*!< PDCI_T::INTSTS: G3TXUND Mask           */

#define PDCI_INTSTS_G4TXUND_Pos          (12)                                              /*!< PDCI_T::INTSTS: G4TXUND Position       */
#define PDCI_INTSTS_G4TXUND_Msk          (0x1ul << PDCI_INTSTS_G4TXUND_Pos)                /*!< PDCI_T::INTSTS: G4TXUND Mask           */

#define PDCI_INTSTS_G5TXUND_Pos          (13)                                              /*!< PDCI_T::INTSTS: G5TXUND Position       */
#define PDCI_INTSTS_G5TXUND_Msk          (0x1ul << PDCI_INTSTS_G5TXUND_Pos)                /*!< PDCI_T::INTSTS: G5TXUND Mask           */

#define PDCI_INTSTS_G0TXDF_Pos           (16)                                              /*!< PDCI_T::INTSTS: G0TXDF Position        */
#define PDCI_INTSTS_G0TXDF_Msk           (0x1ul << PDCI_INTSTS_G0TXDF_Pos)                 /*!< PDCI_T::INTSTS: G0TXDF Mask            */

#define PDCI_INTSTS_G1TXDF_Pos           (17)                                              /*!< PDCI_T::INTSTS: G1TXDF Position        */
#define PDCI_INTSTS_G1TXDF_Msk           (0x1ul << PDCI_INTSTS_G1TXDF_Pos)                 /*!< PDCI_T::INTSTS: G1TXDF Mask            */

#define PDCI_INTSTS_G2TXDF_Pos           (18)                                              /*!< PDCI_T::INTSTS: G2TXDF Position        */
#define PDCI_INTSTS_G2TXDF_Msk           (0x1ul << PDCI_INTSTS_G2TXDF_Pos)                 /*!< PDCI_T::INTSTS: G2TXDF Mask            */

#define PDCI_INTSTS_G3TXDF_Pos           (19)                                              /*!< PDCI_T::INTSTS: G3TXDF Position        */
#define PDCI_INTSTS_G3TXDF_Msk           (0x1ul << PDCI_INTSTS_G3TXDF_Pos)                 /*!< PDCI_T::INTSTS: G3TXDF Mask            */

#define PDCI_INTSTS_G4TXDF_Pos           (20)                                              /*!< PDCI_T::INTSTS: G4TXDF Position        */
#define PDCI_INTSTS_G4TXDF_Msk           (0x1ul << PDCI_INTSTS_G4TXDF_Pos)                 /*!< PDCI_T::INTSTS: G4TXDF Mask            */

#define PDCI_INTSTS_G5TXDF_Pos           (21)                                              /*!< PDCI_T::INTSTS: G5TXDF Position        */
#define PDCI_INTSTS_G5TXDF_Msk           (0x1ul << PDCI_INTSTS_G5TXDF_Pos)                 /*!< PDCI_T::INTSTS: G5TXDF Mask            */

#define PDCI_CHEMPTY_CH0EPT_Pos          (0)                                               /*!< PDCI_T::CHEMPTY: CH0EPT Position       */
#define PDCI_CHEMPTY_CH0EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH0EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH0EPT Mask           */

#define PDCI_CHEMPTY_CH1EPT_Pos          (1)                                               /*!< PDCI_T::CHEMPTY: CH1EPT Position       */
#define PDCI_CHEMPTY_CH1EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH1EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH1EPT Mask           */

#define PDCI_CHEMPTY_CH2EPT_Pos          (2)                                               /*!< PDCI_T::CHEMPTY: CH2EPT Position       */
#define PDCI_CHEMPTY_CH2EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH2EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH2EPT Mask           */

#define PDCI_CHEMPTY_CH3EPT_Pos          (3)                                               /*!< PDCI_T::CHEMPTY: CH3EPT Position       */
#define PDCI_CHEMPTY_CH3EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH3EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH3EPT Mask           */

#define PDCI_CHEMPTY_CH4EPT_Pos          (4)                                               /*!< PDCI_T::CHEMPTY: CH4EPT Position       */
#define PDCI_CHEMPTY_CH4EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH4EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH4EPT Mask           */

#define PDCI_CHEMPTY_CH5EPT_Pos          (5)                                               /*!< PDCI_T::CHEMPTY: CH5EPT Position       */
#define PDCI_CHEMPTY_CH5EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH5EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH5EPT Mask           */

#define PDCI_CHEMPTY_CH6EPT_Pos          (6)                                               /*!< PDCI_T::CHEMPTY: CH6EPT Position       */
#define PDCI_CHEMPTY_CH6EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH6EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH6EPT Mask           */

#define PDCI_CHEMPTY_CH7EPT_Pos          (7)                                               /*!< PDCI_T::CHEMPTY: CH7EPT Position       */
#define PDCI_CHEMPTY_CH7EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH7EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH7EPT Mask           */

#define PDCI_CHEMPTY_CH8EPT_Pos          (8)                                               /*!< PDCI_T::CHEMPTY: CH8EPT Position       */
#define PDCI_CHEMPTY_CH8EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH8EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH8EPT Mask           */

#define PDCI_CHEMPTY_CH9EPT_Pos          (9)                                               /*!< PDCI_T::CHEMPTY: CH9EPT Position       */
#define PDCI_CHEMPTY_CH9EPT_Msk          (0x1ul << PDCI_CHEMPTY_CH9EPT_Pos)                /*!< PDCI_T::CHEMPTY: CH9EPT Mask           */

#define PDCI_CHEMPTY_CH10EPT_Pos         (10)                                              /*!< PDCI_T::CHEMPTY: CH10EPT Position      */
#define PDCI_CHEMPTY_CH10EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH10EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH10EPT Mask          */

#define PDCI_CHEMPTY_CH11EPT_Pos         (11)                                              /*!< PDCI_T::CHEMPTY: CH11EPT Position      */
#define PDCI_CHEMPTY_CH11EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH11EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH11EPT Mask          */

#define PDCI_CHEMPTY_CH12EPT_Pos         (12)                                              /*!< PDCI_T::CHEMPTY: CH12EPT Position      */
#define PDCI_CHEMPTY_CH12EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH12EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH12EPT Mask          */

#define PDCI_CHEMPTY_CH13EPT_Pos         (13)                                              /*!< PDCI_T::CHEMPTY: CH13EPT Position      */
#define PDCI_CHEMPTY_CH13EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH13EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH13EPT Mask          */

#define PDCI_CHEMPTY_CH14EPT_Pos         (14)                                              /*!< PDCI_T::CHEMPTY: CH14EPT Position      */
#define PDCI_CHEMPTY_CH14EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH14EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH14EPT Mask          */

#define PDCI_CHEMPTY_CH15EPT_Pos         (15)                                              /*!< PDCI_T::CHEMPTY: CH15EPT Position      */
#define PDCI_CHEMPTY_CH15EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH15EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH15EPT Mask          */

#define PDCI_CHEMPTY_CH16EPT_Pos         (16)                                              /*!< PDCI_T::CHEMPTY: CH16EPT Position      */
#define PDCI_CHEMPTY_CH16EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH16EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH16EPT Mask          */

#define PDCI_CHEMPTY_CH17EPT_Pos         (17)                                              /*!< PDCI_T::CHEMPTY: CH17EPT Position      */
#define PDCI_CHEMPTY_CH17EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH17EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH17EPT Mask          */

#define PDCI_CHEMPTY_CH18EPT_Pos         (18)                                              /*!< PDCI_T::CHEMPTY: CH18EPT Position      */
#define PDCI_CHEMPTY_CH18EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH18EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH18EPT Mask          */

#define PDCI_CHEMPTY_CH19EPT_Pos         (19)                                              /*!< PDCI_T::CHEMPTY: CH19EPT Position      */
#define PDCI_CHEMPTY_CH19EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH19EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH19EPT Mask          */

#define PDCI_CHEMPTY_CH20EPT_Pos         (20)                                              /*!< PDCI_T::CHEMPTY: CH20EPT Position      */
#define PDCI_CHEMPTY_CH20EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH20EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH20EPT Mask          */

#define PDCI_CHEMPTY_CH21EPT_Pos         (21)                                              /*!< PDCI_T::CHEMPTY: CH21EPT Position      */
#define PDCI_CHEMPTY_CH21EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH21EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH21EPT Mask          */

#define PDCI_CHEMPTY_CH22EPT_Pos         (22)                                              /*!< PDCI_T::CHEMPTY: CH22EPT Position      */
#define PDCI_CHEMPTY_CH22EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH22EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH22EPT Mask          */

#define PDCI_CHEMPTY_CH23EPT_Pos         (23)                                              /*!< PDCI_T::CHEMPTY: CH23EPT Position      */
#define PDCI_CHEMPTY_CH23EPT_Msk         (0x1ul << PDCI_CHEMPTY_CH23EPT_Pos)               /*!< PDCI_T::CHEMPTY: CH23EPT Mask          */

#define PDCI_TXDATG0_CH0_TXDAT_Pos       (0)                                               /*!< PDCI_T::TXDATG0: CH0_TXDAT Position    */
#define PDCI_TXDATG0_CH0_TXDAT_Msk       (0xfful << PDCI_TXDATG0_CH0_TXDAT_Pos)            /*!< PDCI_T::TXDATG0: CH0_TXDAT Mask        */

#define PDCI_TXDATG0_CH1_TXDAT_Pos       (8)                                               /*!< PDCI_T::TXDATG0: CH1_TXDAT Position    */
#define PDCI_TXDATG0_CH1_TXDAT_Msk       (0xfful << PDCI_TXDATG0_CH1_TXDAT_Pos)            /*!< PDCI_T::TXDATG0: CH1_TXDAT Mask        */

#define PDCI_TXDATG0_CH2_TXDAT_Pos       (16)                                              /*!< PDCI_T::TXDATG0: CH2_TXDAT Position    */
#define PDCI_TXDATG0_CH2_TXDAT_Msk       (0xfful << PDCI_TXDATG0_CH2_TXDAT_Pos)            /*!< PDCI_T::TXDATG0: CH2_TXDAT Mask        */

#define PDCI_TXDATG0_CH3_TXDAT_Pos       (24)                                              /*!< PDCI_T::TXDATG0: CH3_TXDAT Position    */
#define PDCI_TXDATG0_CH3_TXDAT_Msk       (0xfful << PDCI_TXDATG0_CH3_TXDAT_Pos)            /*!< PDCI_T::TXDATG0: CH3_TXDAT Mask        */

#define PDCI_TXDATG1_CH4_TXDAT_Pos       (0)                                               /*!< PDCI_T::TXDATG1: CH4_TXDAT Position    */
#define PDCI_TXDATG1_CH4_TXDAT_Msk       (0xfful << PDCI_TXDATG1_CH4_TXDAT_Pos)            /*!< PDCI_T::TXDATG1: CH4_TXDAT Mask        */

#define PDCI_TXDATG1_CH5_TXDAT_Pos       (8)                                               /*!< PDCI_T::TXDATG1: CH5_TXDAT Position    */
#define PDCI_TXDATG1_CH5_TXDAT_Msk       (0xfful << PDCI_TXDATG1_CH5_TXDAT_Pos)            /*!< PDCI_T::TXDATG1: CH5_TXDAT Mask        */

#define PDCI_TXDATG1_CH6_TXDAT_Pos       (16)                                              /*!< PDCI_T::TXDATG1: CH6_TXDAT Position    */
#define PDCI_TXDATG1_CH6_TXDAT_Msk       (0xfful << PDCI_TXDATG1_CH6_TXDAT_Pos)            /*!< PDCI_T::TXDATG1: CH6_TXDAT Mask        */

#define PDCI_TXDATG1_CH7_TXDAT_Pos       (24)                                              /*!< PDCI_T::TXDATG1: CH7_TXDAT Position    */
#define PDCI_TXDATG1_CH7_TXDAT_Msk       (0xfful << PDCI_TXDATG1_CH7_TXDAT_Pos)            /*!< PDCI_T::TXDATG1: CH7_TXDAT Mask        */

#define PDCI_TXDATG2_CH8_TXDAT_Pos       (0)                                               /*!< PDCI_T::TXDATG2: CH8_TXDAT Position    */
#define PDCI_TXDATG2_CH8_TXDAT_Msk       (0xfful << PDCI_TXDATG2_CH8_TXDAT_Pos)            /*!< PDCI_T::TXDATG2: CH8_TXDAT Mask        */

#define PDCI_TXDATG2_CH9_TXDAT_Pos       (8)                                               /*!< PDCI_T::TXDATG2: CH9_TXDAT Position    */
#define PDCI_TXDATG2_CH9_TXDAT_Msk       (0xfful << PDCI_TXDATG2_CH9_TXDAT_Pos)            /*!< PDCI_T::TXDATG2: CH9_TXDAT Mask        */

#define PDCI_TXDATG2_CH10_TXDAT_Pos      (16)                                              /*!< PDCI_T::TXDATG2: CH10_TXDAT Position   */
#define PDCI_TXDATG2_CH10_TXDAT_Msk      (0xfful << PDCI_TXDATG2_CH10_TXDAT_Pos)           /*!< PDCI_T::TXDATG2: CH10_TXDAT Mask       */

#define PDCI_TXDATG2_CH11_TXDAT_Pos      (24)                                              /*!< PDCI_T::TXDATG2: CH11_TXDAT Position   */
#define PDCI_TXDATG2_CH11_TXDAT_Msk      (0xfful << PDCI_TXDATG2_CH11_TXDAT_Pos)           /*!< PDCI_T::TXDATG2: CH11_TXDAT Mask       */

#define PDCI_TXDATG3_CH12_TXDAT_Pos      (0)                                               /*!< PDCI_T::TXDATG3: CH12_TXDAT Position   */
#define PDCI_TXDATG3_CH12_TXDAT_Msk      (0xfful << PDCI_TXDATG3_CH12_TXDAT_Pos)           /*!< PDCI_T::TXDATG3: CH12_TXDAT Mask       */

#define PDCI_TXDATG3_CH13_TXDAT_Pos      (8)                                               /*!< PDCI_T::TXDATG3: CH13_TXDAT Position   */
#define PDCI_TXDATG3_CH13_TXDAT_Msk      (0xfful << PDCI_TXDATG3_CH13_TXDAT_Pos)           /*!< PDCI_T::TXDATG3: CH13_TXDAT Mask       */

#define PDCI_TXDATG3_CH14_TXDAT_Pos      (16)                                              /*!< PDCI_T::TXDATG3: CH14_TXDAT Position   */
#define PDCI_TXDATG3_CH14_TXDAT_Msk      (0xfful << PDCI_TXDATG3_CH14_TXDAT_Pos)           /*!< PDCI_T::TXDATG3: CH14_TXDAT Mask       */

#define PDCI_TXDATG3_CH15_TXDAT_Pos      (24)                                              /*!< PDCI_T::TXDATG3: CH15_TXDAT Position   */
#define PDCI_TXDATG3_CH15_TXDAT_Msk      (0xfful << PDCI_TXDATG3_CH15_TXDAT_Pos)           /*!< PDCI_T::TXDATG3: CH15_TXDAT Mask       */

#define PDCI_TXDATG4_CH16_TXDAT_Pos      (0)                                               /*!< PDCI_T::TXDATG4: CH16_TXDAT Position   */
#define PDCI_TXDATG4_CH16_TXDAT_Msk      (0xfful << PDCI_TXDATG4_CH16_TXDAT_Pos)           /*!< PDCI_T::TXDATG4: CH16_TXDAT Mask       */

#define PDCI_TXDATG4_CH17_TXDAT_Pos      (8)                                               /*!< PDCI_T::TXDATG4: CH17_TXDAT Position   */
#define PDCI_TXDATG4_CH17_TXDAT_Msk      (0xfful << PDCI_TXDATG4_CH17_TXDAT_Pos)           /*!< PDCI_T::TXDATG4: CH17_TXDAT Mask       */

#define PDCI_TXDATG4_CH18_TXDAT_Pos      (16)                                              /*!< PDCI_T::TXDATG4: CH18_TXDAT Position   */
#define PDCI_TXDATG4_CH18_TXDAT_Msk      (0xfful << PDCI_TXDATG4_CH18_TXDAT_Pos)           /*!< PDCI_T::TXDATG4: CH18_TXDAT Mask       */

#define PDCI_TXDATG4_CH19_TXDAT_Pos      (24)                                              /*!< PDCI_T::TXDATG4: CH19_TXDAT Position   */
#define PDCI_TXDATG4_CH19_TXDAT_Msk      (0xfful << PDCI_TXDATG4_CH19_TXDAT_Pos)           /*!< PDCI_T::TXDATG4: CH19_TXDAT Mask       */

#define PDCI_TXDATG5_CH20_TXDAT_Pos      (0)                                               /*!< PDCI_T::TXDATG5: CH20_TXDAT Position   */
#define PDCI_TXDATG5_CH20_TXDAT_Msk      (0xfful << PDCI_TXDATG5_CH20_TXDAT_Pos)           /*!< PDCI_T::TXDATG5: CH20_TXDAT Mask       */

#define PDCI_TXDATG5_CH21_TXDAT_Pos      (8)                                               /*!< PDCI_T::TXDATG5: CH21_TXDAT Position   */
#define PDCI_TXDATG5_CH21_TXDAT_Msk      (0xfful << PDCI_TXDATG5_CH21_TXDAT_Pos)           /*!< PDCI_T::TXDATG5: CH21_TXDAT Mask       */

#define PDCI_TXDATG5_CH22_TXDAT_Pos      (16)                                              /*!< PDCI_T::TXDATG5: CH22_TXDAT Position   */
#define PDCI_TXDATG5_CH22_TXDAT_Msk      (0xfful << PDCI_TXDATG5_CH22_TXDAT_Pos)           /*!< PDCI_T::TXDATG5: CH22_TXDAT Mask       */

#define PDCI_TXDATG5_CH23_TXDAT_Pos      (24)                                              /*!< PDCI_T::TXDATG5: CH23_TXDAT Position   */
#define PDCI_TXDATG5_CH23_TXDAT_Msk      (0xfful << PDCI_TXDATG5_CH23_TXDAT_Pos)           /*!< PDCI_T::TXDATG5: CH23_TXDAT Mask       */

#define PDCI_CTL2_PRENUM_Pos             (0)                                               /*!< PDCI_T::CTL2: PRENUM Position          */
#define PDCI_CTL2_PRENUM_Msk             (0xful << PDCI_CTL2_PRENUM_Pos)                   /*!< PDCI_T::CTL2: PRENUM Mask              */

#define PDCI_CTL2_TAILNUM_Pos            (5)                                               /*!< PDCI_T::CTL2: TAILNUM Position         */
#define PDCI_CTL2_TAILNUM_Msk            (0x7ul << PDCI_CTL2_TAILNUM_Pos)                  /*!< PDCI_T::CTL2: TAILNUM Mask             */

#define PDCI_CTL2_KCODES_Pos             (8)                                               /*!< PDCI_T::CTL2: KCODES Position          */
#define PDCI_CTL2_KCODES_Msk             (0x1ul << PDCI_CTL2_KCODES_Pos)                   /*!< PDCI_T::CTL2: KCODES Mask              */

#define PDCI_CTL2_CRCEN_Pos              (9)                                               /*!< PDCI_T::CTL2: CRCEN Position           */
#define PDCI_CTL2_CRCEN_Msk              (0x1ul << PDCI_CTL2_CRCEN_Pos)                    /*!< PDCI_T::CTL2: CRCEN Mask               */

#define PDCI_CTL2_B4B5EN_Pos             (11)                                              /*!< PDCI_T::CTL2: B4B5EN Position          */
#define PDCI_CTL2_B4B5EN_Msk             (0x1ul << PDCI_CTL2_B4B5EN_Pos)                   /*!< PDCI_T::CTL2: B4B5EN Mask              */

#define PDCI_CTL2_TWOCHEN_Pos            (12)                                              /*!< PDCI_T::CTL2: TWOCHEN Position         */
#define PDCI_CTL2_TWOCHEN_Msk            (0x1ul << PDCI_CTL2_TWOCHEN_Pos)                  /*!< PDCI_T::CTL2: TWOCHEN Mask             */

#define PDCI_CTL2_MSB_Pos                (13)                                              /*!< PDCI_T::CTL2: MSB Position             */
#define PDCI_CTL2_MSB_Msk                (0x1ul << PDCI_CTL2_MSB_Pos)                      /*!< PDCI_T::CTL2: MSB Mask                 */

#define PDCI_CTL2_ZERO3B_Pos             (14)                                              /*!< PDCI_T::CTL2: ZERO3B Position          */
#define PDCI_CTL2_ZERO3B_Msk             (0x1ul << PDCI_CTL2_ZERO3B_Pos)                   /*!< PDCI_T::CTL2: ZERO3B Mask              */

#define PDCI_CTL2_MODES_Pos              (16)                                              /*!< PDCI_T::CTL2: MODES Position           */
#define PDCI_CTL2_MODES_Msk              (0x3ul << PDCI_CTL2_MODES_Pos)                    /*!< PDCI_T::CTL2: MODES Mask               */

#define PDCI_CTL2_HDMODE_Pos             (20)                                              /*!< PDCI_T::CTL2: HDMODE Position          */
#define PDCI_CTL2_HDMODE_Msk             (0x1ul << PDCI_CTL2_HDMODE_Pos)                   /*!< PDCI_T::CTL2: HDMODE Mask              */

#define PDCI_KCODE_KCODE1_Pos            (0)                                               /*!< PDCI_T::KCODE: KCODE1 Position         */
#define PDCI_KCODE_KCODE1_Msk            (0x1ful << PDCI_KCODE_KCODE1_Pos)                 /*!< PDCI_T::KCODE: KCODE1 Mask             */

#define PDCI_KCODE_KCODE2_Pos            (8)                                               /*!< PDCI_T::KCODE: KCODE2 Position         */
#define PDCI_KCODE_KCODE2_Msk            (0x1ful << PDCI_KCODE_KCODE2_Pos)                 /*!< PDCI_T::KCODE: KCODE2 Mask             */

#define PDCI_KCODE_KCODE3_Pos            (16)                                              /*!< PDCI_T::KCODE: KCODE3 Position         */
#define PDCI_KCODE_KCODE3_Msk            (0x1ful << PDCI_KCODE_KCODE3_Pos)                 /*!< PDCI_T::KCODE: KCODE3 Mask             */

#define PDCI_KCODE_KCODE4_Pos            (24)                                              /*!< PDCI_T::KCODE: KCODE4 Position         */
#define PDCI_KCODE_KCODE4_Msk            (0x1ful << PDCI_KCODE_KCODE4_Pos)                 /*!< PDCI_T::KCODE: KCODE4 Mask             */

#define PDCI_CONFIGIO_G0TGION_Pos        (0)                                               /*!< PDCI_T::CONFIGIO: G0TGION Position     */
#define PDCI_CONFIGIO_G0TGION_Msk        (0x7ul << PDCI_CONFIGIO_G0TGION_Pos)              /*!< PDCI_T::CONFIGIO: G0TGION Mask         */

#define PDCI_CONFIGIO_G1TGION_Pos        (4)                                               /*!< PDCI_T::CONFIGIO: G1TGION Position     */
#define PDCI_CONFIGIO_G1TGION_Msk        (0x7ul << PDCI_CONFIGIO_G1TGION_Pos)              /*!< PDCI_T::CONFIGIO: G1TGION Mask         */

#define PDCI_CONFIGIO_G2TGION_Pos        (8)                                               /*!< PDCI_T::CONFIGIO: G2TGION Position     */
#define PDCI_CONFIGIO_G2TGION_Msk        (0x7ul << PDCI_CONFIGIO_G2TGION_Pos)              /*!< PDCI_T::CONFIGIO: G2TGION Mask         */

#define PDCI_CONFIGIO_G3TGION_Pos        (12)                                              /*!< PDCI_T::CONFIGIO: G3TGION Position     */
#define PDCI_CONFIGIO_G3TGION_Msk        (0x7ul << PDCI_CONFIGIO_G3TGION_Pos)              /*!< PDCI_T::CONFIGIO: G3TGION Mask         */

#define PDCI_CONFIGIO_G4TGION_Pos        (16)                                              /*!< PDCI_T::CONFIGIO: G4TGION Position     */
#define PDCI_CONFIGIO_G4TGION_Msk        (0x7ul << PDCI_CONFIGIO_G4TGION_Pos)              /*!< PDCI_T::CONFIGIO: G4TGION Mask         */

#define PDCI_CONFIGIO_G5TGION_Pos        (20)                                              /*!< PDCI_T::CONFIGIO: G5TGION Position     */
#define PDCI_CONFIGIO_G5TGION_Msk        (0x7ul << PDCI_CONFIGIO_G5TGION_Pos)              /*!< PDCI_T::CONFIGIO: G5TGION Mask         */

#define PDCI_RXCTL0_RXPDMAEN_Pos         (0)                                               /*!< PDCI_T::RXCTL0: RXPDMAEN Position      */
#define PDCI_RXCTL0_RXPDMAEN_Msk         (0x1ul << PDCI_RXCTL0_RXPDMAEN_Pos)               /*!< PDCI_T::RXCTL0: RXPDMAEN Mask          */

#define PDCI_RXCTL0_RXCRCIEN_Pos         (1)                                               /*!< PDCI_T::RXCTL0: RXCRCIEN Position      */
#define PDCI_RXCTL0_RXCRCIEN_Msk         (0x1ul << PDCI_RXCTL0_RXCRCIEN_Pos)               /*!< PDCI_T::RXCTL0: RXCRCIEN Mask          */

#define PDCI_RXCTL0_RXCRCAIEN_Pos        (2)                                               /*!< PDCI_T::RXCTL0: RXCRCAIEN Position     */
#define PDCI_RXCTL0_RXCRCAIEN_Msk        (0x1ul << PDCI_RXCTL0_RXCRCAIEN_Pos)              /*!< PDCI_T::RXCTL0: RXCRCAIEN Mask         */

#define PDCI_RXCTL0_RXFIEN_Pos           (4)                                               /*!< PDCI_T::RXCTL0: RXFIEN Position        */
#define PDCI_RXCTL0_RXFIEN_Msk           (0x1ul << PDCI_RXCTL0_RXFIEN_Pos)                 /*!< PDCI_T::RXCTL0: RXFIEN Mask            */

#define PDCI_RXCTL0_RXDIEN_Pos           (5)                                               /*!< PDCI_T::RXCTL0: RXDIEN Position        */
#define PDCI_RXCTL0_RXDIEN_Msk           (0x1ul << PDCI_RXCTL0_RXDIEN_Pos)                 /*!< PDCI_T::RXCTL0: RXDIEN Mask            */

#define PDCI_RXCTL0_RXOVIEN_Pos          (6)                                               /*!< PDCI_T::RXCTL0: RXOVIEN Position       */
#define PDCI_RXCTL0_RXOVIEN_Msk          (0x1ul << PDCI_RXCTL0_RXOVIEN_Pos)                /*!< PDCI_T::RXCTL0: RXOVIEN Mask           */

#define PDCI_RXCTL0_RXFAILIEN_Pos        (7)                                               /*!< PDCI_T::RXCTL0: RXFAILIEN Position     */
#define PDCI_RXCTL0_RXFAILIEN_Msk        (0x1ul << PDCI_RXCTL0_RXFAILIEN_Pos)              /*!< PDCI_T::RXCTL0: RXFAILIEN Mask         */

#define PDCI_RXCTL0_RXTHRES_Pos          (8)                                               /*!< PDCI_T::RXCTL0: RXTHRES Position       */
#define PDCI_RXCTL0_RXTHRES_Msk          (0xfful << PDCI_RXCTL0_RXTHRES_Pos)               /*!< PDCI_T::RXCTL0: RXTHRES Mask           */

#define PDCI_RXCTL0_OSPBMODE_Pos         (24)                                              /*!< PDCI_T::RXCTL0: OSPBMODE Position      */
#define PDCI_RXCTL0_OSPBMODE_Msk         (0x1ul << PDCI_RXCTL0_OSPBMODE_Pos)                /*!< PDCI_T::RXCTL0: OSPBMODE Mask          */

#define PDCI_RXCTL1_RX0EN_Pos            (0)                                               /*!< PDCI_T::RXCTL1: RX0EN Position         */
#define PDCI_RXCTL1_RX0EN_Msk            (0x1ul << PDCI_RXCTL1_RX0EN_Pos)                  /*!< PDCI_T::RXCTL1: RX0EN Mask             */

#define PDCI_RXCTL1_RX1EN_Pos            (1)                                               /*!< PDCI_T::RXCTL1: RX1EN Position         */
#define PDCI_RXCTL1_RX1EN_Msk            (0x1ul << PDCI_RXCTL1_RX1EN_Pos)                  /*!< PDCI_T::RXCTL1: RX1EN Mask             */

#define PDCI_RXCTL1_RX2EN_Pos            (2)                                               /*!< PDCI_T::RXCTL1: RX2EN Position         */
#define PDCI_RXCTL1_RX2EN_Msk            (0x1ul << PDCI_RXCTL1_RX2EN_Pos)                  /*!< PDCI_T::RXCTL1: RX2EN Mask             */

#define PDCI_RXCTL1_RX3EN_Pos            (3)                                               /*!< PDCI_T::RXCTL1: RX3EN Position         */
#define PDCI_RXCTL1_RX3EN_Msk            (0x1ul << PDCI_RXCTL1_RX3EN_Pos)                  /*!< PDCI_T::RXCTL1: RX3EN Mask             */

#define PDCI_RXCTL1_RX4EN_Pos            (4)                                               /*!< PDCI_T::RXCTL1: RX4EN Position         */
#define PDCI_RXCTL1_RX4EN_Msk            (0x1ul << PDCI_RXCTL1_RX4EN_Pos)                  /*!< PDCI_T::RXCTL1: RX4EN Mask             */

#define PDCI_RXCTL1_RX5EN_Pos            (5)                                               /*!< PDCI_T::RXCTL1: RX5EN Position         */
#define PDCI_RXCTL1_RX5EN_Msk            (0x1ul << PDCI_RXCTL1_RX5EN_Pos)                  /*!< PDCI_T::RXCTL1: RX5EN Mask             */

#define PDCI_RXCTL1_RX6EN_Pos            (6)                                               /*!< PDCI_T::RXCTL1: RX6EN Position         */
#define PDCI_RXCTL1_RX6EN_Msk            (0x1ul << PDCI_RXCTL1_RX6EN_Pos)                  /*!< PDCI_T::RXCTL1: RX6EN Mask             */

#define PDCI_RXCTL1_RX7EN_Pos            (7)                                               /*!< PDCI_T::RXCTL1: RX7EN Position         */
#define PDCI_RXCTL1_RX7EN_Msk            (0x1ul << PDCI_RXCTL1_RX7EN_Pos)                  /*!< PDCI_T::RXCTL1: RX7EN Mask             */

#define PDCI_RXCTL1_RX8EN_Pos            (8)                                               /*!< PDCI_T::RXCTL1: RX8EN Position         */
#define PDCI_RXCTL1_RX8EN_Msk            (0x1ul << PDCI_RXCTL1_RX8EN_Pos)                  /*!< PDCI_T::RXCTL1: RX8EN Mask             */

#define PDCI_RXCTL1_RX9EN_Pos            (9)                                               /*!< PDCI_T::RXCTL1: RX9EN Position         */
#define PDCI_RXCTL1_RX9EN_Msk            (0x1ul << PDCI_RXCTL1_RX9EN_Pos)                  /*!< PDCI_T::RXCTL1: RX9EN Mask             */

#define PDCI_RXCTL1_RX10EN_Pos           (10)                                              /*!< PDCI_T::RXCTL1: RX10EN Position        */
#define PDCI_RXCTL1_RX10EN_Msk           (0x1ul << PDCI_RXCTL1_RX10EN_Pos)                 /*!< PDCI_T::RXCTL1: RX10EN Mask            */

#define PDCI_RXCTL1_RX11EN_Pos           (11)                                              /*!< PDCI_T::RXCTL1: RX11EN Position        */
#define PDCI_RXCTL1_RX11EN_Msk           (0x1ul << PDCI_RXCTL1_RX11EN_Pos)                 /*!< PDCI_T::RXCTL1: RX11EN Mask            */

#define PDCI_RXCTL1_RX12EN_Pos           (12)                                              /*!< PDCI_T::RXCTL1: RX12EN Position        */
#define PDCI_RXCTL1_RX12EN_Msk           (0x1ul << PDCI_RXCTL1_RX12EN_Pos)                 /*!< PDCI_T::RXCTL1: RX12EN Mask            */

#define PDCI_RXCTL1_RX13EN_Pos           (13)                                              /*!< PDCI_T::RXCTL1: RX13EN Position        */
#define PDCI_RXCTL1_RX13EN_Msk           (0x1ul << PDCI_RXCTL1_RX13EN_Pos)                 /*!< PDCI_T::RXCTL1: RX13EN Mask            */

#define PDCI_RXCTL1_RX14EN_Pos           (14)                                              /*!< PDCI_T::RXCTL1: RX14EN Position        */
#define PDCI_RXCTL1_RX14EN_Msk           (0x1ul << PDCI_RXCTL1_RX14EN_Pos)                 /*!< PDCI_T::RXCTL1: RX14EN Mask            */

#define PDCI_RXCTL1_RX15EN_Pos           (15)                                              /*!< PDCI_T::RXCTL1: RX15EN Position        */
#define PDCI_RXCTL1_RX15EN_Msk           (0x1ul << PDCI_RXCTL1_RX15EN_Pos)                 /*!< PDCI_T::RXCTL1: RX15EN Mask            */

#define PDCI_RXCTL1_RX16EN_Pos           (16)                                              /*!< PDCI_T::RXCTL1: RX16EN Position        */
#define PDCI_RXCTL1_RX16EN_Msk           (0x1ul << PDCI_RXCTL1_RX16EN_Pos)                 /*!< PDCI_T::RXCTL1: RX16EN Mask            */

#define PDCI_RXCTL1_RX17EN_Pos           (17)                                              /*!< PDCI_T::RXCTL1: RX17EN Position        */
#define PDCI_RXCTL1_RX17EN_Msk           (0x1ul << PDCI_RXCTL1_RX17EN_Pos)                 /*!< PDCI_T::RXCTL1: RX17EN Mask            */

#define PDCI_RXCTL1_RX18EN_Pos           (18)                                              /*!< PDCI_T::RXCTL1: RX18EN Position        */
#define PDCI_RXCTL1_RX18EN_Msk           (0x1ul << PDCI_RXCTL1_RX18EN_Pos)                 /*!< PDCI_T::RXCTL1: RX18EN Mask            */

#define PDCI_RXCTL1_RX19EN_Pos           (19)                                              /*!< PDCI_T::RXCTL1: RX19EN Position        */
#define PDCI_RXCTL1_RX19EN_Msk           (0x1ul << PDCI_RXCTL1_RX19EN_Pos)                 /*!< PDCI_T::RXCTL1: RX19EN Mask            */

#define PDCI_RXCTL1_RX20EN_Pos           (20)                                              /*!< PDCI_T::RXCTL1: RX20EN Position        */
#define PDCI_RXCTL1_RX20EN_Msk           (0x1ul << PDCI_RXCTL1_RX20EN_Pos)                 /*!< PDCI_T::RXCTL1: RX20EN Mask            */

#define PDCI_RXCTL1_RX21EN_Pos           (21)                                              /*!< PDCI_T::RXCTL1: RX21EN Position        */
#define PDCI_RXCTL1_RX21EN_Msk           (0x1ul << PDCI_RXCTL1_RX21EN_Pos)                 /*!< PDCI_T::RXCTL1: RX21EN Mask            */

#define PDCI_RXCTL1_RX22EN_Pos           (22)                                              /*!< PDCI_T::RXCTL1: RX22EN Position        */
#define PDCI_RXCTL1_RX22EN_Msk           (0x1ul << PDCI_RXCTL1_RX22EN_Pos)                 /*!< PDCI_T::RXCTL1: RX22EN Mask            */

#define PDCI_RXCTL1_RX23EN_Pos           (23)                                              /*!< PDCI_T::RXCTL1: RX23EN Position        */
#define PDCI_RXCTL1_RX23EN_Msk           (0x1ul << PDCI_RXCTL1_RX23EN_Pos)                 /*!< PDCI_T::RXCTL1: RX23EN Mask            */

#define PDCI_RXCNT_RXADDRC_Pos           (0)                                               /*!< PDCI_T::RXCNT: RXADDRC Position        */
#define PDCI_RXCNT_RXADDRC_Msk           (0xful << PDCI_RXCNT_RXADDRC_Pos)                 /*!< PDCI_T::RXCNT: RXADDRC Mask            */

#define PDCI_RXG01STS_RXDAT0F_Pos        (0)                                               /*!< PDCI_T::RXG01STS: RXDAT0F Position     */
#define PDCI_RXG01STS_RXDAT0F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT0F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT0F Mask         */

#define PDCI_RXG01STS_RXDAT1F_Pos        (1)                                               /*!< PDCI_T::RXG01STS: RXDAT1F Position     */
#define PDCI_RXG01STS_RXDAT1F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT1F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT1F Mask         */

#define PDCI_RXG01STS_RXDAT2F_Pos        (2)                                               /*!< PDCI_T::RXG01STS: RXDAT2F Position     */
#define PDCI_RXG01STS_RXDAT2F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT2F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT2F Mask         */

#define PDCI_RXG01STS_RXDAT3F_Pos        (3)                                               /*!< PDCI_T::RXG01STS: RXDAT3F Position     */
#define PDCI_RXG01STS_RXDAT3F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT3F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT3F Mask         */

#define PDCI_RXG01STS_RXDAT4F_Pos        (4)                                               /*!< PDCI_T::RXG01STS: RXDAT4F Position     */
#define PDCI_RXG01STS_RXDAT4F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT4F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT4F Mask         */

#define PDCI_RXG01STS_RXDAT5F_Pos        (5)                                               /*!< PDCI_T::RXG01STS: RXDAT5F Position     */
#define PDCI_RXG01STS_RXDAT5F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT5F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT5F Mask         */

#define PDCI_RXG01STS_RXDAT6F_Pos        (6)                                               /*!< PDCI_T::RXG01STS: RXDAT6F Position     */
#define PDCI_RXG01STS_RXDAT6F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT6F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT6F Mask         */

#define PDCI_RXG01STS_RXDAT7F_Pos        (7)                                               /*!< PDCI_T::RXG01STS: RXDAT7F Position     */
#define PDCI_RXG01STS_RXDAT7F_Msk        (0x1ul << PDCI_RXG01STS_RXDAT7F_Pos)              /*!< PDCI_T::RXG01STS: RXDAT7F Mask         */

#define PDCI_RXG01STS_RX0DONE_Pos        (8)                                               /*!< PDCI_T::RXG01STS: RX0DONE Position     */
#define PDCI_RXG01STS_RX0DONE_Msk        (0x1ul << PDCI_RXG01STS_RX0DONE_Pos)              /*!< PDCI_T::RXG01STS: RX0DONE Mask         */

#define PDCI_RXG01STS_RX1DONE_Pos        (9)                                               /*!< PDCI_T::RXG01STS: RX1DONE Position     */
#define PDCI_RXG01STS_RX1DONE_Msk        (0x1ul << PDCI_RXG01STS_RX1DONE_Pos)              /*!< PDCI_T::RXG01STS: RX1DONE Mask         */

#define PDCI_RXG01STS_RX2DONE_Pos        (10)                                              /*!< PDCI_T::RXG01STS: RX2DONE Position     */
#define PDCI_RXG01STS_RX2DONE_Msk        (0x1ul << PDCI_RXG01STS_RX2DONE_Pos)              /*!< PDCI_T::RXG01STS: RX2DONE Mask         */

#define PDCI_RXG01STS_RX3DONE_Pos        (11)                                              /*!< PDCI_T::RXG01STS: RX3DONE Position     */
#define PDCI_RXG01STS_RX3DONE_Msk        (0x1ul << PDCI_RXG01STS_RX3DONE_Pos)              /*!< PDCI_T::RXG01STS: RX3DONE Mask         */

#define PDCI_RXG01STS_RX4DONE_Pos        (12)                                              /*!< PDCI_T::RXG01STS: RX4DONE Position     */
#define PDCI_RXG01STS_RX4DONE_Msk        (0x1ul << PDCI_RXG01STS_RX4DONE_Pos)              /*!< PDCI_T::RXG01STS: RX4DONE Mask         */

#define PDCI_RXG01STS_RX5DONE_Pos        (13)                                              /*!< PDCI_T::RXG01STS: RX5DONE Position     */
#define PDCI_RXG01STS_RX5DONE_Msk        (0x1ul << PDCI_RXG01STS_RX5DONE_Pos)              /*!< PDCI_T::RXG01STS: RX5DONE Mask         */

#define PDCI_RXG01STS_RX6DONE_Pos        (14)                                              /*!< PDCI_T::RXG01STS: RX6DONE Position     */
#define PDCI_RXG01STS_RX6DONE_Msk        (0x1ul << PDCI_RXG01STS_RX6DONE_Pos)              /*!< PDCI_T::RXG01STS: RX6DONE Mask         */

#define PDCI_RXG01STS_RX7DONE_Pos        (15)                                              /*!< PDCI_T::RXG01STS: RX7DONE Position     */
#define PDCI_RXG01STS_RX7DONE_Msk        (0x1ul << PDCI_RXG01STS_RX7DONE_Pos)              /*!< PDCI_T::RXG01STS: RX7DONE Mask         */

#define PDCI_RXG01STS_RX0OV_Pos          (16)                                              /*!< PDCI_T::RXG01STS: RX0OV Position       */
#define PDCI_RXG01STS_RX0OV_Msk          (0x1ul << PDCI_RXG01STS_RX0OV_Pos)                /*!< PDCI_T::RXG01STS: RX0OV Mask           */

#define PDCI_RXG01STS_RX1OV_Pos          (17)                                              /*!< PDCI_T::RXG01STS: RX1OV Position       */
#define PDCI_RXG01STS_RX1OV_Msk          (0x1ul << PDCI_RXG01STS_RX1OV_Pos)                /*!< PDCI_T::RXG01STS: RX1OV Mask           */

#define PDCI_RXG01STS_RX2OV_Pos          (18)                                              /*!< PDCI_T::RXG01STS: RX2OV Position       */
#define PDCI_RXG01STS_RX2OV_Msk          (0x1ul << PDCI_RXG01STS_RX2OV_Pos)                /*!< PDCI_T::RXG01STS: RX2OV Mask           */

#define PDCI_RXG01STS_RX3OV_Pos          (19)                                              /*!< PDCI_T::RXG01STS: RX3OV Position       */
#define PDCI_RXG01STS_RX3OV_Msk          (0x1ul << PDCI_RXG01STS_RX3OV_Pos)                /*!< PDCI_T::RXG01STS: RX3OV Mask           */

#define PDCI_RXG01STS_RX4OV_Pos          (20)                                              /*!< PDCI_T::RXG01STS: RX4OV Position       */
#define PDCI_RXG01STS_RX4OV_Msk          (0x1ul << PDCI_RXG01STS_RX4OV_Pos)                /*!< PDCI_T::RXG01STS: RX4OV Mask           */

#define PDCI_RXG01STS_RX5OV_Pos          (21)                                              /*!< PDCI_T::RXG01STS: RX5OV Position       */
#define PDCI_RXG01STS_RX5OV_Msk          (0x1ul << PDCI_RXG01STS_RX5OV_Pos)                /*!< PDCI_T::RXG01STS: RX5OV Mask           */

#define PDCI_RXG01STS_RX6OV_Pos          (22)                                              /*!< PDCI_T::RXG01STS: RX6OV Position       */
#define PDCI_RXG01STS_RX6OV_Msk          (0x1ul << PDCI_RXG01STS_RX6OV_Pos)                /*!< PDCI_T::RXG01STS: RX6OV Mask           */

#define PDCI_RXG01STS_RX7OV_Pos          (23)                                              /*!< PDCI_T::RXG01STS: RX7OV Position       */
#define PDCI_RXG01STS_RX7OV_Msk          (0x1ul << PDCI_RXG01STS_RX7OV_Pos)                /*!< PDCI_T::RXG01STS: RX7OV Mask           */

#define PDCI_RXG01STS_RX0FAIL_Pos        (24)                                              /*!< PDCI_T::RXG01STS: RX0FAIL Position     */
#define PDCI_RXG01STS_RX0FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX0FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX0FAIL Mask         */

#define PDCI_RXG01STS_RX1FAIL_Pos        (25)                                              /*!< PDCI_T::RXG01STS: RX1FAIL Position     */
#define PDCI_RXG01STS_RX1FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX1FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX1FAIL Mask         */

#define PDCI_RXG01STS_RX2FAIL_Pos        (26)                                              /*!< PDCI_T::RXG01STS: RX2FAIL Position     */
#define PDCI_RXG01STS_RX2FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX2FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX2FAIL Mask         */

#define PDCI_RXG01STS_RX3FAIL_Pos        (27)                                              /*!< PDCI_T::RXG01STS: RX3FAIL Position     */
#define PDCI_RXG01STS_RX3FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX3FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX3FAIL Mask         */

#define PDCI_RXG01STS_RX4FAIL_Pos        (28)                                              /*!< PDCI_T::RXG01STS: RX4FAIL Position     */
#define PDCI_RXG01STS_RX4FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX4FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX4FAIL Mask         */

#define PDCI_RXG01STS_RX5FAIL_Pos        (29)                                              /*!< PDCI_T::RXG01STS: RX5FAIL Position     */
#define PDCI_RXG01STS_RX5FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX5FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX5FAIL Mask         */

#define PDCI_RXG01STS_RX6FAIL_Pos        (30)                                              /*!< PDCI_T::RXG01STS: RX6FAIL Position     */
#define PDCI_RXG01STS_RX6FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX6FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX6FAIL Mask         */

#define PDCI_RXG01STS_RX7FAIL_Pos        (31)                                              /*!< PDCI_T::RXG01STS: RX7FAIL Position     */
#define PDCI_RXG01STS_RX7FAIL_Msk        (0x1ul << PDCI_RXG01STS_RX7FAIL_Pos)              /*!< PDCI_T::RXG01STS: RX7FAIL Mask         */

#define PDCI_RXG23STS_RXDAT8F_Pos        (0)                                               /*!< PDCI_T::RXG23STS: RXDAT8F Position     */
#define PDCI_RXG23STS_RXDAT8F_Msk        (0x1ul << PDCI_RXG23STS_RXDAT8F_Pos)              /*!< PDCI_T::RXG23STS: RXDAT8F Mask         */

#define PDCI_RXG23STS_RXDAT9F_Pos        (1)                                               /*!< PDCI_T::RXG23STS: RXDAT9F Position     */
#define PDCI_RXG23STS_RXDAT9F_Msk        (0x1ul << PDCI_RXG23STS_RXDAT9F_Pos)              /*!< PDCI_T::RXG23STS: RXDAT9F Mask         */

#define PDCI_RXG23STS_RXDAT10F_Pos       (2)                                               /*!< PDCI_T::RXG23STS: RXDAT10F Position    */
#define PDCI_RXG23STS_RXDAT10F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT10F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT10F Mask        */

#define PDCI_RXG23STS_RXDAT11F_Pos       (3)                                               /*!< PDCI_T::RXG23STS: RXDAT11F Position    */
#define PDCI_RXG23STS_RXDAT11F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT11F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT11F Mask        */

#define PDCI_RXG23STS_RXDAT12F_Pos       (4)                                               /*!< PDCI_T::RXG23STS: RXDAT12F Position    */
#define PDCI_RXG23STS_RXDAT12F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT12F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT12F Mask        */

#define PDCI_RXG23STS_RXDAT13F_Pos       (5)                                               /*!< PDCI_T::RXG23STS: RXDAT13F Position    */
#define PDCI_RXG23STS_RXDAT13F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT13F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT13F Mask        */

#define PDCI_RXG23STS_RXDAT14F_Pos       (6)                                               /*!< PDCI_T::RXG23STS: RXDAT14F Position    */
#define PDCI_RXG23STS_RXDAT14F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT14F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT14F Mask        */

#define PDCI_RXG23STS_RXDAT15F_Pos       (7)                                               /*!< PDCI_T::RXG23STS: RXDAT15F Position    */
#define PDCI_RXG23STS_RXDAT15F_Msk       (0x1ul << PDCI_RXG23STS_RXDAT15F_Pos)             /*!< PDCI_T::RXG23STS: RXDAT15F Mask        */

#define PDCI_RXG23STS_RX8DONE_Pos        (8)                                               /*!< PDCI_T::RXG23STS: RX8DONE Position     */
#define PDCI_RXG23STS_RX8DONE_Msk        (0x1ul << PDCI_RXG23STS_RX8DONE_Pos)              /*!< PDCI_T::RXG23STS: RX8DONE Mask         */

#define PDCI_RXG23STS_RX9DONE_Pos        (9)                                               /*!< PDCI_T::RXG23STS: RX9DONE Position     */
#define PDCI_RXG23STS_RX9DONE_Msk        (0x1ul << PDCI_RXG23STS_RX9DONE_Pos)              /*!< PDCI_T::RXG23STS: RX9DONE Mask         */

#define PDCI_RXG23STS_RX10DONE_Pos       (10)                                              /*!< PDCI_T::RXG23STS: RX10DONE Position    */
#define PDCI_RXG23STS_RX10DONE_Msk       (0x1ul << PDCI_RXG23STS_RX10DONE_Pos)             /*!< PDCI_T::RXG23STS: RX10DONE Mask        */

#define PDCI_RXG23STS_RX11DONE_Pos       (11)                                              /*!< PDCI_T::RXG23STS: RX11DONE Position    */
#define PDCI_RXG23STS_RX11DONE_Msk       (0x1ul << PDCI_RXG23STS_RX11DONE_Pos)             /*!< PDCI_T::RXG23STS: RX11DONE Mask        */

#define PDCI_RXG23STS_RX12DONE_Pos       (12)                                              /*!< PDCI_T::RXG23STS: RX12DONE Position    */
#define PDCI_RXG23STS_RX12DONE_Msk       (0x1ul << PDCI_RXG23STS_RX12DONE_Pos)             /*!< PDCI_T::RXG23STS: RX12DONE Mask        */

#define PDCI_RXG23STS_RX13DONE_Pos       (13)                                              /*!< PDCI_T::RXG23STS: RX13DONE Position    */
#define PDCI_RXG23STS_RX13DONE_Msk       (0x1ul << PDCI_RXG23STS_RX13DONE_Pos)             /*!< PDCI_T::RXG23STS: RX13DONE Mask        */

#define PDCI_RXG23STS_RX14DONE_Pos       (14)                                              /*!< PDCI_T::RXG23STS: RX14DONE Position    */
#define PDCI_RXG23STS_RX14DONE_Msk       (0x1ul << PDCI_RXG23STS_RX14DONE_Pos)             /*!< PDCI_T::RXG23STS: RX14DONE Mask        */

#define PDCI_RXG23STS_RX15DONE_Pos       (15)                                              /*!< PDCI_T::RXG23STS: RX15DONE Position    */
#define PDCI_RXG23STS_RX15DONE_Msk       (0x1ul << PDCI_RXG23STS_RX15DONE_Pos)             /*!< PDCI_T::RXG23STS: RX15DONE Mask        */

#define PDCI_RXG23STS_RX8OV_Pos          (16)                                              /*!< PDCI_T::RXG23STS: RX8OV Position       */
#define PDCI_RXG23STS_RX8OV_Msk          (0x1ul << PDCI_RXG23STS_RX8OV_Pos)                /*!< PDCI_T::RXG23STS: RX8OV Mask           */

#define PDCI_RXG23STS_RX9OV_Pos          (17)                                              /*!< PDCI_T::RXG23STS: RX9OV Position       */
#define PDCI_RXG23STS_RX9OV_Msk          (0x1ul << PDCI_RXG23STS_RX9OV_Pos)                /*!< PDCI_T::RXG23STS: RX9OV Mask           */

#define PDCI_RXG23STS_RX10OV_Pos         (18)                                              /*!< PDCI_T::RXG23STS: RX10OV Position      */
#define PDCI_RXG23STS_RX10OV_Msk         (0x1ul << PDCI_RXG23STS_RX10OV_Pos)               /*!< PDCI_T::RXG23STS: RX10OV Mask          */

#define PDCI_RXG23STS_RX11OV_Pos         (19)                                              /*!< PDCI_T::RXG23STS: RX11OV Position      */
#define PDCI_RXG23STS_RX11OV_Msk         (0x1ul << PDCI_RXG23STS_RX11OV_Pos)               /*!< PDCI_T::RXG23STS: RX11OV Mask          */

#define PDCI_RXG23STS_RX12OV_Pos         (20)                                              /*!< PDCI_T::RXG23STS: RX12OV Position      */
#define PDCI_RXG23STS_RX12OV_Msk         (0x1ul << PDCI_RXG23STS_RX12OV_Pos)               /*!< PDCI_T::RXG23STS: RX12OV Mask          */

#define PDCI_RXG23STS_RX13OV_Pos         (21)                                              /*!< PDCI_T::RXG23STS: RX13OV Position      */
#define PDCI_RXG23STS_RX13OV_Msk         (0x1ul << PDCI_RXG23STS_RX13OV_Pos)               /*!< PDCI_T::RXG23STS: RX13OV Mask          */

#define PDCI_RXG23STS_RX14OV_Pos         (22)                                              /*!< PDCI_T::RXG23STS: RX14OV Position      */
#define PDCI_RXG23STS_RX14OV_Msk         (0x1ul << PDCI_RXG23STS_RX14OV_Pos)               /*!< PDCI_T::RXG23STS: RX14OV Mask          */

#define PDCI_RXG23STS_RX15OV_Pos         (23)                                              /*!< PDCI_T::RXG23STS: RX15OV Position      */
#define PDCI_RXG23STS_RX15OV_Msk         (0x1ul << PDCI_RXG23STS_RX15OV_Pos)               /*!< PDCI_T::RXG23STS: RX15OV Mask          */

#define PDCI_RXG23STS_RX8FAIL_Pos        (24)                                              /*!< PDCI_T::RXG23STS: RX8FAIL Position     */
#define PDCI_RXG23STS_RX8FAIL_Msk        (0x1ul << PDCI_RXG23STS_RX8FAIL_Pos)              /*!< PDCI_T::RXG23STS: RX8FAIL Mask         */

#define PDCI_RXG23STS_RX9FAIL_Pos        (25)                                              /*!< PDCI_T::RXG23STS: RX9FAIL Position     */
#define PDCI_RXG23STS_RX9FAIL_Msk        (0x1ul << PDCI_RXG23STS_RX9FAIL_Pos)              /*!< PDCI_T::RXG23STS: RX9FAIL Mask         */

#define PDCI_RXG23STS_RX10FAIL_Pos       (26)                                              /*!< PDCI_T::RXG23STS: RX10FAIL Position    */
#define PDCI_RXG23STS_RX10FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX10FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX10FAIL Mask        */

#define PDCI_RXG23STS_RX11FAIL_Pos       (27)                                              /*!< PDCI_T::RXG23STS: RX11FAIL Position    */
#define PDCI_RXG23STS_RX11FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX11FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX11FAIL Mask        */

#define PDCI_RXG23STS_RX12FAIL_Pos       (28)                                              /*!< PDCI_T::RXG23STS: RX12FAIL Position    */
#define PDCI_RXG23STS_RX12FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX12FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX12FAIL Mask        */

#define PDCI_RXG23STS_RX13FAIL_Pos       (29)                                              /*!< PDCI_T::RXG23STS: RX13FAIL Position    */
#define PDCI_RXG23STS_RX13FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX13FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX13FAIL Mask        */

#define PDCI_RXG23STS_RX14FAIL_Pos       (30)                                              /*!< PDCI_T::RXG23STS: RX14FAIL Position    */
#define PDCI_RXG23STS_RX14FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX14FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX14FAIL Mask        */

#define PDCI_RXG23STS_RX15FAIL_Pos       (31)                                              /*!< PDCI_T::RXG23STS: RX15FAIL Position    */
#define PDCI_RXG23STS_RX15FAIL_Msk       (0x1ul << PDCI_RXG23STS_RX15FAIL_Pos)             /*!< PDCI_T::RXG23STS: RX15FAIL Mask        */

#define PDCI_RXG45STS_RXDAT16F_Pos       (0)                                               /*!< PDCI_T::RXG45STS: RXDAT16F Position    */
#define PDCI_RXG45STS_RXDAT16F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT16F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT16F Mask        */

#define PDCI_RXG45STS_RXDAT17F_Pos       (1)                                               /*!< PDCI_T::RXG45STS: RXDAT17F Position    */
#define PDCI_RXG45STS_RXDAT17F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT17F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT17F Mask        */

#define PDCI_RXG45STS_RXDAT18F_Pos       (2)                                               /*!< PDCI_T::RXG45STS: RXDAT18F Position    */
#define PDCI_RXG45STS_RXDAT18F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT18F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT18F Mask        */

#define PDCI_RXG45STS_RXDAT19F_Pos       (3)                                               /*!< PDCI_T::RXG45STS: RXDAT19F Position    */
#define PDCI_RXG45STS_RXDAT19F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT19F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT19F Mask        */

#define PDCI_RXG45STS_RXDAT20F_Pos       (4)                                               /*!< PDCI_T::RXG45STS: RXDAT20F Position    */
#define PDCI_RXG45STS_RXDAT20F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT20F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT20F Mask        */

#define PDCI_RXG45STS_RXDAT21F_Pos       (5)                                               /*!< PDCI_T::RXG45STS: RXDAT21F Position    */
#define PDCI_RXG45STS_RXDAT21F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT21F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT21F Mask        */

#define PDCI_RXG45STS_RXDAT22F_Pos       (6)                                               /*!< PDCI_T::RXG45STS: RXDAT22F Position    */
#define PDCI_RXG45STS_RXDAT22F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT22F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT22F Mask        */

#define PDCI_RXG45STS_RXDAT23F_Pos       (7)                                               /*!< PDCI_T::RXG45STS: RXDAT23F Position    */
#define PDCI_RXG45STS_RXDAT23F_Msk       (0x1ul << PDCI_RXG45STS_RXDAT23F_Pos)             /*!< PDCI_T::RXG45STS: RXDAT23F Mask        */

#define PDCI_RXG45STS_RX16DONE_Pos       (8)                                               /*!< PDCI_T::RXG45STS: RX16DONE Position    */
#define PDCI_RXG45STS_RX16DONE_Msk       (0x1ul << PDCI_RXG45STS_RX16DONE_Pos)             /*!< PDCI_T::RXG45STS: RX16DONE Mask        */

#define PDCI_RXG45STS_RX17DONE_Pos       (9)                                               /*!< PDCI_T::RXG45STS: RX17DONE Position    */
#define PDCI_RXG45STS_RX17DONE_Msk       (0x1ul << PDCI_RXG45STS_RX17DONE_Pos)             /*!< PDCI_T::RXG45STS: RX17DONE Mask        */

#define PDCI_RXG45STS_RX18DONE_Pos       (10)                                              /*!< PDCI_T::RXG45STS: RX18DONE Position    */
#define PDCI_RXG45STS_RX18DONE_Msk       (0x1ul << PDCI_RXG45STS_RX18DONE_Pos)             /*!< PDCI_T::RXG45STS: RX18DONE Mask        */

#define PDCI_RXG45STS_RX19DONE_Pos       (11)                                              /*!< PDCI_T::RXG45STS: RX19DONE Position    */
#define PDCI_RXG45STS_RX19DONE_Msk       (0x1ul << PDCI_RXG45STS_RX19DONE_Pos)             /*!< PDCI_T::RXG45STS: RX19DONE Mask        */

#define PDCI_RXG45STS_RX20DONE_Pos       (12)                                              /*!< PDCI_T::RXG45STS: RX20DONE Position    */
#define PDCI_RXG45STS_RX20DONE_Msk       (0x1ul << PDCI_RXG45STS_RX20DONE_Pos)             /*!< PDCI_T::RXG45STS: RX20DONE Mask        */

#define PDCI_RXG45STS_RX21DONE_Pos       (13)                                              /*!< PDCI_T::RXG45STS: RX21DONE Position    */
#define PDCI_RXG45STS_RX21DONE_Msk       (0x1ul << PDCI_RXG45STS_RX21DONE_Pos)             /*!< PDCI_T::RXG45STS: RX21DONE Mask        */

#define PDCI_RXG45STS_RX22DONE_Pos       (14)                                              /*!< PDCI_T::RXG45STS: RX22DONE Position    */
#define PDCI_RXG45STS_RX22DONE_Msk       (0x1ul << PDCI_RXG45STS_RX22DONE_Pos)             /*!< PDCI_T::RXG45STS: RX22DONE Mask        */

#define PDCI_RXG45STS_RX23DONE_Pos       (15)                                              /*!< PDCI_T::RXG45STS: RX23DONE Position    */
#define PDCI_RXG45STS_RX23DONE_Msk       (0x1ul << PDCI_RXG45STS_RX23DONE_Pos)             /*!< PDCI_T::RXG45STS: RX23DONE Mask        */

#define PDCI_RXG45STS_RX16OV_Pos         (16)                                              /*!< PDCI_T::RXG45STS: RX16OV Position      */
#define PDCI_RXG45STS_RX16OV_Msk         (0x1ul << PDCI_RXG45STS_RX16OV_Pos)               /*!< PDCI_T::RXG45STS: RX16OV Mask          */

#define PDCI_RXG45STS_RX17OV_Pos         (17)                                              /*!< PDCI_T::RXG45STS: RX17OV Position      */
#define PDCI_RXG45STS_RX17OV_Msk         (0x1ul << PDCI_RXG45STS_RX17OV_Pos)               /*!< PDCI_T::RXG45STS: RX17OV Mask          */

#define PDCI_RXG45STS_RX18OV_Pos         (18)                                              /*!< PDCI_T::RXG45STS: RX18OV Position      */
#define PDCI_RXG45STS_RX18OV_Msk         (0x1ul << PDCI_RXG45STS_RX18OV_Pos)               /*!< PDCI_T::RXG45STS: RX18OV Mask          */

#define PDCI_RXG45STS_RX19OV_Pos         (19)                                              /*!< PDCI_T::RXG45STS: RX19OV Position      */
#define PDCI_RXG45STS_RX19OV_Msk         (0x1ul << PDCI_RXG45STS_RX19OV_Pos)               /*!< PDCI_T::RXG45STS: RX19OV Mask          */

#define PDCI_RXG45STS_RX20OV_Pos         (20)                                              /*!< PDCI_T::RXG45STS: RX20OV Position      */
#define PDCI_RXG45STS_RX20OV_Msk         (0x1ul << PDCI_RXG45STS_RX20OV_Pos)               /*!< PDCI_T::RXG45STS: RX20OV Mask          */

#define PDCI_RXG45STS_RX21OV_Pos         (21)                                              /*!< PDCI_T::RXG45STS: RX21OV Position      */
#define PDCI_RXG45STS_RX21OV_Msk         (0x1ul << PDCI_RXG45STS_RX21OV_Pos)               /*!< PDCI_T::RXG45STS: RX21OV Mask          */

#define PDCI_RXG45STS_RX22OV_Pos         (22)                                              /*!< PDCI_T::RXG45STS: RX22OV Position      */
#define PDCI_RXG45STS_RX22OV_Msk         (0x1ul << PDCI_RXG45STS_RX22OV_Pos)               /*!< PDCI_T::RXG45STS: RX22OV Mask          */

#define PDCI_RXG45STS_RX23OV_Pos         (23)                                              /*!< PDCI_T::RXG45STS: RX23OV Position      */
#define PDCI_RXG45STS_RX23OV_Msk         (0x1ul << PDCI_RXG45STS_RX23OV_Pos)               /*!< PDCI_T::RXG45STS: RX23OV Mask          */

#define PDCI_RXG45STS_RX16FAIL_Pos       (24)                                              /*!< PDCI_T::RXG45STS: RX16FAIL Position    */
#define PDCI_RXG45STS_RX16FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX16FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX16FAIL Mask        */

#define PDCI_RXG45STS_RX17FAIL_Pos       (25)                                              /*!< PDCI_T::RXG45STS: RX17FAIL Position    */
#define PDCI_RXG45STS_RX17FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX17FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX17FAIL Mask        */

#define PDCI_RXG45STS_RX18FAIL_Pos       (26)                                              /*!< PDCI_T::RXG45STS: RX18FAIL Position    */
#define PDCI_RXG45STS_RX18FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX18FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX18FAIL Mask        */

#define PDCI_RXG45STS_RX19FAIL_Pos       (27)                                              /*!< PDCI_T::RXG45STS: RX19FAIL Position    */
#define PDCI_RXG45STS_RX19FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX19FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX19FAIL Mask        */

#define PDCI_RXG45STS_RX20FAIL_Pos       (28)                                              /*!< PDCI_T::RXG45STS: RX20FAIL Position    */
#define PDCI_RXG45STS_RX20FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX20FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX20FAIL Mask        */

#define PDCI_RXG45STS_RX21FAIL_Pos       (29)                                              /*!< PDCI_T::RXG45STS: RX21FAIL Position    */
#define PDCI_RXG45STS_RX21FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX21FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX21FAIL Mask        */

#define PDCI_RXG45STS_RX22FAIL_Pos       (30)                                              /*!< PDCI_T::RXG45STS: RX22FAIL Position    */
#define PDCI_RXG45STS_RX22FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX22FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX22FAIL Mask        */

#define PDCI_RXG45STS_RX23FAIL_Pos       (31)                                              /*!< PDCI_T::RXG45STS: RX23FAIL Position    */
#define PDCI_RXG45STS_RX23FAIL_Msk       (0x1ul << PDCI_RXG45STS_RX23FAIL_Pos)             /*!< PDCI_T::RXG45STS: RX23FAIL Mask        */

#define PDCI_RXDATG0_CH0_RXDAT_Pos       (0)                                               /*!< PDCI_T::RXDATG0: CH0_RXDAT Position    */
#define PDCI_RXDATG0_CH0_RXDAT_Msk       (0xfful << PDCI_RXDATG0_CH0_RXDAT_Pos)            /*!< PDCI_T::RXDATG0: CH0_RXDAT Mask        */

#define PDCI_RXDATG0_CH1_RXDAT_Pos       (8)                                               /*!< PDCI_T::RXDATG0: CH1_RXDAT Position    */
#define PDCI_RXDATG0_CH1_RXDAT_Msk       (0xfful << PDCI_RXDATG0_CH1_RXDAT_Pos)            /*!< PDCI_T::RXDATG0: CH1_RXDAT Mask        */

#define PDCI_RXDATG0_CH2_RXDAT_Pos       (16)                                              /*!< PDCI_T::RXDATG0: CH2_RXDAT Position    */
#define PDCI_RXDATG0_CH2_RXDAT_Msk       (0xfful << PDCI_RXDATG0_CH2_RXDAT_Pos)            /*!< PDCI_T::RXDATG0: CH2_RXDAT Mask        */

#define PDCI_RXDATG0_CH3_RXDAT_Pos       (24)                                              /*!< PDCI_T::RXDATG0: CH3_RXDAT Position    */
#define PDCI_RXDATG0_CH3_RXDAT_Msk       (0xfful << PDCI_RXDATG0_CH3_RXDAT_Pos)            /*!< PDCI_T::RXDATG0: CH3_RXDAT Mask        */

#define PDCI_RXDATG1_CH4_RXDAT_Pos       (0)                                               /*!< PDCI_T::RXDATG1: CH4_RXDAT Position    */
#define PDCI_RXDATG1_CH4_RXDAT_Msk       (0xfful << PDCI_RXDATG1_CH4_RXDAT_Pos)            /*!< PDCI_T::RXDATG1: CH4_RXDAT Mask        */

#define PDCI_RXDATG1_CH5_RXDAT_Pos       (8)                                               /*!< PDCI_T::RXDATG1: CH5_RXDAT Position    */
#define PDCI_RXDATG1_CH5_RXDAT_Msk       (0xfful << PDCI_RXDATG1_CH5_RXDAT_Pos)            /*!< PDCI_T::RXDATG1: CH5_RXDAT Mask        */

#define PDCI_RXDATG1_CH6_RXDAT_Pos       (16)                                              /*!< PDCI_T::RXDATG1: CH6_RXDAT Position    */
#define PDCI_RXDATG1_CH6_RXDAT_Msk       (0xfful << PDCI_RXDATG1_CH6_RXDAT_Pos)            /*!< PDCI_T::RXDATG1: CH6_RXDAT Mask        */

#define PDCI_RXDATG1_CH7_RXDAT_Pos       (24)                                              /*!< PDCI_T::RXDATG1: CH7_RXDAT Position    */
#define PDCI_RXDATG1_CH7_RXDAT_Msk       (0xfful << PDCI_RXDATG1_CH7_RXDAT_Pos)            /*!< PDCI_T::RXDATG1: CH7_RXDAT Mask        */

#define PDCI_RXDATG2_CH8_RXDAT_Pos       (0)                                               /*!< PDCI_T::RXDATG2: CH8_RXDAT Position    */
#define PDCI_RXDATG2_CH8_RXDAT_Msk       (0xfful << PDCI_RXDATG2_CH8_RXDAT_Pos)            /*!< PDCI_T::RXDATG2: CH8_RXDAT Mask        */

#define PDCI_RXDATG2_CH9_RXDAT_Pos       (8)                                               /*!< PDCI_T::RXDATG2: CH9_RXDAT Position    */
#define PDCI_RXDATG2_CH9_RXDAT_Msk       (0xfful << PDCI_RXDATG2_CH9_RXDAT_Pos)            /*!< PDCI_T::RXDATG2: CH9_RXDAT Mask        */

#define PDCI_RXDATG2_CH10_RXDAT_Pos      (16)                                              /*!< PDCI_T::RXDATG2: CH10_RXDAT Position   */
#define PDCI_RXDATG2_CH10_RXDAT_Msk      (0xfful << PDCI_RXDATG2_CH10_RXDAT_Pos)           /*!< PDCI_T::RXDATG2: CH10_RXDAT Mask       */

#define PDCI_RXDATG2_CH11_RXDAT_Pos      (24)                                              /*!< PDCI_T::RXDATG2: CH11_RXDAT Position   */
#define PDCI_RXDATG2_CH11_RXDAT_Msk      (0xfful << PDCI_RXDATG2_CH11_RXDAT_Pos)           /*!< PDCI_T::RXDATG2: CH11_RXDAT Mask       */

#define PDCI_RXDATG3_CH12_RXDAT_Pos      (0)                                               /*!< PDCI_T::RXDATG3: CH12_RXDAT Position   */
#define PDCI_RXDATG3_CH12_RXDAT_Msk      (0xfful << PDCI_RXDATG3_CH12_RXDAT_Pos)           /*!< PDCI_T::RXDATG3: CH12_RXDAT Mask       */

#define PDCI_RXDATG3_CH13_RXDAT_Pos      (8)                                               /*!< PDCI_T::RXDATG3: CH13_RXDAT Position   */
#define PDCI_RXDATG3_CH13_RXDAT_Msk      (0xfful << PDCI_RXDATG3_CH13_RXDAT_Pos)           /*!< PDCI_T::RXDATG3: CH13_RXDAT Mask       */

#define PDCI_RXDATG3_CH14_RXDAT_Pos      (16)                                              /*!< PDCI_T::RXDATG3: CH14_RXDAT Position   */
#define PDCI_RXDATG3_CH14_RXDAT_Msk      (0xfful << PDCI_RXDATG3_CH14_RXDAT_Pos)           /*!< PDCI_T::RXDATG3: CH14_RXDAT Mask       */

#define PDCI_RXDATG3_CH15_RXDAT_Pos      (24)                                              /*!< PDCI_T::RXDATG3: CH15_RXDAT Position   */
#define PDCI_RXDATG3_CH15_RXDAT_Msk      (0xfful << PDCI_RXDATG3_CH15_RXDAT_Pos)           /*!< PDCI_T::RXDATG3: CH15_RXDAT Mask       */

#define PDCI_RXDATG4_CH16_RXDAT_Pos      (0)                                               /*!< PDCI_T::RXDATG4: CH16_RXDAT Position   */
#define PDCI_RXDATG4_CH16_RXDAT_Msk      (0xfful << PDCI_RXDATG4_CH16_RXDAT_Pos)           /*!< PDCI_T::RXDATG4: CH16_RXDAT Mask       */

#define PDCI_RXDATG4_CH17_RXDAT_Pos      (8)                                               /*!< PDCI_T::RXDATG4: CH17_RXDAT Position   */
#define PDCI_RXDATG4_CH17_RXDAT_Msk      (0xfful << PDCI_RXDATG4_CH17_RXDAT_Pos)           /*!< PDCI_T::RXDATG4: CH17_RXDAT Mask       */

#define PDCI_RXDATG4_CH18_RXDAT_Pos      (16)                                              /*!< PDCI_T::RXDATG4: CH18_RXDAT Position   */
#define PDCI_RXDATG4_CH18_RXDAT_Msk      (0xfful << PDCI_RXDATG4_CH18_RXDAT_Pos)           /*!< PDCI_T::RXDATG4: CH18_RXDAT Mask       */

#define PDCI_RXDATG4_CH19_RXDAT_Pos      (24)                                              /*!< PDCI_T::RXDATG4: CH19_RXDAT Position   */
#define PDCI_RXDATG4_CH19_RXDAT_Msk      (0xfful << PDCI_RXDATG4_CH19_RXDAT_Pos)           /*!< PDCI_T::RXDATG4: CH19_RXDAT Mask       */

#define PDCI_RXDATG5_CH20_RXDAT_Pos      (0)                                               /*!< PDCI_T::RXDATG5: CH20_RXDAT Position   */
#define PDCI_RXDATG5_CH20_RXDAT_Msk      (0xfful << PDCI_RXDATG5_CH20_RXDAT_Pos)           /*!< PDCI_T::RXDATG5: CH20_RXDAT Mask       */

#define PDCI_RXDATG5_CH21_RXDAT_Pos      (8)                                               /*!< PDCI_T::RXDATG5: CH21_RXDAT Position   */
#define PDCI_RXDATG5_CH21_RXDAT_Msk      (0xfful << PDCI_RXDATG5_CH21_RXDAT_Pos)           /*!< PDCI_T::RXDATG5: CH21_RXDAT Mask       */

#define PDCI_RXDATG5_CH22_RXDAT_Pos      (16)                                              /*!< PDCI_T::RXDATG5: CH22_RXDAT Position   */
#define PDCI_RXDATG5_CH22_RXDAT_Msk      (0xfful << PDCI_RXDATG5_CH22_RXDAT_Pos)           /*!< PDCI_T::RXDATG5: CH22_RXDAT Mask       */

#define PDCI_RXDATG5_CH23_RXDAT_Pos      (24)                                              /*!< PDCI_T::RXDATG5: CH23_RXDAT Position   */
#define PDCI_RXDATG5_CH23_RXDAT_Msk      (0xfful << PDCI_RXDATG5_CH23_RXDAT_Pos)           /*!< PDCI_T::RXDATG5: CH23_RXDAT Mask       */

#define PDCI_RXCRCSTS0_RX0CRCE_Pos       (0)                                               /*!< PDCI_T::RXCRCSTS0: RX0CRCE Position    */
#define PDCI_RXCRCSTS0_RX0CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX0CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX0CRCE Mask        */

#define PDCI_RXCRCSTS0_RX1CRCE_Pos       (1)                                               /*!< PDCI_T::RXCRCSTS0: RX1CRCE Position    */
#define PDCI_RXCRCSTS0_RX1CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX1CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX1CRCE Mask        */

#define PDCI_RXCRCSTS0_RX2CRCE_Pos       (2)                                               /*!< PDCI_T::RXCRCSTS0: RX2CRCE Position    */
#define PDCI_RXCRCSTS0_RX2CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX2CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX2CRCE Mask        */

#define PDCI_RXCRCSTS0_RX3CRCE_Pos       (3)                                               /*!< PDCI_T::RXCRCSTS0: RX3CRCE Position    */
#define PDCI_RXCRCSTS0_RX3CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX3CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX3CRCE Mask        */

#define PDCI_RXCRCSTS0_RX4CRCE_Pos       (4)                                               /*!< PDCI_T::RXCRCSTS0: RX4CRCE Position    */
#define PDCI_RXCRCSTS0_RX4CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX4CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX4CRCE Mask        */

#define PDCI_RXCRCSTS0_RX5CRCE_Pos       (5)                                               /*!< PDCI_T::RXCRCSTS0: RX5CRCE Position    */
#define PDCI_RXCRCSTS0_RX5CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX5CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX5CRCE Mask        */

#define PDCI_RXCRCSTS0_RX6CRCE_Pos       (6)                                               /*!< PDCI_T::RXCRCSTS0: RX6CRCE Position    */
#define PDCI_RXCRCSTS0_RX6CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX6CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX6CRCE Mask        */

#define PDCI_RXCRCSTS0_RX7CRCE_Pos       (7)                                               /*!< PDCI_T::RXCRCSTS0: RX7CRCE Position    */
#define PDCI_RXCRCSTS0_RX7CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX7CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX7CRCE Mask        */

#define PDCI_RXCRCSTS0_RX8CRCE_Pos       (8)                                               /*!< PDCI_T::RXCRCSTS0: RX8CRCE Position    */
#define PDCI_RXCRCSTS0_RX8CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX8CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX8CRCE Mask        */

#define PDCI_RXCRCSTS0_RX9CRCE_Pos       (9)                                               /*!< PDCI_T::RXCRCSTS0: RX9CRCE Position    */
#define PDCI_RXCRCSTS0_RX9CRCE_Msk       (0x1ul << PDCI_RXCRCSTS0_RX9CRCE_Pos)             /*!< PDCI_T::RXCRCSTS0: RX9CRCE Mask        */

#define PDCI_RXCRCSTS0_RX10CRCE_Pos      (10)                                              /*!< PDCI_T::RXCRCSTS0: RX10CRCE Position   */
#define PDCI_RXCRCSTS0_RX10CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX10CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX10CRCE Mask       */

#define PDCI_RXCRCSTS0_RX11CRCE_Pos      (11)                                              /*!< PDCI_T::RXCRCSTS0: RX11CRCE Position   */
#define PDCI_RXCRCSTS0_RX11CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX11CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX11CRCE Mask       */

#define PDCI_RXCRCSTS0_RX12CRCE_Pos      (12)                                              /*!< PDCI_T::RXCRCSTS0: RX12CRCE Position   */
#define PDCI_RXCRCSTS0_RX12CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX12CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX12CRCE Mask       */

#define PDCI_RXCRCSTS0_RX13CRCE_Pos      (13)                                              /*!< PDCI_T::RXCRCSTS0: RX13CRCE Position   */
#define PDCI_RXCRCSTS0_RX13CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX13CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX13CRCE Mask       */

#define PDCI_RXCRCSTS0_RX14CRCE_Pos      (14)                                              /*!< PDCI_T::RXCRCSTS0: RX14CRCE Position   */
#define PDCI_RXCRCSTS0_RX14CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX14CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX14CRCE Mask       */

#define PDCI_RXCRCSTS0_RX15CRCE_Pos      (15)                                              /*!< PDCI_T::RXCRCSTS0: RX15CRCE Position   */
#define PDCI_RXCRCSTS0_RX15CRCE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX15CRCE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX15CRCE Mask       */

#define PDCI_RXCRCSTS0_RX0CRCAE_Pos      (16)                                              /*!< PDCI_T::RXCRCSTS0: RX0CRCAE Position   */
#define PDCI_RXCRCSTS0_RX0CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX0CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX0CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX1CRCAE_Pos      (17)                                              /*!< PDCI_T::RXCRCSTS0: RX1CRCAE Position   */
#define PDCI_RXCRCSTS0_RX1CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX1CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX1CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX2CRCAE_Pos      (18)                                              /*!< PDCI_T::RXCRCSTS0: RX2CRCAE Position   */
#define PDCI_RXCRCSTS0_RX2CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX2CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX2CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX3CRCAE_Pos      (19)                                              /*!< PDCI_T::RXCRCSTS0: RX3CRCAE Position   */
#define PDCI_RXCRCSTS0_RX3CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX3CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX3CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX4CRCAE_Pos      (20)                                              /*!< PDCI_T::RXCRCSTS0: RX4CRCAE Position   */
#define PDCI_RXCRCSTS0_RX4CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX4CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX4CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX5CRCAE_Pos      (21)                                              /*!< PDCI_T::RXCRCSTS0: RX5CRCAE Position   */
#define PDCI_RXCRCSTS0_RX5CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX5CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX5CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX6CRCAE_Pos      (22)                                              /*!< PDCI_T::RXCRCSTS0: RX6CRCAE Position   */
#define PDCI_RXCRCSTS0_RX6CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX6CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX6CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX7CRCAE_Pos      (23)                                              /*!< PDCI_T::RXCRCSTS0: RX7CRCAE Position   */
#define PDCI_RXCRCSTS0_RX7CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX7CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX7CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX8CRCAE_Pos      (24)                                              /*!< PDCI_T::RXCRCSTS0: RX8CRCAE Position   */
#define PDCI_RXCRCSTS0_RX8CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX8CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX8CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX9CRCAE_Pos      (25)                                              /*!< PDCI_T::RXCRCSTS0: RX9CRCAE Position   */
#define PDCI_RXCRCSTS0_RX9CRCAE_Msk      (0x1ul << PDCI_RXCRCSTS0_RX9CRCAE_Pos)            /*!< PDCI_T::RXCRCSTS0: RX9CRCAE Mask       */

#define PDCI_RXCRCSTS0_RX10CRCAE_Pos     (26)                                              /*!< PDCI_T::RXCRCSTS0: RX10CRCAE Position  */
#define PDCI_RXCRCSTS0_RX10CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX10CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX10CRCAE Mask      */

#define PDCI_RXCRCSTS0_RX11CRCAE_Pos     (27)                                              /*!< PDCI_T::RXCRCSTS0: RX11CRCAE Position  */
#define PDCI_RXCRCSTS0_RX11CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX11CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX11CRCAE Mask      */

#define PDCI_RXCRCSTS0_RX12CRCAE_Pos     (28)                                              /*!< PDCI_T::RXCRCSTS0: RX12CRCAE Position  */
#define PDCI_RXCRCSTS0_RX12CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX12CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX12CRCAE Mask      */

#define PDCI_RXCRCSTS0_RX13CRCAE_Pos     (29)                                              /*!< PDCI_T::RXCRCSTS0: RX13CRCAE Position  */
#define PDCI_RXCRCSTS0_RX13CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX13CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX13CRCAE Mask      */

#define PDCI_RXCRCSTS0_RX14CRCAE_Pos     (30)                                              /*!< PDCI_T::RXCRCSTS0: RX14CRCAE Position  */
#define PDCI_RXCRCSTS0_RX14CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX14CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX14CRCAE Mask      */

#define PDCI_RXCRCSTS0_RX15CRCAE_Pos     (31)                                              /*!< PDCI_T::RXCRCSTS0: RX15CRCAE Position  */
#define PDCI_RXCRCSTS0_RX15CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS0_RX15CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS0: RX15CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX16CRCE_Pos      (0)                                               /*!< PDCI_T::RXCRCSTS1: RX16CRCE Position   */
#define PDCI_RXCRCSTS1_RX16CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX16CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX16CRCE Mask       */

#define PDCI_RXCRCSTS1_RX17CRCE_Pos      (1)                                               /*!< PDCI_T::RXCRCSTS1: RX17CRCE Position   */
#define PDCI_RXCRCSTS1_RX17CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX17CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX17CRCE Mask       */

#define PDCI_RXCRCSTS1_RX18CRCE_Pos      (2)                                               /*!< PDCI_T::RXCRCSTS1: RX18CRCE Position   */
#define PDCI_RXCRCSTS1_RX18CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX18CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX18CRCE Mask       */

#define PDCI_RXCRCSTS1_RX19CRCE_Pos      (3)                                               /*!< PDCI_T::RXCRCSTS1: RX19CRCE Position   */
#define PDCI_RXCRCSTS1_RX19CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX19CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX19CRCE Mask       */

#define PDCI_RXCRCSTS1_RX20CRCE_Pos      (4)                                               /*!< PDCI_T::RXCRCSTS1: RX20CRCE Position   */
#define PDCI_RXCRCSTS1_RX20CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX20CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX20CRCE Mask       */

#define PDCI_RXCRCSTS1_RX21CRCE_Pos      (5)                                               /*!< PDCI_T::RXCRCSTS1: RX21CRCE Position   */
#define PDCI_RXCRCSTS1_RX21CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX21CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX21CRCE Mask       */

#define PDCI_RXCRCSTS1_RX22CRCE_Pos      (6)                                               /*!< PDCI_T::RXCRCSTS1: RX22CRCE Position   */
#define PDCI_RXCRCSTS1_RX22CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX22CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX22CRCE Mask       */

#define PDCI_RXCRCSTS1_RX23CRCE_Pos      (7)                                               /*!< PDCI_T::RXCRCSTS1: RX23CRCE Position   */
#define PDCI_RXCRCSTS1_RX23CRCE_Msk      (0x1ul << PDCI_RXCRCSTS1_RX23CRCE_Pos)            /*!< PDCI_T::RXCRCSTS1: RX23CRCE Mask       */

#define PDCI_RXCRCSTS1_RX16CRCAE_Pos     (16)                                              /*!< PDCI_T::RXCRCSTS1: RX16CRCAE Position  */
#define PDCI_RXCRCSTS1_RX16CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX16CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX16CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX17CRCAE_Pos     (17)                                              /*!< PDCI_T::RXCRCSTS1: RX17CRCAE Position  */
#define PDCI_RXCRCSTS1_RX17CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX17CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX17CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX18CRCAE_Pos     (18)                                              /*!< PDCI_T::RXCRCSTS1: RX18CRCAE Position  */
#define PDCI_RXCRCSTS1_RX18CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX18CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX18CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX19CRCAE_Pos     (19)                                              /*!< PDCI_T::RXCRCSTS1: RX19CRCAE Position  */
#define PDCI_RXCRCSTS1_RX19CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX19CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX19CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX20CRCAE_Pos     (20)                                              /*!< PDCI_T::RXCRCSTS1: RX20CRCAE Position  */
#define PDCI_RXCRCSTS1_RX20CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX20CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX20CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX21CRCAE_Pos     (21)                                              /*!< PDCI_T::RXCRCSTS1: RX21CRCAE Position  */
#define PDCI_RXCRCSTS1_RX21CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX21CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX21CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX22CRCAE_Pos     (22)                                              /*!< PDCI_T::RXCRCSTS1: RX22CRCAE Position  */
#define PDCI_RXCRCSTS1_RX22CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX22CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX22CRCAE Mask      */

#define PDCI_RXCRCSTS1_RX23CRCAE_Pos     (23)                                              /*!< PDCI_T::RXCRCSTS1: RX23CRCAE Position  */
#define PDCI_RXCRCSTS1_RX23CRCAE_Msk     (0x1ul << PDCI_RXCRCSTS1_RX23CRCAE_Pos)           /*!< PDCI_T::RXCRCSTS1: RX23CRCAE Mask      */

/**@}*/ /* PDCI_CONST */
/**@}*/ /* end of PDCI register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __PDCI_REG_H__ */
