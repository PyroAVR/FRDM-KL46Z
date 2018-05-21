//**********************************************************************
//Freescale MKL46Z256xxx4 device values and configuration code
//* .equ Various, ATES for memory map
//[1] Freescale Semiconductor, <B>Kinetis L Peripheral Module Quick
//    Reference</B>, KLQRUG, Rev. 0, 9/2012.
//[2] ARM, <B>Application Note 48 Scatter Loading</B>, ARM DAI 0048A,
//    Jan. 1998
//[3] Freescale Semiconductor, <B>KL46 Sub-Family Reference Manual</B>,
//    KL46P121M48SF4RM, Rev. 3, 7/2013.
//[4] Freescale Semiconductor, MKL46Z4.h, rev. 2.2, 4/12/2013
//---------------------------------------------------------------
//Author:  R. W. Melton
//Date:  September 25, 2017
//***************************************************************
//.equ , ates
//Standard data masks
.syntax unified
.equ BYTE_MASK, 0xFF
.equ NIBBLE_MASK, 0x0F
//Standard data sizes (in bits)
.equ BYTE_BITS, 8
.equ NIBBLE_BITS, 4
//Architecture data sizes (in bytes)
.equ WORD_SIZE, 4  //Cortex-M0+
.equ HALFWORD_SIZE, 2  //Cortex-M0+
//Architecture data masks
.equ HALFWORD_MASK, 0xFFFF
//Return                 
.equ RET_ADDR_T_MASK, 1  //Bit 0 of ret. addr. must be
                          //set for BX, BLX, or POP
                          //mask in thumb
//---------------------------------------------------------------
//Vectors
.equ VECTOR_TABLE_SIZE, 0x000000C0  //KL46
.equ VECTOR_SIZE, 4           //Bytes per vector
//---------------------------------------------------------------
//CPU PSR:  Program status register
//Combined APSR, EPSR, and IPSR
//----------------------------------------------------------
//APSR:  Application Program Status Register
//31  :N=negative flag
//30  :Z=zero flag
//29  :C=carry flag
//28  :V=overflow flag
//27-0:(reserved)
.equ APSR_MASK, 0xF0000000
.equ APSR_SHIFT, 28
.equ APSR_N_MASK, 0x80000000
.equ APSR_N_SHIFT, 31
.equ APSR_Z_MASK, 0x40000000
.equ APSR_Z_SHIFT, 30
.equ APSR_C_MASK, 0x20000000
.equ APSR_C_SHIFT, 29
.equ APSR_V_MASK, 0x10000000
.equ APSR_V_SHIFT, 28
//----------------------------------------------------------
//EPSR
//31-25:(reserved)
//   24:T=thumb state bit
//23- 0:(reserved)
.equ EPSR_MASK, 0x01000000
.equ EPSR_SHIFT, 24
.equ EPSR_T_MASK, 0x01000000
.equ EPSR_T_SHIFT, 24
//----------------------------------------------------------
//IPSR
//31-6:(reserved)
// 5-0:Exception number=number of current exception
//      0=thread mode
//      1:(reserved)
//      2=NMI
//      3=hard fault
//      4-10:(reserved)
//     11=SVCall
//     12-13:(reserved)
//     14=PendSV
//     15=SysTick
//     16=IRQ0
//     16-47:IRQ(Exception number - 16)
//     47=IRQ31
//     48-63:(reserved)
.equ IPSR_MASK, 0x0000003F
.equ IPSR_SHIFT, 0
.equ IPSR_EXCEPTION_MASK, 0x0000003F
.equ IPSR_EXCEPTION_SHIFT, 0
//----------------------------------------------------------
.equ PSR_N_MASK, APSR_N_MASK
.equ PSR_N_SHIFT, APSR_N_SHIFT
.equ PSR_Z_MASK, APSR_Z_MASK
.equ PSR_Z_SHIFT, APSR_Z_SHIFT
.equ PSR_C_MASK, APSR_C_MASK
.equ PSR_C_SHIFT, APSR_C_SHIFT
.equ PSR_V_MASK, APSR_V_MASK
.equ PSR_V_SHIFT, APSR_V_SHIFT
.equ PSR_T_MASK, EPSR_T_MASK
.equ PSR_T_SHIFT, EPSR_T_SHIFT
.equ PSR_EXCEPTION_MASK, IPSR_EXCEPTION_MASK
.equ PSR_EXCEPTION_SHIFT, IPSR_EXCEPTION_SHIFT
//---------------------------------------------------------------
//Cortex-M0+ Core
.equ __CM0PLUS_REV, 0x0000  //Core revision r0p0
.equ __MPU_PRESENT, 0       //Whether MPU is present
.equ __NVIC_PRIO_BITS, 2       //Number of NVIC priority bits
.equ __Vendor_SysTickConfig, 0       //Whether vendor-specific 
                                     //SysTickConfig is defined
.equ __VTOR_PRESENT, 1       //Whether VTOR is present
//---------------------------------------------------------------
//Interrupt numbers
//Core interrupts
.equ NonMaskableInt_IRQn, -14  //Non-maskable interrupt (NMI)
.equ HardFault_IRQn, -13  //Hard fault interrupt
.equ SVCall_IRQn, -5   //Supervisor call interrupt (SVCall)
.equ PendSV_IRQn, -2   //Pendable request for system service interrupt
                               //(PendableSrvReq)
.equ SysTick_IRQn, -1   //System tick timer interrupt (SysTick)
//--------------------------
//Device specific interrupts
.equ DMA0_IRQn, 0   //DMA channel 0 transfer complete/error interrupt
.equ DMA1_IRQn, 1   //DMA channel 1 transfer complete/error interrupt
.equ DMA2_IRQn, 2   //DMA channel 2 transfer complete/error interrupt
.equ DMA3_IRQn, 3   //DMA channel 3 transfer complete/error interrupt
.equ Reserved20_IRQn, 4   //Reserved interrupt 20
.equ FTFA_IRQn, 5   //FTFA command complete/read collision interrupt
.equ LVD_LVW_IRQn, 6   //Low-voltage detect, low-voltage warning interrupt
.equ LLW_IRQn, 7   //Low leakage wakeup interrupt
.equ I2C0_IRQn, 8   //I2C0 interrupt
.equ I2C1_IRQn, 9   //I2C1 interrupt
.equ SPI0_IRQn, 10  //SPI0 interrupt
.equ SPI1_IRQn, 11  //SPI1 interrupt
.equ UART0_IRQn, 12  //UART0 status/error interrupt
.equ UART1_IRQn, 13  //UART1 status/error interrupt
.equ UART2_IRQn, 14  //UART2 status/error interrupt
.equ ADC0_IRQn, 15  //ADC0 interrupt
.equ CMP0_IRQn, 16  //CMP0 interrupt
.equ TPM0_IRQn, 17  //TPM0 fault, overflow, and channels interrupt
.equ TPM1_IRQn, 18  //TPM1 fault, overflow, and channels interrupt
.equ TPM2_IRQn, 19  //TPM2 fault, overflow, and channels interrupt
.equ RTC_IRQn, 20  //RTC alarm interrupt
.equ RTC_Seconds_IRQn, 21  //RTC seconds interrupt
.equ PIT_IRQn, 22  //PIT interrupt
.equ I2S0_IRQn, 23  //I2S0 interrupt
.equ USB0_IRQn, 24  //USB OTG interrupt
.equ DAC0_IRQn, 25  //DAC0 interrupt
.equ TSI0_IRQn, 26  //TSI0 interrupt
.equ MCG_IRQn, 27  //MCG interrupt
.equ LPTimer_IRQn, 28  //LPTMR0 interrupt
.equ LCD_IRQn, 29  //SLCD interrupt
.equ PORTA_IRQn, 30  //Port A pin detect interrupt
.equ PORTC_PORTD_IRQn, 31  //Port C and Port D pin detectinterrupt
//---------------------------------------------------------------
//Memory map major version
//(Memory maps with equal major version number are compatible)
.equ MCU_MEM_MAP_VERSION, 0x0200
//Memory map minor version
.equ MCU_MEM_MAP_VERSION_MINOR, 0x0002
//---------------------------------------------------------------
//ADC
.equ ADC0_BASE, 0x4003B000
.equ ADC_SC1A_OFFSET, 0x00
.equ ADC_SC1B_OFFSET, 0x04
.equ ADC_CFG1_OFFSET, 0x08
.equ ADC_CFG2_OFFSET, 0x0C
.equ ADC_RA_OFFSET, 0x10
.equ ADC_RB_OFFSET, 0x14
.equ ADC_CV1_OFFSET, 0x18
.equ ADC_CV2_OFFSET, 0x1C
.equ ADC_SC2_OFFSET, 0x20
.equ ADC_SC3_OFFSET, 0x24
.equ ADC_OFS_OFFSET, 0x28
.equ ADC_PG_OFFSET, 0x2C
.equ ADC_MG_OFFSET, 0x30
.equ ADC_CLPD_OFFSET, 0x34
.equ ADC_CLPS_OFFSET, 0x38
.equ ADC_CLP4_OFFSET, 0x3C
.equ ADC_CLP3_OFFSET, 0x40
.equ ADC_CLP2_OFFSET, 0x44
.equ ADC_CLP1_OFFSET, 0x48
.equ ADC_CLP0_OFFSET, 0x4C
.equ ADC_CLMD_OFFSET, 0x54
.equ ADC_CLMS_OFFSET, 0x58
.equ ADC_CLM4_OFFSET, 0x5C
.equ ADC_CLM3_OFFSET, 0x60
.equ ADC_CLM2_OFFSET, 0x64
.equ ADC_CLM1_OFFSET, 0x68
.equ ADC_CLM0_OFFSET, 0x6C
.equ ADC0_CFG1, ( ADC0_BASE + ADC_CFG1_OFFSET)
.equ ADC0_CFG2, ( ADC0_BASE + ADC_CFG2_OFFSET)
.equ ADC0_CLMD, ( ADC0_BASE + ADC_CLMD_OFFSET)
.equ ADC0_CLMS, ( ADC0_BASE + ADC_CLMS_OFFSET)
.equ ADC0_CLM0, ( ADC0_BASE + ADC_CLM0_OFFSET)
.equ ADC0_CLM1, ( ADC0_BASE + ADC_CLM1_OFFSET)
.equ ADC0_CLM2, ( ADC0_BASE + ADC_CLM2_OFFSET)
.equ ADC0_CLM3, ( ADC0_BASE + ADC_CLM3_OFFSET)
.equ ADC0_CLM4, ( ADC0_BASE + ADC_CLM4_OFFSET)
.equ ADC0_CLPD, ( ADC0_BASE + ADC_CLPD_OFFSET)
.equ ADC0_CLPS, ( ADC0_BASE + ADC_CLPS_OFFSET)
.equ ADC0_CLP0, ( ADC0_BASE + ADC_CLP0_OFFSET)
.equ ADC0_CLP1, ( ADC0_BASE + ADC_CLP1_OFFSET)
.equ ADC0_CLP2, ( ADC0_BASE + ADC_CLP2_OFFSET)
.equ ADC0_CLP3, ( ADC0_BASE + ADC_CLP3_OFFSET)
.equ ADC0_CLP4, ( ADC0_BASE + ADC_CLP4_OFFSET)
.equ ADC0_CV1, ( ADC0_BASE + ADC_CV1_OFFSET)
.equ ADC0_CV2, ( ADC0_BASE + ADC_CV2_OFFSET)
.equ ADC0_MG, ( ADC0_BASE + ADC_MG_OFFSET) 
.equ ADC0_OFS, ( ADC0_BASE + ADC_OFS_OFFSET)
.equ ADC0_PG, ( ADC0_BASE + ADC_PG_OFFSET) 
.equ ADC0_RA, ( ADC0_BASE + ADC_RA_OFFSET)  
.equ ADC0_RB, ( ADC0_BASE + ADC_RB_OFFSET)  
.equ ADC0_SC1A, ( ADC0_BASE + ADC_SC1A_OFFSET)
.equ ADC0_SC1B, ( ADC0_BASE + ADC_SC1B_OFFSET)
.equ ADC0_SC2, ( ADC0_BASE + ADC_SC2_OFFSET)
.equ ADC0_SC3, ( ADC0_BASE + ADC_SC3_OFFSET)
//---------------------------------------------------------------
//ADC_CFG1:  ADC configuration register 1
//31-8:(reserved):read-only:0
//   7:ADLPC=ADC low-power configuration
// 6-5:ADIV=ADC clock divide select
//     Internal ADC clock = input clock / 2^ADIV
//   4:ADLSMP=ADC long sample time configuration
//            0=short
//            1=long
// 3-2:MODE=conversion mode selection
//          00=(DIFF'):single-ended 8-bit conversion
//             (DIFF):differential 9-bit 2's complement conversion
//          01=(DIFF'):single-ended 12-bit conversion
//             (DIFF):differential 13-bit 2's complement conversion
//          10=(DIFF'):single-ended 10-bit conversion
//             (DIFF):differential 11-bit 2's complement conversion
//          11=(DIFF'):single-ended 16-bit conversion
//             (DIFF):differential 16-bit 2's complement conversion
// 1-0:ADICLK=ADC input clock select
//          00=bus clock
//          01=bus clock / 2
//          10=alternate clock (ALTCLK)
//          11=asynchronous clock (ADACK)
.equ ADC_CFG1_ADLPC_MASK, 0x80
.equ ADC_CFG1_ADLPC_SHIFT, 7
.equ ADC_CFG1_ADIV_MASK, 0x60
.equ ADC_CFG1_ADIV_SHIFT, 5
.equ ADC_CFG1_ADLSMP_MASK, 0x10
.equ ADC_CFG1_ADLSMP_SHIFT, 4
.equ ADC_CFG1_MODE_MASK, 0x0C
.equ ADC_CFG1_MODE_SHIFT, 2
.equ ADC_CFG1_ADICLK_MASK, 0x03
.equ ADC_CFG1_ADICLK_SHIFT, 0
//---------------------------------------------------------------
//ADC_CFG2:  ADC configuration register 2
//31-8:(reserved):read-only:0
// 7-5:(reserved):read-only:0
//   4:MUXSEL=ADC mux select
//            0=ADxxA channels are selected
//            1=ADxxB channels are selected
//   3:ADACKEN=ADC asynchronous clock output enable
//             0=asynchronous clock determined by ACD0_CFG1.ADICLK 
//             1=asynchronous clock enabled
//   2:ADHSC=ADC high-speed configuration
//           0=normal conversion
//           1=high-speed conversion (only 2 additional ADK cycles)
// 1-0:ADLSTS=ADC long sample time select (ADK cycles)
//          00=default longest sample time:  
//             24 total ADK cycles (20 extra)
//          01=16 total ADK cycles (12 extra)
//          10=10 total ADK cycles (6 extra)
//          11=6 total ADK cycles (2 extra)
.equ ADC_CFG2_MUXSEL_MASK, 0x10
.equ ADC_CFG2_MUXSEL_SHIFT, 4
.equ ADC_CFG2_ADACKEN_MASK, 0x08
.equ ADC_CFG2_ADACKEN_SHIFT, 3
.equ ADC_CFG2_ADHSC_MASK, 0x04
.equ ADC_CFG2_ADHSC_SHIFT, 2
.equ ADC_CFG2_ADLSTS_MASK, 0x03
.equ ADC_CFG2_ADLSTS_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLMD:  ADC minus-side general calibration value register D
//31-6:(reserved):read-only:0
// 5-0:CLMD=calibration value
.equ ADC_CLMD_MASK, 0x3F
.equ ADC_CLMD_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLMS:  ADC minus-side general calibration value register S
//31-6:(reserved):read-only:0
// 5-0:CLMS=calibration value
.equ ADC_CLMS_MASK, 0x3F
.equ ADC_CLMS_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLM0:  ADC minus-side general calibration value register 0
//31-6:(reserved):read-only:0
// 5-0:CLM0=calibration value
.equ ADC_CLM0_MASK, 0x3F
.equ ADC_CLM0_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLM1:  ADC minus-side general calibration value register 1
//31-7:(reserved):read-only:0
// 6-0:CLM1=calibration value
.equ ADC_CLM1_MASK, 0x7F
.equ ADC_CLM1_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLM2:  ADC minus-side general calibration value register 2
//31-8:(reserved):read-only:0
// 7-0:CLM2=calibration value
.equ ADC_CLM2_MASK, 0xFF
.equ ADC_CLM2_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLM3:  ADC minus-side general calibration value register 3
//31-9:(reserved):read-only:0
// 8-0:CLM3=calibration value
.equ ADC_CLM3_MASK, 0x1FF
.equ ADC_CLM3_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLM4:  ADC minus-side general calibration value register 4
//31-10:(reserved):read-only:0
// 9- 0:CLM4=calibration value
.equ ADC_CLM4_MASK, 0x3FF
.equ ADC_CLM4_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLPD:  ADC plus-side general calibration value register D
//31-6:(reserved):read-only:0
// 5-0:CLPD=calibration value
.equ ADC_CLPD_MASK, 0x3F
.equ ADC_CLPD_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLPS:  ADC plus-side general calibration value register S
//31-6:(reserved):read-only:0
// 5-0:CLPS=calibration value
.equ ADC_CLPS_MASK, 0x3F
.equ ADC_CLPS_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLP0:  ADC plus-side general calibration value register 0
//31-6:(reserved):read-only:0
// 5-0:CLP0=calibration value
.equ ADC_CLP0_MASK, 0x3F
.equ ADC_CLP0_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLP1:  ADC plus-side general calibration value register 1
//31-7:(reserved):read-only:0
// 6-0:CLP1=calibration value
.equ ADC_CLP1_MASK, 0x7F
.equ ADC_CLP1_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLP2:  ADC plus-side general calibration value register 2
//31-8:(reserved):read-only:0
// 7-0:CLP2=calibration value
.equ ADC_CLP2_MASK, 0xFF
.equ ADC_CLP2_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLP3:  ADC plus-side general calibration value register 3
//31-9:(reserved):read-only:0
// 8-0:CLP3=calibration value
.equ ADC_CLP3_MASK, 0x1FF
.equ ADC_CLP3_SHIFT, 0
//---------------------------------------------------------------
//ADC_CLP4:  ADC plus-side general calibration value register 4
//31-10:(reserved):read-only:0
// 9- 0:CLP4=calibration value
.equ ADC_CLP4_MASK, 0x3FF
.equ ADC_CLP4_SHIFT, 0
//---------------------------------------------------------------
//ADC_CVn:  ADC channel n compare value register
//CV1 used to compare result when ADC_SC2.ACFE=1
//CV2 used to compare result when ADC_SC2.ACREN=1
//31-16:(reserved):read-only:0
//15- 0:compare value (sign- or zero-extended if fewer than 16 bits)
.equ ADC_CV_MASK, 0xFFFF
.equ ADC_CV_SHIFT, 0
//---------------------------------------------------------------
//ADC_MG:  ADC minus-side gain register
//31-16:(reserved):read-only:0
//15- 0:MG=minus-side gain
.equ ADC_MG_MASK, 0xFFFF
.equ ADC_MG_SHIFT, 0
//---------------------------------------------------------------
//ADC_OFS:  ADC offset correction register
//31-16:(reserved):read-only:0
//15- 0:OFS=offset error correction value
.equ ADC_OFS_MASK, 0xFFFF
.equ ADC_OFS_SHIFT, 0
//---------------------------------------------------------------
//ADC_PG:  ADC plus-side gain register
//31-16:(reserved):read-only:0
//15- 0:PG=plus-side gain
.equ ADC_PG_MASK, 0xFFFF
.equ ADC_PG_SHIFT, 0
//---------------------------------------------------------------
//ADC_Rn:  ADC channel n data result register
//31-16:(reserved):read-only:0
//15- 0:data result (sign- or zero-extended if fewer than 16 bits)
.equ ADC_D_MASK, 0xFFFF
.equ ADC_D_SHIFT, 0
//---------------------------------------------------------------
//ADC_SC1n:  ADC channel n status and control register 1
//31-8:(reserved):read-only:0
//   7:COCO=conversion complete flag (read-only)
//   6:AIEN=ADC interrupt enabled
//   5:DIFF=differential mode enable
// 4-0:ADCH=ADC input channel select
//          00000=(DIFF'):DADP0//(DIFF):DAD0
//          00001=(DIFF'):DADP1//(DIFF):DAD1
//          00010=(DIFF'):DADP2//(DIFF):DAD2
//          00011=(DIFF'):DADP3//(DIFF):DAD3
//          00100=(DIFF'):AD4//(DIFF):(reserved)
//          00101=(DIFF'):AD5//(DIFF):(reserved)
//          00110=(DIFF'):AD6//(DIFF):(reserved)
//          00111=(DIFF'):AD7//(DIFF):(reserved)
//          01000=(DIFF'):AD8//(DIFF):(reserved)
//          01001=(DIFF'):AD9//(DIFF):(reserved)
//          01010=(DIFF'):AD10//(DIFF):(reserved)
//          01011=(DIFF'):AD11//(DIFF):(reserved)
//          01100=(DIFF'):AD12//(DIFF):(reserved)
//          01101=(DIFF'):AD13//(DIFF):(reserved)
//          01110=(DIFF'):AD14//(DIFF):(reserved)
//          01111=(DIFF'):AD15//(DIFF):(reserved)
//          10000=(DIFF'):AD16//(DIFF):(reserved)
//          10001=(DIFF'):AD17//(DIFF):(reserved)
//          10010=(DIFF'):AD18//(DIFF):(reserved)
//          10011=(DIFF'):AD19//(DIFF):(reserved)
//          10100=(DIFF'):AD20//(DIFF):(reserved)
//          10101=(DIFF'):AD21//(DIFF):(reserved)
//          10110=(DIFF'):AD22//(DIFF):(reserved)
//          10111=(DIFF'):AD23//(DIFF):(reserved)
//          11000 (reserved)
//          11001 (reserved)
//          11010=(DIFF'):temp sensor (single-ended)
//                (DIFF):temp sensor (differential)
//          11011=(DIFF'):bandgap (single-ended)
//                (DIFF):bandgap (differential)
//          11100 (reserved)
//          11101=(DIFF'):VREFSH (single-ended)
//                (DIFF):-VREFSH (differential)
//          11110=(DIFF'):VREFSL (single-ended)
//                (DIFF):(reserved)
//          11111=disabled
.equ ADC_COCO_MASK, 0x80
.equ ADC_COCO_SHIFT, 7
.equ ADC_AIEN_MASK, 0x40
.equ ADC_AIEN_SHIFT, 6
.equ ADC_DIFF_MASK, 0x20
.equ ADC_DIFF_SHIFT, 5
.equ ADC_ADCH_MASK, 0x1F
.equ ADC_ADCH_SHIFT, 0
//---------------------------------------------------------------
//ADC_SC2:  ADC status and control register 2
//31-8:(reserved):read-only:0
//   7:ADACT=ADC conversion active
//   6:ADTRG=ADC conversion trigger select
//           0=software trigger
//           1=hardware trigger
//   5:ACFE=ADC compare function enable
//   4:ACFGT=ADC compare function greater than enable
//           based on values in ADC_CV1 and ADC_CV2
//           0=configure less than threshold and non-inclusive range
//           1=configure greater than threshold and non-inclusive range
//   3:ACREN=ADC compare function range enable
//           0=disabled// only ADC_CV1 compared
//           1=enabled// both ADC_CV1 and ADC_CV2 compared
//   2:DMAEN=DMA enable
// 1-0:REFSEL=voltage reference selection
//            00=default:VREFH and VREFL
//            01=alterantive:VALTH and VALTL
//            10=(reserved)
//            11=(reserved)
.equ ADC_ADACT_MASK, 0x80
.equ ADC_ADACT_SHIFT, 7
.equ ADC_ADTRG_MASK, 0x40
.equ ADC_ADTRG_SHIFT, 6
.equ ADC_ACFE_MASK, 0x20
.equ ADC_ACFE_SHIFT, 5
.equ ADC_ACFGT_MASK, 0x10
.equ ADC_ACFGT_SHIFT, 4
.equ ADC_ACREN_MASK, 0x08
.equ ADC_ACREN_SHIFT, 3
.equ ADC_DMAEN_MASK, 0x04
.equ ADC_DMAEN_SHIFT, 2
.equ ADC_REFSEL_MASK, 0x03
.equ ADC_REFSEL_SHIFT, 0
//---------------------------------------------------------------
//ADC_SC3:  ADC status and control register 3
//31-8:(reserved):read-only:0
//   7:CAL=calibration
//         write:0=(no effect)
//               1=start calibration sequence
//         read:0=calibration sequence complete
//              1=calibration sequence in progress
//   6:CALF=calibration failed flag
// 5-4:(reserved):read-only:0
//   3:ADC=ADC continuous conversion enable (if ADC_SC3.AVGE = 1)
//   2:AVGE=hardware average enable
// 1-0:AVGS=hardware average select:  2^(2+AVGS) samples
.equ ADC_CAL_MASK, 0x80
.equ ADC_CAL_SHIFT, 7
.equ ADC_CALF_MASK, 0x40
.equ ADC_CALF_SHIFT, 6
.equ ADC_ADCO_MASK, 0x08
.equ ADC_ADCO_SHIFT, 3
.equ ADC_AVGE_MASK, 0x04
.equ ADC_AVGE_SHIFT, 2
.equ ADC_AVGS_MASK, 0x03
.equ ADC_AVGS_SHIFT, 0
//---------------------------------------------------------------
//CMP
.equ CMP0_BASE, 0x40073000
.equ CMP0_CR0_OFFSET, 0x00
.equ CMP0_CR1_OFFSET, 0x01
.equ CMP0_FPR_OFFSET, 0x02
.equ CMP0_SCR_OFFSET, 0x03
.equ CMP0_DACCR_OFFSET, 0x04
.equ CMP0_MUXCR_OFFSET, 0x05
.equ CMP0_CR0, (CMP0_BASE + CMP0_CR0_OFFSET)
.equ CMP0_CR1, (CMP0_BASE + CMP0_CR1_OFFSET)
.equ CMP0_FPR, (CMP0_BASE + CMP0_FPR_OFFSET)
.equ CMP0_SCR, (CMP0_BASE + CMP0_SCR_OFFSET)
.equ CMP0_DACCR, (CMP0_BASE + CMP0_DACCR_OFFSET)
.equ CMP0_MUXCR, (CMP0_BASE + CMP0_MUXCR_OFFSET)
//---------------------------------------------------------------
//CMP0_CR0:  CMP0 control register 0 (0x00)
//  7:(reserved):read-only:0
//6-4:FILTER_CNT=filter sample count (00)
//  3:(reserved):read-only:0
//  2:(reserved):read-only:0
//1-0:HYSTCTR=comparator hard block hysteresis control (00)
.equ CMP_CR0_HYSTCTR_MASK, 0x3
.equ CMP_CR0_HYSTCTR_SHIFT, 0
.equ CMP_CR0_FILTER_CNT_MASK, 0x70
.equ CMP_CR0_FILTER_CNT_SHIFT, 4
//---------------------------------------------------------------
//CMP0_CR0:  CMP0 control register 1 (0x00)
//7:SE=sample enable (0)
//6:WE=windowing enable (0)
//5:TRIGM=trigger mode enable (0)
//4:PMODE=power mode select (0)
//3:INV=comparator invert (0)
//2:COS=comparator output select (0)
//1:OPE=comparator output pin enable (0)
//0:EN=comparator module enable (0)
.equ CMP_CR1_EN_MASK, 0x1
.equ CMP_CR1_EN_SHIFT, 0
.equ CMP_CR1_OPE_MASK, 0x2
.equ CMP_CR1_OPE_SHIFT, 1
.equ CMP_CR1_COS_MASK, 0x4
.equ CMP_CR1_COS_SHIFT, 2
.equ CMP_CR1_INV_MASK, 0x8
.equ CMP_CR1_INV_SHIFT, 3
.equ CMP_CR1_PMODE_MASK, 0x10
.equ CMP_CR1_PMODE_SHIFT, 4
.equ CMP_CR1_TRIGM_MASK, 0x20
.equ CMP_CR1_TRIGM_SHIFT, 5
.equ CMP_CR1_WE_MASK, 0x40
.equ CMP_CR1_WE_SHIFT, 6
.equ CMP_CR1_SE_MASK, 0x80
.equ CMP_CR1_SE_SHIFT, 7
//---------------------------------------------------------------
//CMP0_FPR=CMP filter period register (0x00)
//7-0:FILT_PER=CMP filter period register (0x00)
.equ CMP_FPR_FILT_PER_MASK, 0xFF
.equ CMP_FPR_FILT_PER_SHIFT, 0
//---------------------------------------------------------------
//CMP0_SCR=CMP status and control register (0x00)
//7:(reserved):read-only:0
//6:DMAEN=DMA enable control (0)
//5:(reserved):read-only:0
//4:IER=comparator interrupt enable rising (0)
//3:IEF=comparator interrupt enable falling (0)
//2:CFR=analog comparator flag rising: w1c (0)
//1:CFF=analog comparator flag falling: w1c (0)
//0:COUT=analog comparator output (0)
.equ CMP_SCR_COUT_MASK, 0x1
.equ CMP_SCR_COUT_SHIFT, 0
.equ CMP_SCR_CFF_MASK, 0x2
.equ CMP_SCR_CFF_SHIFT, 1
.equ CMP_SCR_CFR_MASK, 0x4
.equ CMP_SCR_CFR_SHIFT, 2
.equ CMP_SCR_IEF_MASK, 0x8
.equ CMP_SCR_IEF_SHIFT, 3
.equ CMP_SCR_IER_MASK, 0x10
.equ CMP_SCR_IER_SHIFT, 4
.equ CMP_SCR_DMAEN_MASK, 0x40
.equ CMP_SCR_DMAEN_SHIFT, 6
//---------------------------------------------------------------
//CMP0_DACCR=DAC control register (0x00)
//  7:DACEN=DAC enable (0)
//  6:VRSEL=supply voltage reference source select (0)
//5-0:VOSEL=DAC output voltage select (00000)
//          DAC0 = (Vin / 64) x (VOSEL[5:0] + 1)
.equ CMP_DACCR_VOSEL_MASK, 0x3F
.equ CMP_DACCR_VOSEL_SHIFT, 0
.equ CMP_DACCR_VRSEL_MASK, 0x40
.equ CMP_DACCR_VRSEL_SHIFT, 6
.equ CMP_DACCR_DACEN_MASK, 0x80
.equ CMP_DACCR_DACEN_SHIFT, 7
//---------------------------------------------------------------
//CMP0_MUXCR=MUX control register (0x00)
//  7:PSTM=pass through mode enable (0)
//5-3:PSEL=plus input mux control (000)
//         selects IN[PSEL]
//2-0:MSEL=minus input mux control (000)
//         selects IN[MSEL]
.equ CMP_MUXCR_MSEL_MASK, 0x7
.equ CMP_MUXCR_MSEL_SHIFT, 0
.equ CMP_MUXCR_PSEL_MASK, 0x38
.equ CMP_MUXCR_PSEL_SHIFT, 3
.equ CMP_MUXCR_PSTM_MASK, 0x80
.equ CMP_MUXCR_PSTM_SHIFT, 7
//---------------------------------------------------------------
//DAC
.equ DAC0_BASE, 0x4003F000
.equ DAC0_DAT0L_OFFSET, 0x00
.equ DAC0_DAT0H_OFFSET, 0x01
.equ DAC0_DAT1L_OFFSET, 0x02
.equ DAC0_DAT1H_OFFSET, 0x03
.equ DAC0_SR_OFFSET, 0x20
.equ DAC0_C0_OFFSET, 0x21
.equ DAC0_C1_OFFSET, 0x22
.equ DAC0_C2_OFFSET, 0x23
.equ DAC0_DAT0L, (DAC0_BASE + DAC0_DAT0L_OFFSET)
.equ DAC0_DAT0H, (DAC0_BASE + DAC0_DAT0H_OFFSET)
.equ DAC0_DAT1L, (DAC0_BASE + DAC0_DAT1L_OFFSET)
.equ DAC0_DAT1H, (DAC0_BASE + DAC0_DAT1H_OFFSET)
.equ DAC0_SR, (DAC0_BASE + DAC0_SR_OFFSET)
.equ DAC0_C0, (DAC0_BASE + DAC0_C0_OFFSET)
.equ DAC0_C1, (DAC0_BASE + DAC0_C1_OFFSET)
.equ DAC0_C2, (DAC0_BASE + DAC0_C2_OFFSET)
//---------------------------------------------------------------
//DAC_DAT0H:  DAC data high register 0
//If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
//7-4:(reserved):read-only:0
//3-0:DATA1=DATA[11:8]
.equ DAC_DAT0H_MASK, 0x0F
.equ DAC_DAT0H_SHIFT, 0
//---------------------------------------------------------------
//DAC_DAT0L:  DAC data low register 0
//If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
//7-0:DATA0=DATA[7:0]
//---------------------------------------------------------------
//DAC_DAT1H:  DAC data high register 1
//If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
//7-4:(reserved):read-only:0
//3-0:DATA1=DATA[11:8]
.equ DAC_DAT1H_MASK, 0x0F
.equ DAC_DAT1H_SHIFT, 0
//---------------------------------------------------------------
//DAC_DAT1L:  DAC data low register 1
//If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
//7-0:DATA0=DATA[7:0]
//---------------------------------------------------------------
//DAC_C0:  DAC control register 0
//7:DACEN=DAC enable
//6:DACRFS=DAC reference select
//         0:DACREF_1=VREFH
//         1:DACREF_2=VDDA (best for ADC operation)
//5:DACTRGSEL=DAC trigger select
//            0:HW
//            1:SW
//4:DACSWTRG=DAC software trigger
//           active-high write-only field that reads 0
//           DACBFEN & DACTRGSEL:  writing 1 advances buffer pointer
//3:LPEN=DAC low power control
//       0:high-power mode
//       1:low-power mode
//2:(reserved):read-only:0
//1:DACBTIEN=DAC buffer read pointer top flag interrupt enable
//0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable
.equ DAC_C0_DACEN_MASK, 0x80
.equ DAC_C0_DACEN_SHIFT, 7
.equ DAC_C0_DACRFS_MASK, 0x40
.equ DAC_C0_DACRFS_SHIFT, 6
.equ DAC_C0_DACTRGSEL_MASK, 0x20
.equ DAC_C0_DACTRGSEL_SHIFT, 5
.equ DAC_C0_DACSWTRG_MASK, 0x10
.equ DAC_C0_DACSWTRG_SHIFT, 4
.equ DAC_C0_LPEN_MASK, 0x08
.equ DAC_C0_LPEN_SHIFT, 3
.equ DAC_C0_DACBTIEN_MASK, 0x02
.equ DAC_C0_DACBTIEN_SHIFT, 1
.equ DAC_C0_DACBBIEN_MASK, 0x01
.equ DAC_C0_DACBBIEN_SHIFT, 0
//---------------------------------------------------------------
//DAC_C1:  DAC control register 1
//  7:DMAEN=DMA enable select
//6-3:(reserved)
//  2:DACBFMD=DAC buffer work mode select
//            0:normal
//            1:one-time scan
//  1:(reserved)
//  0:DACBFEN=DAC buffer enable
//            0:disabled:data in first word of buffer
//            1:enabled:read pointer points to data
.equ DAC_C1_DMAEN_MASK, 0x80
.equ DAC_C1_DMAEN_SHIFT, 7
.equ DAC_C1_DACBFMD_MASK, 0x04
.equ DAC_C1_DACBFMD_SHIFT, 2
.equ DAC_C1_DACBFEN_MASK, 0x01
.equ DAC_C1_DACBFEN_SHIFT, 0
//---------------------------------------------------------------
//DAC_C2:  DAC control register 2
//7-5:(reserved):read-only:0
//  4:DACBFRP=DAC buffer read pointer
//3-1:(reserved):read-only:0
//  0:DACBFUP=DAC buffer read upper limit
.equ DAC_C2_DACBFRP_MASK, 0x10
.equ DAC_C2_DACBFRP_SHIFT, 4
.equ DAC_C2_DACBFUP_MASK, 0x01
.equ DAC_C2_DACBFUP_SHIFT, 0
//---------------------------------------------------------------
//DAC_SR:  DAC status register
//Writing 0 clears a field// writing 1 has no effect.
//7-2:(reserved):read-only:0
//1:DACBFRPTF=DAC buffer read pointer top position flag
//            Indicates whether pointer is zero
//0:DACBFRPBF=DAC buffer read pointer bottom position flag
//            Indicates whether pointer is equal to DAC0_C2.DACBFUP.
.equ DAC_SR_DACBFRPTF_MASK, 0x02
.equ DAC_SR_DACBFRPTF_SHIFT, 1
.equ DAC_SR_DACBFRPBF_MASK, 0x01
.equ DAC_SR_DACBFRPBF_SHIFT, 0
//---------------------------------------------------------------
//Fast (zero wait state) GPIO (FGPIO) or (IOPORT)
//FGPIOx_PDD: Port x Data Direction Register
//Bit n:  0=Port x pin n configured as input
//        1=Port x pin n configured as output
.equ FGPIO_BASE, 0xF80FF000
//offsets for PDOR, PSOR, PCOR, PTOR, PDIR, and PDDR defined
//  with .equ GPIO, ates
//offsets for Ports A-E defined with .equ GPIO, ates
//Port A
.equ FGPIOA_BASE, 0xF80FF000
.equ FGPIOA_PDOR, (FGPIOA_BASE + GPIO_PDOR_OFFSET)
.equ FGPIOA_PSOR, (FGPIOA_BASE + GPIO_PSOR_OFFSET)
.equ FGPIOA_PCOR, (FGPIOA_BASE + GPIO_PCOR_OFFSET)
.equ FGPIOA_PTOR, (FGPIOA_BASE + GPIO_PTOR_OFFSET)
.equ FGPIOA_PDIR, (FGPIOA_BASE + GPIO_PDIR_OFFSET)
.equ FGPIOA_PDDR, (FGPIOA_BASE + GPIO_PDDR_OFFSET)
//Port B
.equ FGPIOB_BASE, 0xF80FF040
.equ FGPIOB_PDOR, (FGPIOB_BASE + GPIO_PDOR_OFFSET)
.equ FGPIOB_PSOR, (FGPIOB_BASE + GPIO_PSOR_OFFSET)
.equ FGPIOB_PCOR, (FGPIOB_BASE + GPIO_PCOR_OFFSET)
.equ FGPIOB_PTOR, (FGPIOB_BASE + GPIO_PTOR_OFFSET)
.equ FGPIOB_PDIR, (FGPIOB_BASE + GPIO_PDIR_OFFSET)
.equ FGPIOB_PDDR, (FGPIOB_BASE + GPIO_PDDR_OFFSET)
//Port C
.equ FGPIOC_BASE, 0xF80FF080
.equ FGPIOC_PDOR, (FGPIOC_BASE + GPIO_PDOR_OFFSET)
.equ FGPIOC_PSOR, (FGPIOC_BASE + GPIO_PSOR_OFFSET)
.equ FGPIOC_PCOR, (FGPIOC_BASE + GPIO_PCOR_OFFSET)
.equ FGPIOC_PTOR, (FGPIOC_BASE + GPIO_PTOR_OFFSET)
.equ FGPIOC_PDIR, (FGPIOC_BASE + GPIO_PDIR_OFFSET)
.equ FGPIOC_PDDR, (FGPIOC_BASE + GPIO_PDDR_OFFSET)
//Port D
.equ FGPIOD_BASE, 0xF80FF0C0
.equ FGPIOD_PDOR, (FGPIOD_BASE + GPIO_PDOR_OFFSET)
.equ FGPIOD_PSOR, (FGPIOD_BASE + GPIO_PSOR_OFFSET)
.equ FGPIOD_PCOR, (FGPIOD_BASE + GPIO_PCOR_OFFSET)
.equ FGPIOD_PTOR, (FGPIOD_BASE + GPIO_PTOR_OFFSET)
.equ FGPIOD_PDIR, (FGPIOD_BASE + GPIO_PDIR_OFFSET)
.equ FGPIOD_PDDR, (FGPIOD_BASE + GPIO_PDDR_OFFSET)
//Port E
.equ FGPIOE_BASE, 0xF80FF100
.equ FGPIOE_PDOR, (FGPIOE_BASE + GPIO_PDOR_OFFSET)
.equ FGPIOE_PSOR, (FGPIOE_BASE + GPIO_PSOR_OFFSET)
.equ FGPIOE_PCOR, (FGPIOE_BASE + GPIO_PCOR_OFFSET)
.equ FGPIOE_PTOR, (FGPIOE_BASE + GPIO_PTOR_OFFSET)
.equ FGPIOE_PDIR, (FGPIOE_BASE + GPIO_PDIR_OFFSET)
.equ FGPIOE_PDDR, (FGPIOE_BASE + GPIO_PDDR_OFFSET)
//---------------------------------------------------------------
//Flash Configuration Field (FCF) 0x400-0x40F
//Following Freescale startup_MKL46Z4.s
//     CMSIS Cortex-M0plus Core Device Startup File for the MKL64Z4
//     v2.2, 4/12/2013
//16-byte flash configuration field that stores default protection settings
//(loaded on reset) and security information that allows the MCU to 
//restrict acces to the FTFL module.
//FCF Backdoor Comparison Key
//8 bytes from 0x400-0x407
//-----------------------------------------------------
//FCF Backdoor Comparison Key 0
//7-0:Backdoor Key 0
.equ FCF_BACKDOOR_KEY0, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 1
//7-0:Backdoor Key 1
.equ FCF_BACKDOOR_KEY1, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 2
//7-0:Backdoor Key 2
.equ FCF_BACKDOOR_KEY2, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 3
//7-0:Backdoor Key 3
.equ FCF_BACKDOOR_KEY3, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 4
//7-0:Backdoor Key 4
.equ FCF_BACKDOOR_KEY4, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 5
//7-0:Backdoor Key 5
.equ FCF_BACKDOOR_KEY5, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 6
//7-0:Backdoor Key 6
.equ FCF_BACKDOOR_KEY6, 0xFF
//-----------------------------------------------------
//FCF Backdoor Comparison Key 7
//7-0:Backdoor Key 7
.equ FCF_BACKDOOR_KEY7, 0xFF
//-----------------------------------------------------
//FCF Flash nonvolatile option byte (FCF_FOPT)
//Allows user to customize operation of the MCU at boot time.
//7-6:11:(reserved)
//  5: 1:FAST_INIT=fast initialization
//4,0:11:LPBOOT=core and system clock divider:  2^(3-LPBOOT)
//  3: 1:RESET_PIN_CFG=enable reset pin following POR
//  2: 1:NMI_DIS=Enable NMI
//  1: 1:(reserved)
//  0:(see bit 4 above)
.equ FCF_FOPT, 0xFF
//-----------------------------------------------------
//FCF Program flash protection bytes (FCF_FPROT)
//Each program flash region can be protected from program and erase 
//operation by setting the associated PROT bit.  Each bit protects a 
//1/32 region of the program flash memory.
//FCF FPROT0
//7:1:FCF_PROT7=Program flash region 7/32 not protected
//6:1:FCF_PROT6=Program flash region 6/32 not protected
//5:1:FCF_PROT5=Program flash region 5/32 not protected
//4:1:FCF_PROT4=Program flash region 4/32 not protected
//3:1:FCF_PROT3=Program flash region 3/32 not protected
//2:1:FCF_PROT2=Program flash region 2/32 not protected
//1:1:FCF_PROT1=Program flash region 1/32 not protected
//0:1:FCF_PROT0=Program flash region 0/32 not protected
.equ FCF_FPROT0, 0xFF
//-----------------------------------------------------
//FCF FPROT1
//7:1:FCF_PROT15=Program flash region 15/32 not protected
//6:1:FCF_PROT14=Program flash region 14/32 not protected
//5:1:FCF_PROT13=Program flash region 13/32 not protected
//4:1:FCF_PROT12=Program flash region 12/32 not protected
//3:1:FCF_PROT11=Program flash region 11/32 not protected
//2:1:FCF_PROT10=Program flash region 10/32 not protected
//1:1:FCF_PROT9=Program flash region 9/32 not protected
//0:1:FCF_PROT8=Program flash region 8/32 not protected
.equ FCF_FPROT1, 0xFF
//-----------------------------------------------------
//FCF FPROT2
//7:1:FCF_PROT23=Program flash region 23/32 not protected
//6:1:FCF_PROT22=Program flash region 22/32 not protected
//5:1:FCF_PROT21=Program flash region 21/32 not protected
//4:1:FCF_PROT20=Program flash region 20/32 not protected
//3:1:FCF_PROT19=Program flash region 19/32 not protected
//2:1:FCF_PROT18=Program flash region 18/32 not protected
//1:1:FCF_PROT17=Program flash region 17/32 not protected
//0:1:FCF_PROT16=Program flash region 16/32 not protected
.equ FCF_FPROT2, 0xFF
//-----------------------------------------------------
//FCF FPROT3
//7:1:FCF_PROT31=Program flash region 31/32 not protected
//6:1:FCF_PROT30=Program flash region 30/32 not protected
//5:1:FCF_PROT29=Program flash region 29/32 not protected
//4:1:FCF_PROT28=Program flash region 28/32 not protected
//3:1:FCF_PROT27=Program flash region 27/32 not protected
//2:1:FCF_PROT26=Program flash region 26/32 not protected
//1:1:FCF_PROT25=Program flash region 25/32 not protected
//0:1:FCF_PROT24=Program flash region 24/32 not protected
.equ FCF_FPROT3, 0xFF
//-----------------------------------------------------
//FCF Flash security byte (FCF_FSEC)
//WARNING: If SEC field is configured as "MCU security status is 
//secure" and MEEN field is configured as "Mass erase is disabled",
//MCU's security status cannot be set back to unsecure state since 
//mass erase via the debugger is blocked !!!
//7-6:01:KEYEN=backdoor key security enable
//            :00=Backdoor key access disabled
//            :01=Backdoor key access disabled (preferred value)
//            :10=Backdoor key access enabled
//            :11=Backdoor key access disabled
//5-4:11:MEEN=mass erase enable bits
//           (does not matter if SEC unsecure)
//           :00=mass erase enabled
//           :01=mass erase enabled
//           :10=mass erase disabled
//           :11=mass erase enabled
//3-2:11:FSLACC=Freescale failure analysis access code
//             (does not matter if SEC unsecure)
//             :00=Freescale factory access granted
//             :01=Freescale factory access denied
//             :10=Freescale factory access denied
//             :11=Freescale factory access granted
//1-0:10:SEC=flash security
//          :00=MCU secure
//          :01=MCU secure
//          :10=MCU unsecure (standard value)
//          :11=MCU secure
.equ FCF_FSEC, 0x7E
//---------------------------------------------------------------
//General-purpose input and output (GPIO)
//GPIOx_PDD: Port x Data Direction Register
//Bit n:  0=Port x pin n configured as input
//        1=Port x pin n configured as output
.equ GPIO_BASE, 0x400FF000
.equ GPIO_PDOR_OFFSET, 0x00
.equ GPIO_PSOR_OFFSET, 0x04
.equ GPIO_PCOR_OFFSET, 0x08
.equ GPIO_PTOR_OFFSET, 0x0C
.equ GPIO_PDIR_OFFSET, 0x10
.equ GPIO_PDDR_OFFSET, 0x14
.equ GPIOA_OFFSET, 0x00
.equ GPIOB_OFFSET, 0x40
.equ GPIOC_OFFSET, 0x80
.equ GPIOD_OFFSET, 0xC0
.equ GPIOE_OFFSET, 0x0100
//Port A
.equ GPIOA_BASE, 0x400FF000
.equ GPIOA_PDOR, (GPIOA_BASE + GPIO_PDOR_OFFSET)
.equ GPIOA_PSOR, (GPIOA_BASE + GPIO_PSOR_OFFSET)
.equ GPIOA_PCOR, (GPIOA_BASE + GPIO_PCOR_OFFSET)
.equ GPIOA_PTOR, (GPIOA_BASE + GPIO_PTOR_OFFSET)
.equ GPIOA_PDIR, (GPIOA_BASE + GPIO_PDIR_OFFSET)
.equ GPIOA_PDDR, (GPIOA_BASE + GPIO_PDDR_OFFSET)
//Port B
.equ GPIOB_BASE, 0x400FF040
.equ GPIOB_PDOR, (GPIOB_BASE + GPIO_PDOR_OFFSET)
.equ GPIOB_PSOR, (GPIOB_BASE + GPIO_PSOR_OFFSET)
.equ GPIOB_PCOR, (GPIOB_BASE + GPIO_PCOR_OFFSET)
.equ GPIOB_PTOR, (GPIOB_BASE + GPIO_PTOR_OFFSET)
.equ GPIOB_PDIR, (GPIOB_BASE + GPIO_PDIR_OFFSET)
.equ GPIOB_PDDR, (GPIOB_BASE + GPIO_PDDR_OFFSET)
//Port C
.equ GPIOC_BASE, 0x400FF080
.equ GPIOC_PDOR, (GPIOC_BASE + GPIO_PDOR_OFFSET)
.equ GPIOC_PSOR, (GPIOC_BASE + GPIO_PSOR_OFFSET)
.equ GPIOC_PCOR, (GPIOC_BASE + GPIO_PCOR_OFFSET)
.equ GPIOC_PTOR, (GPIOC_BASE + GPIO_PTOR_OFFSET)
.equ GPIOC_PDIR, (GPIOC_BASE + GPIO_PDIR_OFFSET)
.equ GPIOC_PDDR, (GPIOC_BASE + GPIO_PDDR_OFFSET)
//Port D
.equ GPIOD_BASE, 0x400FF0C0
.equ GPIOD_PDOR, (GPIOD_BASE + GPIO_PDOR_OFFSET)
.equ GPIOD_PSOR, (GPIOD_BASE + GPIO_PSOR_OFFSET)
.equ GPIOD_PCOR, (GPIOD_BASE + GPIO_PCOR_OFFSET)
.equ GPIOD_PTOR, (GPIOD_BASE + GPIO_PTOR_OFFSET)
.equ GPIOD_PDIR, (GPIOD_BASE + GPIO_PDIR_OFFSET)
.equ GPIOD_PDDR, (GPIOD_BASE + GPIO_PDDR_OFFSET)
//Port E
.equ GPIOE_BASE, 0x400FF100
.equ GPIOE_PDOR, (GPIOE_BASE + GPIO_PDOR_OFFSET)
.equ GPIOE_PSOR, (GPIOE_BASE + GPIO_PSOR_OFFSET)
.equ GPIOE_PCOR, (GPIOE_BASE + GPIO_PCOR_OFFSET)
.equ GPIOE_PTOR, (GPIOE_BASE + GPIO_PTOR_OFFSET)
.equ GPIOE_PDIR, (GPIOE_BASE + GPIO_PDIR_OFFSET)
.equ GPIOE_PDDR, (GPIOE_BASE + GPIO_PDDR_OFFSET)
//---------------------------------------------------------------
//IOPORT:  GPIO alias for zero wait state access to GPIO
//See FGPIO
//---------------------------------------------------------------
//LCD Controller (SLCD)
.equ LCD_BASE, 0x40053000
.equ LCD_GCR_OFFSET, 0x00
.equ LCD_AR_OFFSET, 0x04
.equ LCD_FDCR_OFFSET, 0x08
.equ LCD_FDSR_OFFSET, 0x0C
.equ LCD_PENL_OFFSET, 0x10
.equ LCD_PENH_OFFSET, 0x14
.equ LCD_BPENL_OFFSET, 0x18
.equ LCD_BPENH_OFFSET, 0x1C
.equ LCD_WF_OFFSET, 0x20  //WF.D[16] or WF.B[64]
.equ LCD_WF3TO0_OFFSET, 0x20
.equ LCD_WF7TO4_OFFSET, 0x24
.equ LCD_WF11TO8_OFFSET, 0x28
.equ LCD_WF15TO12_OFFSET, 0x2C
.equ LCD_WF19TO16_OFFSET, 0x30
.equ LCD_WF23TO20_OFFSET, 0x34
.equ LCD_WF27TO24_OFFSET, 0x38
.equ LCD_WF31TO28_OFFSET, 0x3C
.equ LCD_WF35TO32_OFFSET, 0x40
.equ LCD_WF39TO36_OFFSET, 0x44
.equ LCD_WF43TO40_OFFSET, 0x48
.equ LCD_WF47TO44_OFFSET, 0x4C
.equ LCD_WF51TO48_OFFSET, 0x50
.equ LCD_WF55TO52_OFFSET, 0x54
.equ LCD_WF59TO56_OFFSET, 0x58
.equ LCD_WF63TO60_OFFSET, 0x5C
.equ LCD_GCR, (LCD_BASE + LCD_GCR_OFFSET)
.equ LCD_AR, (LCD_BASE + LCD_AR_OFFSET)
.equ LCD_FDCR, (LCD_BASE + LCD_FDCR_OFFSET)
.equ LCD_FDSR, (LCD_BASE + LCD_FDSR_OFFSET)
.equ LCD_PENL, (LCD_BASE + LCD_PENL_OFFSET)
.equ LCD_PENH, (LCD_BASE + LCD_PENH_OFFSET)
.equ LCD_BPENL, (LCD_BASE + LCD_BPENL_OFFSET)
.equ LCD_BPENH, (LCD_BASE + LCD_BPENH_OFFSET)
.equ LCD_WF, (LCD_BASE + LCD_WF_OFFSET)  //WF.D[16] or WF.B[64]
.equ LCD_WF3TO0, (LCD_BASE + LCD_WF3TO0_OFFSET)
.equ LCD_WF7TO4, (LCD_BASE + LCD_WF7TO4_OFFSET)
.equ LCD_WF11TO8, (LCD_BASE + LCD_WF11TO8_OFFSET)
.equ LCD_WF15TO12, (LCD_BASE + LCD_WF15TO12_OFFSET)
.equ LCD_WF19TO16, (LCD_BASE + LCD_WF19TO16_OFFSET)
.equ LCD_WF23TO20, (LCD_BASE + LCD_WF23TO20_OFFSET)
.equ LCD_WF27TO24, (LCD_BASE + LCD_WF27TO24_OFFSET)
.equ LCD_WF31TO28, (LCD_BASE + LCD_WF31TO28_OFFSET)
.equ LCD_WF35TO32, (LCD_BASE + LCD_WF35TO32_OFFSET)
.equ LCD_WF39TO36, (LCD_BASE + LCD_WF39TO36_OFFSET)
.equ LCD_WF43TO40, (LCD_BASE + LCD_WF43TO40_OFFSET)
.equ LCD_WF47TO44, (LCD_BASE + LCD_WF47TO44_OFFSET)
.equ LCD_WF51TO48, (LCD_BASE + LCD_WF51TO48_OFFSET)
.equ LCD_WF55TO52, (LCD_BASE + LCD_WF55TO52_OFFSET)
.equ LCD_WF59TO56, (LCD_BASE + LCD_WF59TO56_OFFSET)
.equ LCD_WF63TO60, (LCD_BASE + LCD_WF63TO60_OFFSET)
//---------------------------------------------------------------
//LCD_GCR:  LCD general control register:  (POR:  0x08300003)
//   31:  RVEN:  regulated voltage enable:  (POR:  0)
//30-28:  (reserved):read-only:000
//27-24:  RVTRIM:  regulated voltage trim:  (1000)
//   23:  CPSEL:  charge pump select: (0)
//                0:  resister network
//                1:  charge pump
//   22:  (reserved):read-only:0
//21-20:  LADJ:  load adjust:  (11)
//          CPSEL = 0:  adjust resistor bias network
//            00:  low load (LCD glass <= 2000 pF)
//                 LCD or GPIO for pins VLL1, VLL2, VCAP1, or VCAP2
//            01:  low laod (LCD glass <= 2000 pF)
//                 LCD or GPIO for pins VLL1, VLL2, VCAP1, or VCAP2
//            10:  high load (LCD glass <= 8000 pF)
//                 LCD or GPIO for pins VCAP1 or VCAP2
//            11:  high load (LCD glass <= 8000 pF)
//                 LCD or GPIO for pins VCAP1 or VCAP2
//          CPSEL = 1:  adjust clock source for charge pump
//            00:  fastest clock (LCD glass <= 8000 pF [4000 pF if FFR])
//            01:  intermediate clock (LCD glass <= 4000 pF [2000 pF if FFR])
//            10:  intermediate clock (LCD glass <= 2000 pF [1000 pF if FFR])
//            11:  slowest clock (LCD glass <= 1000 pF [500 pF if FFR])
//   19:  (reserved):read-only:0
//   18:  (reserved):read-only:0
//   17:  VSUPPLY:  voltage supply control:  (0)
//                  0:  VLL3 internally from VDD
//                  1:  VLL3 externally from VDD or VLL internally from VIREG
//   16:  (reserved):read-only:0
//   15:  PADSAFE:  pad safe state enable:  (0)
//   14:  FDCIEN:  LCD fault detection complete interrupt enable:  (0)
//13-12:  ATLDIV:  LCD alternate clock divider = 8^ALTDIV:  (00)
//   11:  ALTSOURCE:  alternate clock source select = ALTSOURCE + 1:  (0)
//   10:  FFR:  fast frame rate select:  (0)
//              0:  standard frequency, [23.3, 73.1]
//              1:  fast frequency = std. x 2, [46.6, 146.2]
//    9:  LCDDOZE:  LCD doze enable:  (0)
//    8:  LCDSTP:  LCD stop:  (0)
//    7:  LCDEN:  LCD driver enable:  (0)
//    6:  SOURCE:  LCD clock source select:  (0)
//                 0:  Default clock (SIM_SOPT1.OSC32KSEL)
//                 1:  Alternate clock (LCD_GCR.ALTSOURCE)
//  5-3:  LCLK:  LCD clock prescaler:  (000)
//               frequency = SOURCE / ((DUTY + 1) x 8 x (4 +LCLK) x Y)
//  2-0:  DUTY:  LCD duty select:  (011)
//               000, 001, 010, 011, 111:  1 / (DUTY + 1) duty cycle
.equ LCD_GCR_DUTY_MASK, 0x7
.equ LCD_GCR_DUTY_SHIFT, 0
.equ LCD_GCR_LCLK_MASK, 0x38
.equ LCD_GCR_LCLK_SHIFT, 3
.equ LCD_GCR_SOURCE_MASK, 0x40
.equ LCD_GCR_SOURCE_SHIFT, 6
.equ LCD_GCR_LCDEN_MASK, 0x80
.equ LCD_GCR_LCDEN_SHIFT, 7
.equ LCD_GCR_LCDSTP_MASK, 0x100
.equ LCD_GCR_LCDSTP_SHIFT, 8
.equ LCD_GCR_LCDDOZE_MASK, 0x200
.equ LCD_GCR_LCDDOZE_SHIFT, 9
.equ LCD_GCR_FFR_MASK, 0x400
.equ LCD_GCR_FFR_SHIFT, 10
.equ LCD_GCR_ALTSOURCE_MASK, 0x800
.equ LCD_GCR_ALTSOURCE_SHIFT, 11
.equ LCD_GCR_ALTDIV_MASK, 0x3000
.equ LCD_GCR_ALTDIV_SHIFT, 12
.equ LCD_GCR_FDCIEN_MASK, 0x4000
.equ LCD_GCR_FDCIEN_SHIFT, 14
.equ LCD_GCR_PADSAFE_MASK, 0x8000
.equ LCD_GCR_PADSAFE_SHIFT, 15
.equ LCD_GCR_VSUPPLY_MASK, 0x20000
.equ LCD_GCR_VSUPPLY_SHIFT, 17
.equ LCD_GCR_LADJ_MASK, 0x300000
.equ LCD_GCR_LADJ_SHIFT, 20
.equ LCD_GCR_CPSEL_MASK, 0x800000
.equ LCD_GCR_CPSEL_SHIFT, 23
.equ LCD_GCR_RVTRIM_MASK, 0xF000000
.equ LCD_GCR_RVTRIM_SHIFT, 24
.equ LCD_GCR_RVEN_MASK, 0x80000000
.equ LCD_GCR_RVEN_SHIFT, 31
//---------------------------------------------------------------
//LCD_AR:  LCD auxiliary register:  (POR:  0x00000000)
//31-16:  (reserved):read-only:0x0000
//   15:  (reserved):read-only:0
// 14-8:  (reserved):read-only:0000000
//    7:  BLINK:  blink command:  (0)
//    6:  ALT:  alternate display mode:  (0)
//    5:  BLANK:  blank display mode:  (0)
//    4:  (reserved:read-only:0
//    3:  BMODE:  blink mode:  (0)
//                0:  display blank during blink period
//                1:  display alternate display during blink period (LCD_GCR.DUTY < 5)
//  2-0:  BRATE:  blink-rate configuration:  (000)
//                Blink rate = LCD clock / (2 ^ (12 + BRATE))
.equ LCD_AR_BRATE_MASK, 0x7
.equ LCD_AR_BRATE_SHIFT, 0
.equ LCD_AR_BMODE_MASK, 0x8
.equ LCD_AR_BMODE_SHIFT, 3
.equ LCD_AR_BLANK_MASK, 0x20
.equ LCD_AR_BLANK_SHIFT, 5
.equ LCD_AR_ALT_MASK, 0x40
.equ LCD_AR_ALT_SHIFT, 6
.equ LCD_AR_BLINK_MASK, 0x80
.equ LCD_AR_BLINK_SHIFT, 7
//---------------------------------------------------------------
//LCD_FDCR:  LCD fault detect control register:  (POR:  0x00000000)
//31-16:  (reserved):read-only:0x0000
//   15:  (reserved):read-only:0
//14-12:  FDPRS:  fault detect clock prescaler (000)
//                Fault detect sample clock = Bus clock / (2 ^ FDPRS)
// 11-9:  FDSWW:  fault detect sample window width:  (000)
//                Window width = 4 x (2 ^ FDSWW)
//    8:  (reserved):read-only:0
//    7:  FDEN:  fault detect enable:  (0)
//    6:  FDBPEN:  fault detect back plane enable:  (0)
//  5-0:  BDPINID:  fault detect pin ID:  (000000)
.equ LCD_FDCR_FDPINID_MASK, 0x3F
.equ LCD_FDCR_FDPINID_SHIFT, 0
.equ LCD_FDCR_FDBPEN_MASK, 0x40
.equ LCD_FDCR_FDBPEN_SHIFT, 6
.equ LCD_FDCR_FDEN_MASK, 0x80
.equ LCD_FDCR_FDEN_SHIFT, 7
.equ LCD_FDCR_FDSWW_MASK, 0xE00
.equ LCD_FDCR_FDSWW_SHIFT, 9
.equ LCD_FDCR_FDPRS_MASK, 0x7000
.equ LCD_FDCR_FDPRS_SHIFT, 12
//---------------------------------------------------------------
//LCD_FDSR:  LCD fault detect status register:  (POR:  0x00000000)
//31-16:  (reserved):read-only:0x0000
//   15:  FDCF:  fault detection complete flag:  (0)
//         read:0=fault detection not completed
//              1=fault detection completed
//         write:0=(no effect)
//               1=clear
// 14-8:  (reserved):read-only:0000000
//  7-0:  FDCNT:  fault detect counter:  (0x00)
.equ LCD_FDSR_FDCNT_MASK, 0xFF
.equ LCD_FDSR_FDCNT_SHIFT, 0
.equ LCD_FDSR_FDCF_MASK, 0x8000
.equ LCD_FDSR_FDCF_SHIFT, 15
//---------------------------------------------------------------
//LCD_PENn:  LCD pen enable register:  (POR:  0x00000000)
//31-0:  PEN:  LCD pin enable
//       bit n enables LCD operation on LCD pin n
.equ LCD_PEN_PEN_MASK, 0xFFFFFFFF
.equ LCD_PEN_PEN_SHIFT, 0
//---------------------------------------------------------------
//LCD_BPENn:  LCD back plane enable register:  (POR:  0x00000000)
//31-0:  BPEN:  back plane enable
//       bit n enables front'/back plane operation on LCD pin n
.equ LCD_BPEN_BPEN_MASK, 0xFFFFFFFF
.equ LCD_BPEN_BPEN_SHIFT, 0
//---------------------------------------------------------------
//LCD_WFyTOx:  LCD waveform register:  (POR:  0x00000000)
//May be written with 8-, 16-, or 32-bit writes
//y = x + 3
//31-24:  WFy [WF(x+3)]
//23-16:  WF(y-1) [WF(x+2)]
// 15-8:  WF(y-2) [WF(x+1)]
//  7-0:  WF(y-3) [WFx]
//        Each byte WFn (n in {x, x+1, x+2, y}) controls an LCD pen
//          bit 7:segment/phase H
//          bit 6:segment/phase G
//          bit 5:segment/phase F
//          bit 4:segment/phase E
//          bit 3:segment/phase D
//          bit 2:segment/phase C
//          bit 1:segment/phase B
//          bit 0:segment/phase A:
//                value 0:  segment off or phase deactivtated
//                value 1:  segment on or phase activated
//---------------------------------------------------------------
//Custom WFn register mapping not in C MKL46Z4.h
.equ LCD_WF0_REG, LCD_WF3TO0
.equ LCD_WF1_REG, LCD_WF3TO0
.equ LCD_WF2_REG, LCD_WF3TO0
.equ LCD_WF3_REG, LCD_WF3TO0
.equ LCD_WF4_REG, LCD_WF7TO4
.equ LCD_WF5_REG, LCD_WF7TO4
.equ LCD_WF6_REG, LCD_WF7TO4
.equ LCD_WF7_REG, LCD_WF7TO4
.equ LCD_WF8_REG, LCD_WF11TO8
.equ LCD_WF9_REG, LCD_WF11TO8
.equ LCD_WF10_REG, LCD_WF11TO8
.equ LCD_WF11_REG, LCD_WF11TO8
.equ LCD_WF12_REG, LCD_WF15TO12
.equ LCD_WF13_REG, LCD_WF15TO12
.equ LCD_WF14_REG, LCD_WF15TO12
.equ LCD_WF15_REG, LCD_WF15TO12
.equ LCD_WF16_REG, LCD_WF19TO16
.equ LCD_WF17_REG, LCD_WF19TO16
.equ LCD_WF18_REG, LCD_WF19TO16
.equ LCD_WF19_REG, LCD_WF19TO16
.equ LCD_WF20_REG, LCD_WF23TO20
.equ LCD_WF21_REG, LCD_WF23TO20
.equ LCD_WF22_REG, LCD_WF23TO20
.equ LCD_WF23_REG, LCD_WF23TO20
.equ LCD_WF24_REG, LCD_WF27TO24
.equ LCD_WF25_REG, LCD_WF27TO24
.equ LCD_WF26_REG, LCD_WF27TO24
.equ LCD_WF27_REG, LCD_WF27TO24
.equ LCD_WF28_REG, LCD_WF31TO28
.equ LCD_WF29_REG, LCD_WF31TO28
.equ LCD_WF30_REG, LCD_WF31TO28
.equ LCD_WF31_REG, LCD_WF31TO28
.equ LCD_WF32_REG, LCD_WF35TO32
.equ LCD_WF33_REG, LCD_WF35TO32
.equ LCD_WF34_REG, LCD_WF35TO32
.equ LCD_WF35_REG, LCD_WF35TO32
.equ LCD_WF36_REG, LCD_WF39TO36
.equ LCD_WF37_REG, LCD_WF39TO36
.equ LCD_WF38_REG, LCD_WF39TO36
.equ LCD_WF39_REG, LCD_WF39TO36
.equ LCD_WF40_REG, LCD_WF43TO40
.equ LCD_WF41_REG, LCD_WF43TO40
.equ LCD_WF42_REG, LCD_WF43TO40
.equ LCD_WF43_REG, LCD_WF43TO40
.equ LCD_WF44_REG, LCD_WF47TO44
.equ LCD_WF45_REG, LCD_WF47TO44
.equ LCD_WF46_REG, LCD_WF47TO44
.equ LCD_WF47_REG, LCD_WF47TO44
.equ LCD_WF48_REG, LCD_WF51TO48
.equ LCD_WF49_REG, LCD_WF51TO48
.equ LCD_WF50_REG, LCD_WF51TO48
.equ LCD_WF51_REG, LCD_WF51TO48
.equ LCD_WF52_REG, LCD_WF55TO52
.equ LCD_WF53_REG, LCD_WF55TO52
.equ LCD_WF54_REG, LCD_WF55TO52
.equ LCD_WF55_REG, LCD_WF55TO52
.equ LCD_WF56_REG, LCD_WF59TO56
.equ LCD_WF57_REG, LCD_WF59TO56
.equ LCD_WF58_REG, LCD_WF59TO56
.equ LCD_WF59_REG, LCD_WF59TO56
.equ LCD_WF60_REG, LCD_WF63TO60
.equ LCD_WF61_REG, LCD_WF63TO60
.equ LCD_WF62_REG, LCD_WF63TO60
.equ LCD_WF63_REG, LCD_WF63TO60
//---------------------------------------------------------------
///* WF Bit Fields */ from C MKL46Z4.h
.equ LCD_WF_WF0_MASK, 0xFF
.equ LCD_WF_WF0_SHIFT, 0
.equ LCD_WF_WF60_MASK, 0xFF
.equ LCD_WF_WF60_SHIFT, 0
.equ LCD_WF_WF56_MASK, 0xFF
.equ LCD_WF_WF56_SHIFT, 0
.equ LCD_WF_WF52_MASK, 0xFF
.equ LCD_WF_WF52_SHIFT, 0
.equ LCD_WF_WF4_MASK, 0xFF
.equ LCD_WF_WF4_SHIFT, 0
.equ LCD_WF_WF48_MASK, 0xFF
.equ LCD_WF_WF48_SHIFT, 0
.equ LCD_WF_WF44_MASK, 0xFF
.equ LCD_WF_WF44_SHIFT, 0
.equ LCD_WF_WF40_MASK, 0xFF
.equ LCD_WF_WF40_SHIFT, 0
.equ LCD_WF_WF8_MASK, 0xFF
.equ LCD_WF_WF8_SHIFT, 0
.equ LCD_WF_WF36_MASK, 0xFF
.equ LCD_WF_WF36_SHIFT, 0
.equ LCD_WF_WF32_MASK, 0xFF
.equ LCD_WF_WF32_SHIFT, 0
.equ LCD_WF_WF28_MASK, 0xFF
.equ LCD_WF_WF28_SHIFT, 0
.equ LCD_WF_WF12_MASK, 0xFF
.equ LCD_WF_WF12_SHIFT, 0
.equ LCD_WF_WF24_MASK, 0xFF
.equ LCD_WF_WF24_SHIFT, 0
.equ LCD_WF_WF20_MASK, 0xFF
.equ LCD_WF_WF20_SHIFT, 0
.equ LCD_WF_WF16_MASK, 0xFF
.equ LCD_WF_WF16_SHIFT, 0
.equ LCD_WF_WF5_MASK, 0xFF00
.equ LCD_WF_WF5_SHIFT, 8
.equ LCD_WF_WF49_MASK, 0xFF00
.equ LCD_WF_WF49_SHIFT, 8
.equ LCD_WF_WF45_MASK, 0xFF00
.equ LCD_WF_WF45_SHIFT, 8
.equ LCD_WF_WF61_MASK, 0xFF00
.equ LCD_WF_WF61_SHIFT, 8
.equ LCD_WF_WF25_MASK, 0xFF00
.equ LCD_WF_WF25_SHIFT, 8
.equ LCD_WF_WF17_MASK, 0xFF00
.equ LCD_WF_WF17_SHIFT, 8
.equ LCD_WF_WF41_MASK, 0xFF00
.equ LCD_WF_WF41_SHIFT, 8
.equ LCD_WF_WF13_MASK, 0xFF00
.equ LCD_WF_WF13_SHIFT, 8
.equ LCD_WF_WF57_MASK, 0xFF00
.equ LCD_WF_WF57_SHIFT, 8
.equ LCD_WF_WF53_MASK, 0xFF00
.equ LCD_WF_WF53_SHIFT, 8
.equ LCD_WF_WF37_MASK, 0xFF00
.equ LCD_WF_WF37_SHIFT, 8
.equ LCD_WF_WF9_MASK, 0xFF00
.equ LCD_WF_WF9_SHIFT, 8
.equ LCD_WF_WF1_MASK, 0xFF00
.equ LCD_WF_WF1_SHIFT, 8
.equ LCD_WF_WF29_MASK, 0xFF00
.equ LCD_WF_WF29_SHIFT, 8
.equ LCD_WF_WF33_MASK, 0xFF00
.equ LCD_WF_WF33_SHIFT, 8
.equ LCD_WF_WF21_MASK, 0xFF00
.equ LCD_WF_WF21_SHIFT, 8
.equ LCD_WF_WF26_MASK, 0xFF0000
.equ LCD_WF_WF26_SHIFT, 16
.equ LCD_WF_WF46_MASK, 0xFF0000
.equ LCD_WF_WF46_SHIFT, 16
.equ LCD_WF_WF6_MASK, 0xFF0000
.equ LCD_WF_WF6_SHIFT, 16
.equ LCD_WF_WF42_MASK, 0xFF0000
.equ LCD_WF_WF42_SHIFT, 16
.equ LCD_WF_WF18_MASK, 0xFF0000
.equ LCD_WF_WF18_SHIFT, 16
.equ LCD_WF_WF38_MASK, 0xFF0000
.equ LCD_WF_WF38_SHIFT, 16
.equ LCD_WF_WF22_MASK, 0xFF0000
.equ LCD_WF_WF22_SHIFT, 16
.equ LCD_WF_WF34_MASK, 0xFF0000
.equ LCD_WF_WF34_SHIFT, 16
.equ LCD_WF_WF50_MASK, 0xFF0000
.equ LCD_WF_WF50_SHIFT, 16
.equ LCD_WF_WF14_MASK, 0xFF0000
.equ LCD_WF_WF14_SHIFT, 16
.equ LCD_WF_WF54_MASK, 0xFF0000
.equ LCD_WF_WF54_SHIFT, 16
.equ LCD_WF_WF2_MASK, 0xFF0000
.equ LCD_WF_WF2_SHIFT, 16
.equ LCD_WF_WF58_MASK, 0xFF0000
.equ LCD_WF_WF58_SHIFT, 16
.equ LCD_WF_WF30_MASK, 0xFF0000
.equ LCD_WF_WF30_SHIFT, 16
.equ LCD_WF_WF62_MASK, 0xFF0000
.equ LCD_WF_WF62_SHIFT, 16
.equ LCD_WF_WF10_MASK, 0xFF0000
.equ LCD_WF_WF10_SHIFT, 16
.equ LCD_WF_WF63_MASK, 0xFF000000
.equ LCD_WF_WF63_SHIFT, 24
.equ LCD_WF_WF59_MASK, 0xFF000000
.equ LCD_WF_WF59_SHIFT, 24
.equ LCD_WF_WF55_MASK, 0xFF000000
.equ LCD_WF_WF55_SHIFT, 24
.equ LCD_WF_WF3_MASK, 0xFF000000
.equ LCD_WF_WF3_SHIFT, 24
.equ LCD_WF_WF51_MASK, 0xFF000000
.equ LCD_WF_WF51_SHIFT, 24
.equ LCD_WF_WF47_MASK, 0xFF000000
.equ LCD_WF_WF47_SHIFT, 24
.equ LCD_WF_WF43_MASK, 0xFF000000
.equ LCD_WF_WF43_SHIFT, 24
.equ LCD_WF_WF7_MASK, 0xFF000000
.equ LCD_WF_WF7_SHIFT, 24
.equ LCD_WF_WF39_MASK, 0xFF000000
.equ LCD_WF_WF39_SHIFT, 24
.equ LCD_WF_WF35_MASK, 0xFF000000
.equ LCD_WF_WF35_SHIFT, 24
.equ LCD_WF_WF31_MASK, 0xFF000000
.equ LCD_WF_WF31_SHIFT, 24
.equ LCD_WF_WF11_MASK, 0xFF000000
.equ LCD_WF_WF11_SHIFT, 24
.equ LCD_WF_WF27_MASK, 0xFF000000
.equ LCD_WF_WF27_SHIFT, 24
.equ LCD_WF_WF23_MASK, 0xFF000000
.equ LCD_WF_WF23_SHIFT, 24
.equ LCD_WF_WF19_MASK, 0xFF000000
.equ LCD_WF_WF19_SHIFT, 24
.equ LCD_WF_WF15_MASK, 0xFF000000
.equ LCD_WF_WF15_SHIFT, 24
//---------------------------------------------------------------
//WFn Custom bit field names (not in C MKL46Z4.h)
//Each byte WFn (n in {x, x+1, x+2, y}) controls an LCD pen
//          bit 7:segment/phase H
//          bit 6:segment/phase G
//          bit 5:segment/phase F
//          bit 4:segment/phase E
//          bit 3:segment/phase D
//          bit 2:segment/phase C
//          bit 1:segment/phase B
//          bit 0:segment/phase A:
//                value 0:  segment off or phase deactivtated
//                value 1:  segment on or phase activated
.equ LCD_WF_A_MASK, 0x01
.equ LCD_WF_A_SHIFT, 0
.equ LCD_WF_B_MASK, 0x02
.equ LCD_WF_B_SHIFT, 1
.equ LCD_WF_C_MASK, 0x04
.equ LCD_WF_C_SHIFT, 2
.equ LCD_WF_D_MASK, 0x08
.equ LCD_WF_D_SHIFT, 3
.equ LCD_WF_E_MASK, 0x10
.equ LCD_WF_E_SHIFT, 4
.equ LCD_WF_F_MASK, 0x20
.equ LCD_WF_F_SHIFT, 5
.equ LCD_WF_G_MASK, 0x40
.equ LCD_WF_G_SHIFT, 6
.equ LCD_WF_H_MASK, 0x80
.equ LCD_WF_H_SHIFT, 7
//---------------------------------------------------------------
///* WF8B Bit Fields */
.equ LCD_WF8B_BPALCD0_MASK, 0x1
.equ LCD_WF8B_BPALCD0_SHIFT, 0
.equ LCD_WF8B_BPALCD63_MASK, 0x1
.equ LCD_WF8B_BPALCD63_SHIFT, 0
.equ LCD_WF8B_BPALCD62_MASK, 0x1
.equ LCD_WF8B_BPALCD62_SHIFT, 0
.equ LCD_WF8B_BPALCD61_MASK, 0x1
.equ LCD_WF8B_BPALCD61_SHIFT, 0
.equ LCD_WF8B_BPALCD60_MASK, 0x1
.equ LCD_WF8B_BPALCD60_SHIFT, 0
.equ LCD_WF8B_BPALCD59_MASK, 0x1
.equ LCD_WF8B_BPALCD59_SHIFT, 0
.equ LCD_WF8B_BPALCD58_MASK, 0x1
.equ LCD_WF8B_BPALCD58_SHIFT, 0
.equ LCD_WF8B_BPALCD57_MASK, 0x1
.equ LCD_WF8B_BPALCD57_SHIFT, 0
.equ LCD_WF8B_BPALCD1_MASK, 0x1
.equ LCD_WF8B_BPALCD1_SHIFT, 0
.equ LCD_WF8B_BPALCD56_MASK, 0x1
.equ LCD_WF8B_BPALCD56_SHIFT, 0
.equ LCD_WF8B_BPALCD55_MASK, 0x1
.equ LCD_WF8B_BPALCD55_SHIFT, 0
.equ LCD_WF8B_BPALCD54_MASK, 0x1
.equ LCD_WF8B_BPALCD54_SHIFT, 0
.equ LCD_WF8B_BPALCD53_MASK, 0x1
.equ LCD_WF8B_BPALCD53_SHIFT, 0
.equ LCD_WF8B_BPALCD52_MASK, 0x1
.equ LCD_WF8B_BPALCD52_SHIFT, 0
.equ LCD_WF8B_BPALCD51_MASK, 0x1
.equ LCD_WF8B_BPALCD51_SHIFT, 0
.equ LCD_WF8B_BPALCD50_MASK, 0x1
.equ LCD_WF8B_BPALCD50_SHIFT, 0
.equ LCD_WF8B_BPALCD2_MASK, 0x1
.equ LCD_WF8B_BPALCD2_SHIFT, 0
.equ LCD_WF8B_BPALCD49_MASK, 0x1
.equ LCD_WF8B_BPALCD49_SHIFT, 0
.equ LCD_WF8B_BPALCD48_MASK, 0x1
.equ LCD_WF8B_BPALCD48_SHIFT, 0
.equ LCD_WF8B_BPALCD47_MASK, 0x1
.equ LCD_WF8B_BPALCD47_SHIFT, 0
.equ LCD_WF8B_BPALCD46_MASK, 0x1
.equ LCD_WF8B_BPALCD46_SHIFT, 0
.equ LCD_WF8B_BPALCD45_MASK, 0x1
.equ LCD_WF8B_BPALCD45_SHIFT, 0
.equ LCD_WF8B_BPALCD44_MASK, 0x1
.equ LCD_WF8B_BPALCD44_SHIFT, 0
.equ LCD_WF8B_BPALCD43_MASK, 0x1
.equ LCD_WF8B_BPALCD43_SHIFT, 0
.equ LCD_WF8B_BPALCD3_MASK, 0x1
.equ LCD_WF8B_BPALCD3_SHIFT, 0
.equ LCD_WF8B_BPALCD42_MASK, 0x1
.equ LCD_WF8B_BPALCD42_SHIFT, 0
.equ LCD_WF8B_BPALCD41_MASK, 0x1
.equ LCD_WF8B_BPALCD41_SHIFT, 0
.equ LCD_WF8B_BPALCD40_MASK, 0x1
.equ LCD_WF8B_BPALCD40_SHIFT, 0
.equ LCD_WF8B_BPALCD39_MASK, 0x1
.equ LCD_WF8B_BPALCD39_SHIFT, 0
.equ LCD_WF8B_BPALCD38_MASK, 0x1
.equ LCD_WF8B_BPALCD38_SHIFT, 0
.equ LCD_WF8B_BPALCD37_MASK, 0x1
.equ LCD_WF8B_BPALCD37_SHIFT, 0
.equ LCD_WF8B_BPALCD36_MASK, 0x1
.equ LCD_WF8B_BPALCD36_SHIFT, 0
.equ LCD_WF8B_BPALCD4_MASK, 0x1
.equ LCD_WF8B_BPALCD4_SHIFT, 0
.equ LCD_WF8B_BPALCD35_MASK, 0x1
.equ LCD_WF8B_BPALCD35_SHIFT, 0
.equ LCD_WF8B_BPALCD34_MASK, 0x1
.equ LCD_WF8B_BPALCD34_SHIFT, 0
.equ LCD_WF8B_BPALCD33_MASK, 0x1
.equ LCD_WF8B_BPALCD33_SHIFT, 0
.equ LCD_WF8B_BPALCD32_MASK, 0x1
.equ LCD_WF8B_BPALCD32_SHIFT, 0
.equ LCD_WF8B_BPALCD31_MASK, 0x1
.equ LCD_WF8B_BPALCD31_SHIFT, 0
.equ LCD_WF8B_BPALCD30_MASK, 0x1
.equ LCD_WF8B_BPALCD30_SHIFT, 0
.equ LCD_WF8B_BPALCD29_MASK, 0x1
.equ LCD_WF8B_BPALCD29_SHIFT, 0
.equ LCD_WF8B_BPALCD5_MASK, 0x1
.equ LCD_WF8B_BPALCD5_SHIFT, 0
.equ LCD_WF8B_BPALCD28_MASK, 0x1
.equ LCD_WF8B_BPALCD28_SHIFT, 0
.equ LCD_WF8B_BPALCD27_MASK, 0x1
.equ LCD_WF8B_BPALCD27_SHIFT, 0
.equ LCD_WF8B_BPALCD26_MASK, 0x1
.equ LCD_WF8B_BPALCD26_SHIFT, 0
.equ LCD_WF8B_BPALCD25_MASK, 0x1
.equ LCD_WF8B_BPALCD25_SHIFT, 0
.equ LCD_WF8B_BPALCD24_MASK, 0x1
.equ LCD_WF8B_BPALCD24_SHIFT, 0
.equ LCD_WF8B_BPALCD23_MASK, 0x1
.equ LCD_WF8B_BPALCD23_SHIFT, 0
.equ LCD_WF8B_BPALCD22_MASK, 0x1
.equ LCD_WF8B_BPALCD22_SHIFT, 0
.equ LCD_WF8B_BPALCD6_MASK, 0x1
.equ LCD_WF8B_BPALCD6_SHIFT, 0
.equ LCD_WF8B_BPALCD21_MASK, 0x1
.equ LCD_WF8B_BPALCD21_SHIFT, 0
.equ LCD_WF8B_BPALCD20_MASK, 0x1
.equ LCD_WF8B_BPALCD20_SHIFT, 0
.equ LCD_WF8B_BPALCD19_MASK, 0x1
.equ LCD_WF8B_BPALCD19_SHIFT, 0
.equ LCD_WF8B_BPALCD18_MASK, 0x1
.equ LCD_WF8B_BPALCD18_SHIFT, 0
.equ LCD_WF8B_BPALCD17_MASK, 0x1
.equ LCD_WF8B_BPALCD17_SHIFT, 0
.equ LCD_WF8B_BPALCD16_MASK, 0x1
.equ LCD_WF8B_BPALCD16_SHIFT, 0
.equ LCD_WF8B_BPALCD15_MASK, 0x1
.equ LCD_WF8B_BPALCD15_SHIFT, 0
.equ LCD_WF8B_BPALCD7_MASK, 0x1
.equ LCD_WF8B_BPALCD7_SHIFT, 0
.equ LCD_WF8B_BPALCD14_MASK, 0x1
.equ LCD_WF8B_BPALCD14_SHIFT, 0
.equ LCD_WF8B_BPALCD13_MASK, 0x1
.equ LCD_WF8B_BPALCD13_SHIFT, 0
.equ LCD_WF8B_BPALCD12_MASK, 0x1
.equ LCD_WF8B_BPALCD12_SHIFT, 0
.equ LCD_WF8B_BPALCD11_MASK, 0x1
.equ LCD_WF8B_BPALCD11_SHIFT, 0
.equ LCD_WF8B_BPALCD10_MASK, 0x1
.equ LCD_WF8B_BPALCD10_SHIFT, 0
.equ LCD_WF8B_BPALCD9_MASK, 0x1
.equ LCD_WF8B_BPALCD9_SHIFT, 0
.equ LCD_WF8B_BPALCD8_MASK, 0x1
.equ LCD_WF8B_BPALCD8_SHIFT, 0
.equ LCD_WF8B_BPBLCD1_MASK, 0x2
.equ LCD_WF8B_BPBLCD1_SHIFT, 1
.equ LCD_WF8B_BPBLCD32_MASK, 0x2
.equ LCD_WF8B_BPBLCD32_SHIFT, 1
.equ LCD_WF8B_BPBLCD30_MASK, 0x2
.equ LCD_WF8B_BPBLCD30_SHIFT, 1
.equ LCD_WF8B_BPBLCD60_MASK, 0x2
.equ LCD_WF8B_BPBLCD60_SHIFT, 1
.equ LCD_WF8B_BPBLCD24_MASK, 0x2
.equ LCD_WF8B_BPBLCD24_SHIFT, 1
.equ LCD_WF8B_BPBLCD28_MASK, 0x2
.equ LCD_WF8B_BPBLCD28_SHIFT, 1
.equ LCD_WF8B_BPBLCD23_MASK, 0x2
.equ LCD_WF8B_BPBLCD23_SHIFT, 1
.equ LCD_WF8B_BPBLCD48_MASK, 0x2
.equ LCD_WF8B_BPBLCD48_SHIFT, 1
.equ LCD_WF8B_BPBLCD10_MASK, 0x2
.equ LCD_WF8B_BPBLCD10_SHIFT, 1
.equ LCD_WF8B_BPBLCD15_MASK, 0x2
.equ LCD_WF8B_BPBLCD15_SHIFT, 1
.equ LCD_WF8B_BPBLCD36_MASK, 0x2
.equ LCD_WF8B_BPBLCD36_SHIFT, 1
.equ LCD_WF8B_BPBLCD44_MASK, 0x2
.equ LCD_WF8B_BPBLCD44_SHIFT, 1
.equ LCD_WF8B_BPBLCD62_MASK, 0x2
.equ LCD_WF8B_BPBLCD62_SHIFT, 1
.equ LCD_WF8B_BPBLCD53_MASK, 0x2
.equ LCD_WF8B_BPBLCD53_SHIFT, 1
.equ LCD_WF8B_BPBLCD22_MASK, 0x2
.equ LCD_WF8B_BPBLCD22_SHIFT, 1
.equ LCD_WF8B_BPBLCD47_MASK, 0x2
.equ LCD_WF8B_BPBLCD47_SHIFT, 1
.equ LCD_WF8B_BPBLCD33_MASK, 0x2
.equ LCD_WF8B_BPBLCD33_SHIFT, 1
.equ LCD_WF8B_BPBLCD2_MASK, 0x2
.equ LCD_WF8B_BPBLCD2_SHIFT, 1
.equ LCD_WF8B_BPBLCD49_MASK, 0x2
.equ LCD_WF8B_BPBLCD49_SHIFT, 1
.equ LCD_WF8B_BPBLCD0_MASK, 0x2
.equ LCD_WF8B_BPBLCD0_SHIFT, 1
.equ LCD_WF8B_BPBLCD55_MASK, 0x2
.equ LCD_WF8B_BPBLCD55_SHIFT, 1
.equ LCD_WF8B_BPBLCD56_MASK, 0x2
.equ LCD_WF8B_BPBLCD56_SHIFT, 1
.equ LCD_WF8B_BPBLCD21_MASK, 0x2
.equ LCD_WF8B_BPBLCD21_SHIFT, 1
.equ LCD_WF8B_BPBLCD6_MASK, 0x2
.equ LCD_WF8B_BPBLCD6_SHIFT, 1
.equ LCD_WF8B_BPBLCD29_MASK, 0x2
.equ LCD_WF8B_BPBLCD29_SHIFT, 1
.equ LCD_WF8B_BPBLCD25_MASK, 0x2
.equ LCD_WF8B_BPBLCD25_SHIFT, 1
.equ LCD_WF8B_BPBLCD8_MASK, 0x2
.equ LCD_WF8B_BPBLCD8_SHIFT, 1
.equ LCD_WF8B_BPBLCD54_MASK, 0x2
.equ LCD_WF8B_BPBLCD54_SHIFT, 1
.equ LCD_WF8B_BPBLCD38_MASK, 0x2
.equ LCD_WF8B_BPBLCD38_SHIFT, 1
.equ LCD_WF8B_BPBLCD43_MASK, 0x2
.equ LCD_WF8B_BPBLCD43_SHIFT, 1
.equ LCD_WF8B_BPBLCD20_MASK, 0x2
.equ LCD_WF8B_BPBLCD20_SHIFT, 1
.equ LCD_WF8B_BPBLCD9_MASK, 0x2
.equ LCD_WF8B_BPBLCD9_SHIFT, 1
.equ LCD_WF8B_BPBLCD7_MASK, 0x2
.equ LCD_WF8B_BPBLCD7_SHIFT, 1
.equ LCD_WF8B_BPBLCD50_MASK, 0x2
.equ LCD_WF8B_BPBLCD50_SHIFT, 1
.equ LCD_WF8B_BPBLCD40_MASK, 0x2
.equ LCD_WF8B_BPBLCD40_SHIFT, 1
.equ LCD_WF8B_BPBLCD63_MASK, 0x2
.equ LCD_WF8B_BPBLCD63_SHIFT, 1
.equ LCD_WF8B_BPBLCD26_MASK, 0x2
.equ LCD_WF8B_BPBLCD26_SHIFT, 1
.equ LCD_WF8B_BPBLCD12_MASK, 0x2
.equ LCD_WF8B_BPBLCD12_SHIFT, 1
.equ LCD_WF8B_BPBLCD19_MASK, 0x2
.equ LCD_WF8B_BPBLCD19_SHIFT, 1
.equ LCD_WF8B_BPBLCD34_MASK, 0x2
.equ LCD_WF8B_BPBLCD34_SHIFT, 1
.equ LCD_WF8B_BPBLCD39_MASK, 0x2
.equ LCD_WF8B_BPBLCD39_SHIFT, 1
.equ LCD_WF8B_BPBLCD59_MASK, 0x2
.equ LCD_WF8B_BPBLCD59_SHIFT, 1
.equ LCD_WF8B_BPBLCD61_MASK, 0x2
.equ LCD_WF8B_BPBLCD61_SHIFT, 1
.equ LCD_WF8B_BPBLCD37_MASK, 0x2
.equ LCD_WF8B_BPBLCD37_SHIFT, 1
.equ LCD_WF8B_BPBLCD31_MASK, 0x2
.equ LCD_WF8B_BPBLCD31_SHIFT, 1
.equ LCD_WF8B_BPBLCD58_MASK, 0x2
.equ LCD_WF8B_BPBLCD58_SHIFT, 1
.equ LCD_WF8B_BPBLCD18_MASK, 0x2
.equ LCD_WF8B_BPBLCD18_SHIFT, 1
.equ LCD_WF8B_BPBLCD45_MASK, 0x2
.equ LCD_WF8B_BPBLCD45_SHIFT, 1
.equ LCD_WF8B_BPBLCD27_MASK, 0x2
.equ LCD_WF8B_BPBLCD27_SHIFT, 1
.equ LCD_WF8B_BPBLCD14_MASK, 0x2
.equ LCD_WF8B_BPBLCD14_SHIFT, 1
.equ LCD_WF8B_BPBLCD51_MASK, 0x2
.equ LCD_WF8B_BPBLCD51_SHIFT, 1
.equ LCD_WF8B_BPBLCD52_MASK, 0x2
.equ LCD_WF8B_BPBLCD52_SHIFT, 1
.equ LCD_WF8B_BPBLCD4_MASK, 0x2
.equ LCD_WF8B_BPBLCD4_SHIFT, 1
.equ LCD_WF8B_BPBLCD35_MASK, 0x2
.equ LCD_WF8B_BPBLCD35_SHIFT, 1
.equ LCD_WF8B_BPBLCD17_MASK, 0x2
.equ LCD_WF8B_BPBLCD17_SHIFT, 1
.equ LCD_WF8B_BPBLCD41_MASK, 0x2
.equ LCD_WF8B_BPBLCD41_SHIFT, 1
.equ LCD_WF8B_BPBLCD11_MASK, 0x2
.equ LCD_WF8B_BPBLCD11_SHIFT, 1
.equ LCD_WF8B_BPBLCD46_MASK, 0x2
.equ LCD_WF8B_BPBLCD46_SHIFT, 1
.equ LCD_WF8B_BPBLCD57_MASK, 0x2
.equ LCD_WF8B_BPBLCD57_SHIFT, 1
.equ LCD_WF8B_BPBLCD42_MASK, 0x2
.equ LCD_WF8B_BPBLCD42_SHIFT, 1
.equ LCD_WF8B_BPBLCD5_MASK, 0x2
.equ LCD_WF8B_BPBLCD5_SHIFT, 1
.equ LCD_WF8B_BPBLCD3_MASK, 0x2
.equ LCD_WF8B_BPBLCD3_SHIFT, 1
.equ LCD_WF8B_BPBLCD16_MASK, 0x2
.equ LCD_WF8B_BPBLCD16_SHIFT, 1
.equ LCD_WF8B_BPBLCD13_MASK, 0x2
.equ LCD_WF8B_BPBLCD13_SHIFT, 1
.equ LCD_WF8B_BPCLCD10_MASK, 0x4
.equ LCD_WF8B_BPCLCD10_SHIFT, 2
.equ LCD_WF8B_BPCLCD55_MASK, 0x4
.equ LCD_WF8B_BPCLCD55_SHIFT, 2
.equ LCD_WF8B_BPCLCD2_MASK, 0x4
.equ LCD_WF8B_BPCLCD2_SHIFT, 2
.equ LCD_WF8B_BPCLCD23_MASK, 0x4
.equ LCD_WF8B_BPCLCD23_SHIFT, 2
.equ LCD_WF8B_BPCLCD48_MASK, 0x4
.equ LCD_WF8B_BPCLCD48_SHIFT, 2
.equ LCD_WF8B_BPCLCD24_MASK, 0x4
.equ LCD_WF8B_BPCLCD24_SHIFT, 2
.equ LCD_WF8B_BPCLCD60_MASK, 0x4
.equ LCD_WF8B_BPCLCD60_SHIFT, 2
.equ LCD_WF8B_BPCLCD47_MASK, 0x4
.equ LCD_WF8B_BPCLCD47_SHIFT, 2
.equ LCD_WF8B_BPCLCD22_MASK, 0x4
.equ LCD_WF8B_BPCLCD22_SHIFT, 2
.equ LCD_WF8B_BPCLCD8_MASK, 0x4
.equ LCD_WF8B_BPCLCD8_SHIFT, 2
.equ LCD_WF8B_BPCLCD21_MASK, 0x4
.equ LCD_WF8B_BPCLCD21_SHIFT, 2
.equ LCD_WF8B_BPCLCD49_MASK, 0x4
.equ LCD_WF8B_BPCLCD49_SHIFT, 2
.equ LCD_WF8B_BPCLCD25_MASK, 0x4
.equ LCD_WF8B_BPCLCD25_SHIFT, 2
.equ LCD_WF8B_BPCLCD1_MASK, 0x4
.equ LCD_WF8B_BPCLCD1_SHIFT, 2
.equ LCD_WF8B_BPCLCD20_MASK, 0x4
.equ LCD_WF8B_BPCLCD20_SHIFT, 2
.equ LCD_WF8B_BPCLCD50_MASK, 0x4
.equ LCD_WF8B_BPCLCD50_SHIFT, 2
.equ LCD_WF8B_BPCLCD19_MASK, 0x4
.equ LCD_WF8B_BPCLCD19_SHIFT, 2
.equ LCD_WF8B_BPCLCD26_MASK, 0x4
.equ LCD_WF8B_BPCLCD26_SHIFT, 2
.equ LCD_WF8B_BPCLCD59_MASK, 0x4
.equ LCD_WF8B_BPCLCD59_SHIFT, 2
.equ LCD_WF8B_BPCLCD61_MASK, 0x4
.equ LCD_WF8B_BPCLCD61_SHIFT, 2
.equ LCD_WF8B_BPCLCD46_MASK, 0x4
.equ LCD_WF8B_BPCLCD46_SHIFT, 2
.equ LCD_WF8B_BPCLCD18_MASK, 0x4
.equ LCD_WF8B_BPCLCD18_SHIFT, 2
.equ LCD_WF8B_BPCLCD5_MASK, 0x4
.equ LCD_WF8B_BPCLCD5_SHIFT, 2
.equ LCD_WF8B_BPCLCD63_MASK, 0x4
.equ LCD_WF8B_BPCLCD63_SHIFT, 2
.equ LCD_WF8B_BPCLCD27_MASK, 0x4
.equ LCD_WF8B_BPCLCD27_SHIFT, 2
.equ LCD_WF8B_BPCLCD17_MASK, 0x4
.equ LCD_WF8B_BPCLCD17_SHIFT, 2
.equ LCD_WF8B_BPCLCD51_MASK, 0x4
.equ LCD_WF8B_BPCLCD51_SHIFT, 2
.equ LCD_WF8B_BPCLCD9_MASK, 0x4
.equ LCD_WF8B_BPCLCD9_SHIFT, 2
.equ LCD_WF8B_BPCLCD54_MASK, 0x4
.equ LCD_WF8B_BPCLCD54_SHIFT, 2
.equ LCD_WF8B_BPCLCD15_MASK, 0x4
.equ LCD_WF8B_BPCLCD15_SHIFT, 2
.equ LCD_WF8B_BPCLCD16_MASK, 0x4
.equ LCD_WF8B_BPCLCD16_SHIFT, 2
.equ LCD_WF8B_BPCLCD14_MASK, 0x4
.equ LCD_WF8B_BPCLCD14_SHIFT, 2
.equ LCD_WF8B_BPCLCD32_MASK, 0x4
.equ LCD_WF8B_BPCLCD32_SHIFT, 2
.equ LCD_WF8B_BPCLCD28_MASK, 0x4
.equ LCD_WF8B_BPCLCD28_SHIFT, 2
.equ LCD_WF8B_BPCLCD53_MASK, 0x4
.equ LCD_WF8B_BPCLCD53_SHIFT, 2
.equ LCD_WF8B_BPCLCD33_MASK, 0x4
.equ LCD_WF8B_BPCLCD33_SHIFT, 2
.equ LCD_WF8B_BPCLCD0_MASK, 0x4
.equ LCD_WF8B_BPCLCD0_SHIFT, 2
.equ LCD_WF8B_BPCLCD43_MASK, 0x4
.equ LCD_WF8B_BPCLCD43_SHIFT, 2
.equ LCD_WF8B_BPCLCD7_MASK, 0x4
.equ LCD_WF8B_BPCLCD7_SHIFT, 2
.equ LCD_WF8B_BPCLCD4_MASK, 0x4
.equ LCD_WF8B_BPCLCD4_SHIFT, 2
.equ LCD_WF8B_BPCLCD34_MASK, 0x4
.equ LCD_WF8B_BPCLCD34_SHIFT, 2
.equ LCD_WF8B_BPCLCD29_MASK, 0x4
.equ LCD_WF8B_BPCLCD29_SHIFT, 2
.equ LCD_WF8B_BPCLCD45_MASK, 0x4
.equ LCD_WF8B_BPCLCD45_SHIFT, 2
.equ LCD_WF8B_BPCLCD57_MASK, 0x4
.equ LCD_WF8B_BPCLCD57_SHIFT, 2
.equ LCD_WF8B_BPCLCD42_MASK, 0x4
.equ LCD_WF8B_BPCLCD42_SHIFT, 2
.equ LCD_WF8B_BPCLCD35_MASK, 0x4
.equ LCD_WF8B_BPCLCD35_SHIFT, 2
.equ LCD_WF8B_BPCLCD13_MASK, 0x4
.equ LCD_WF8B_BPCLCD13_SHIFT, 2
.equ LCD_WF8B_BPCLCD36_MASK, 0x4
.equ LCD_WF8B_BPCLCD36_SHIFT, 2
.equ LCD_WF8B_BPCLCD30_MASK, 0x4
.equ LCD_WF8B_BPCLCD30_SHIFT, 2
.equ LCD_WF8B_BPCLCD52_MASK, 0x4
.equ LCD_WF8B_BPCLCD52_SHIFT, 2
.equ LCD_WF8B_BPCLCD58_MASK, 0x4
.equ LCD_WF8B_BPCLCD58_SHIFT, 2
.equ LCD_WF8B_BPCLCD41_MASK, 0x4
.equ LCD_WF8B_BPCLCD41_SHIFT, 2
.equ LCD_WF8B_BPCLCD37_MASK, 0x4
.equ LCD_WF8B_BPCLCD37_SHIFT, 2
.equ LCD_WF8B_BPCLCD3_MASK, 0x4
.equ LCD_WF8B_BPCLCD3_SHIFT, 2
.equ LCD_WF8B_BPCLCD12_MASK, 0x4
.equ LCD_WF8B_BPCLCD12_SHIFT, 2
.equ LCD_WF8B_BPCLCD11_MASK, 0x4
.equ LCD_WF8B_BPCLCD11_SHIFT, 2
.equ LCD_WF8B_BPCLCD38_MASK, 0x4
.equ LCD_WF8B_BPCLCD38_SHIFT, 2
.equ LCD_WF8B_BPCLCD44_MASK, 0x4
.equ LCD_WF8B_BPCLCD44_SHIFT, 2
.equ LCD_WF8B_BPCLCD31_MASK, 0x4
.equ LCD_WF8B_BPCLCD31_SHIFT, 2
.equ LCD_WF8B_BPCLCD40_MASK, 0x4
.equ LCD_WF8B_BPCLCD40_SHIFT, 2
.equ LCD_WF8B_BPCLCD62_MASK, 0x4
.equ LCD_WF8B_BPCLCD62_SHIFT, 2
.equ LCD_WF8B_BPCLCD56_MASK, 0x4
.equ LCD_WF8B_BPCLCD56_SHIFT, 2
.equ LCD_WF8B_BPCLCD39_MASK, 0x4
.equ LCD_WF8B_BPCLCD39_SHIFT, 2
.equ LCD_WF8B_BPCLCD6_MASK, 0x4
.equ LCD_WF8B_BPCLCD6_SHIFT, 2
.equ LCD_WF8B_BPDLCD47_MASK, 0x8
.equ LCD_WF8B_BPDLCD47_SHIFT, 3
.equ LCD_WF8B_BPDLCD23_MASK, 0x8
.equ LCD_WF8B_BPDLCD23_SHIFT, 3
.equ LCD_WF8B_BPDLCD48_MASK, 0x8
.equ LCD_WF8B_BPDLCD48_SHIFT, 3
.equ LCD_WF8B_BPDLCD24_MASK, 0x8
.equ LCD_WF8B_BPDLCD24_SHIFT, 3
.equ LCD_WF8B_BPDLCD15_MASK, 0x8
.equ LCD_WF8B_BPDLCD15_SHIFT, 3
.equ LCD_WF8B_BPDLCD22_MASK, 0x8
.equ LCD_WF8B_BPDLCD22_SHIFT, 3
.equ LCD_WF8B_BPDLCD60_MASK, 0x8
.equ LCD_WF8B_BPDLCD60_SHIFT, 3
.equ LCD_WF8B_BPDLCD10_MASK, 0x8
.equ LCD_WF8B_BPDLCD10_SHIFT, 3
.equ LCD_WF8B_BPDLCD21_MASK, 0x8
.equ LCD_WF8B_BPDLCD21_SHIFT, 3
.equ LCD_WF8B_BPDLCD49_MASK, 0x8
.equ LCD_WF8B_BPDLCD49_SHIFT, 3
.equ LCD_WF8B_BPDLCD1_MASK, 0x8
.equ LCD_WF8B_BPDLCD1_SHIFT, 3
.equ LCD_WF8B_BPDLCD25_MASK, 0x8
.equ LCD_WF8B_BPDLCD25_SHIFT, 3
.equ LCD_WF8B_BPDLCD20_MASK, 0x8
.equ LCD_WF8B_BPDLCD20_SHIFT, 3
.equ LCD_WF8B_BPDLCD2_MASK, 0x8
.equ LCD_WF8B_BPDLCD2_SHIFT, 3
.equ LCD_WF8B_BPDLCD55_MASK, 0x8
.equ LCD_WF8B_BPDLCD55_SHIFT, 3
.equ LCD_WF8B_BPDLCD59_MASK, 0x8
.equ LCD_WF8B_BPDLCD59_SHIFT, 3
.equ LCD_WF8B_BPDLCD5_MASK, 0x8
.equ LCD_WF8B_BPDLCD5_SHIFT, 3
.equ LCD_WF8B_BPDLCD19_MASK, 0x8
.equ LCD_WF8B_BPDLCD19_SHIFT, 3
.equ LCD_WF8B_BPDLCD6_MASK, 0x8
.equ LCD_WF8B_BPDLCD6_SHIFT, 3
.equ LCD_WF8B_BPDLCD26_MASK, 0x8
.equ LCD_WF8B_BPDLCD26_SHIFT, 3
.equ LCD_WF8B_BPDLCD0_MASK, 0x8
.equ LCD_WF8B_BPDLCD0_SHIFT, 3
.equ LCD_WF8B_BPDLCD50_MASK, 0x8
.equ LCD_WF8B_BPDLCD50_SHIFT, 3
.equ LCD_WF8B_BPDLCD46_MASK, 0x8
.equ LCD_WF8B_BPDLCD46_SHIFT, 3
.equ LCD_WF8B_BPDLCD18_MASK, 0x8
.equ LCD_WF8B_BPDLCD18_SHIFT, 3
.equ LCD_WF8B_BPDLCD61_MASK, 0x8
.equ LCD_WF8B_BPDLCD61_SHIFT, 3
.equ LCD_WF8B_BPDLCD9_MASK, 0x8
.equ LCD_WF8B_BPDLCD9_SHIFT, 3
.equ LCD_WF8B_BPDLCD17_MASK, 0x8
.equ LCD_WF8B_BPDLCD17_SHIFT, 3
.equ LCD_WF8B_BPDLCD27_MASK, 0x8
.equ LCD_WF8B_BPDLCD27_SHIFT, 3
.equ LCD_WF8B_BPDLCD53_MASK, 0x8
.equ LCD_WF8B_BPDLCD53_SHIFT, 3
.equ LCD_WF8B_BPDLCD51_MASK, 0x8
.equ LCD_WF8B_BPDLCD51_SHIFT, 3
.equ LCD_WF8B_BPDLCD54_MASK, 0x8
.equ LCD_WF8B_BPDLCD54_SHIFT, 3
.equ LCD_WF8B_BPDLCD13_MASK, 0x8
.equ LCD_WF8B_BPDLCD13_SHIFT, 3
.equ LCD_WF8B_BPDLCD16_MASK, 0x8
.equ LCD_WF8B_BPDLCD16_SHIFT, 3
.equ LCD_WF8B_BPDLCD32_MASK, 0x8
.equ LCD_WF8B_BPDLCD32_SHIFT, 3
.equ LCD_WF8B_BPDLCD14_MASK, 0x8
.equ LCD_WF8B_BPDLCD14_SHIFT, 3
.equ LCD_WF8B_BPDLCD28_MASK, 0x8
.equ LCD_WF8B_BPDLCD28_SHIFT, 3
.equ LCD_WF8B_BPDLCD43_MASK, 0x8
.equ LCD_WF8B_BPDLCD43_SHIFT, 3
.equ LCD_WF8B_BPDLCD4_MASK, 0x8
.equ LCD_WF8B_BPDLCD4_SHIFT, 3
.equ LCD_WF8B_BPDLCD45_MASK, 0x8
.equ LCD_WF8B_BPDLCD45_SHIFT, 3
.equ LCD_WF8B_BPDLCD8_MASK, 0x8
.equ LCD_WF8B_BPDLCD8_SHIFT, 3
.equ LCD_WF8B_BPDLCD62_MASK, 0x8
.equ LCD_WF8B_BPDLCD62_SHIFT, 3
.equ LCD_WF8B_BPDLCD33_MASK, 0x8
.equ LCD_WF8B_BPDLCD33_SHIFT, 3
.equ LCD_WF8B_BPDLCD34_MASK, 0x8
.equ LCD_WF8B_BPDLCD34_SHIFT, 3
.equ LCD_WF8B_BPDLCD29_MASK, 0x8
.equ LCD_WF8B_BPDLCD29_SHIFT, 3
.equ LCD_WF8B_BPDLCD58_MASK, 0x8
.equ LCD_WF8B_BPDLCD58_SHIFT, 3
.equ LCD_WF8B_BPDLCD57_MASK, 0x8
.equ LCD_WF8B_BPDLCD57_SHIFT, 3
.equ LCD_WF8B_BPDLCD42_MASK, 0x8
.equ LCD_WF8B_BPDLCD42_SHIFT, 3
.equ LCD_WF8B_BPDLCD35_MASK, 0x8
.equ LCD_WF8B_BPDLCD35_SHIFT, 3
.equ LCD_WF8B_BPDLCD52_MASK, 0x8
.equ LCD_WF8B_BPDLCD52_SHIFT, 3
.equ LCD_WF8B_BPDLCD7_MASK, 0x8
.equ LCD_WF8B_BPDLCD7_SHIFT, 3
.equ LCD_WF8B_BPDLCD36_MASK, 0x8
.equ LCD_WF8B_BPDLCD36_SHIFT, 3
.equ LCD_WF8B_BPDLCD30_MASK, 0x8
.equ LCD_WF8B_BPDLCD30_SHIFT, 3
.equ LCD_WF8B_BPDLCD41_MASK, 0x8
.equ LCD_WF8B_BPDLCD41_SHIFT, 3
.equ LCD_WF8B_BPDLCD37_MASK, 0x8
.equ LCD_WF8B_BPDLCD37_SHIFT, 3
.equ LCD_WF8B_BPDLCD44_MASK, 0x8
.equ LCD_WF8B_BPDLCD44_SHIFT, 3
.equ LCD_WF8B_BPDLCD63_MASK, 0x8
.equ LCD_WF8B_BPDLCD63_SHIFT, 3
.equ LCD_WF8B_BPDLCD38_MASK, 0x8
.equ LCD_WF8B_BPDLCD38_SHIFT, 3
.equ LCD_WF8B_BPDLCD56_MASK, 0x8
.equ LCD_WF8B_BPDLCD56_SHIFT, 3
.equ LCD_WF8B_BPDLCD40_MASK, 0x8
.equ LCD_WF8B_BPDLCD40_SHIFT, 3
.equ LCD_WF8B_BPDLCD31_MASK, 0x8
.equ LCD_WF8B_BPDLCD31_SHIFT, 3
.equ LCD_WF8B_BPDLCD12_MASK, 0x8
.equ LCD_WF8B_BPDLCD12_SHIFT, 3
.equ LCD_WF8B_BPDLCD39_MASK, 0x8
.equ LCD_WF8B_BPDLCD39_SHIFT, 3
.equ LCD_WF8B_BPDLCD3_MASK, 0x8
.equ LCD_WF8B_BPDLCD3_SHIFT, 3
.equ LCD_WF8B_BPDLCD11_MASK, 0x8
.equ LCD_WF8B_BPDLCD11_SHIFT, 3
.equ LCD_WF8B_BPELCD12_MASK, 0x10
.equ LCD_WF8B_BPELCD12_SHIFT, 4
.equ LCD_WF8B_BPELCD39_MASK, 0x10
.equ LCD_WF8B_BPELCD39_SHIFT, 4
.equ LCD_WF8B_BPELCD3_MASK, 0x10
.equ LCD_WF8B_BPELCD3_SHIFT, 4
.equ LCD_WF8B_BPELCD38_MASK, 0x10
.equ LCD_WF8B_BPELCD38_SHIFT, 4
.equ LCD_WF8B_BPELCD40_MASK, 0x10
.equ LCD_WF8B_BPELCD40_SHIFT, 4
.equ LCD_WF8B_BPELCD37_MASK, 0x10
.equ LCD_WF8B_BPELCD37_SHIFT, 4
.equ LCD_WF8B_BPELCD41_MASK, 0x10
.equ LCD_WF8B_BPELCD41_SHIFT, 4
.equ LCD_WF8B_BPELCD36_MASK, 0x10
.equ LCD_WF8B_BPELCD36_SHIFT, 4
.equ LCD_WF8B_BPELCD8_MASK, 0x10
.equ LCD_WF8B_BPELCD8_SHIFT, 4
.equ LCD_WF8B_BPELCD35_MASK, 0x10
.equ LCD_WF8B_BPELCD35_SHIFT, 4
.equ LCD_WF8B_BPELCD42_MASK, 0x10
.equ LCD_WF8B_BPELCD42_SHIFT, 4
.equ LCD_WF8B_BPELCD34_MASK, 0x10
.equ LCD_WF8B_BPELCD34_SHIFT, 4
.equ LCD_WF8B_BPELCD33_MASK, 0x10
.equ LCD_WF8B_BPELCD33_SHIFT, 4
.equ LCD_WF8B_BPELCD11_MASK, 0x10
.equ LCD_WF8B_BPELCD11_SHIFT, 4
.equ LCD_WF8B_BPELCD43_MASK, 0x10
.equ LCD_WF8B_BPELCD43_SHIFT, 4
.equ LCD_WF8B_BPELCD32_MASK, 0x10
.equ LCD_WF8B_BPELCD32_SHIFT, 4
.equ LCD_WF8B_BPELCD31_MASK, 0x10
.equ LCD_WF8B_BPELCD31_SHIFT, 4
.equ LCD_WF8B_BPELCD44_MASK, 0x10
.equ LCD_WF8B_BPELCD44_SHIFT, 4
.equ LCD_WF8B_BPELCD30_MASK, 0x10
.equ LCD_WF8B_BPELCD30_SHIFT, 4
.equ LCD_WF8B_BPELCD29_MASK, 0x10
.equ LCD_WF8B_BPELCD29_SHIFT, 4
.equ LCD_WF8B_BPELCD7_MASK, 0x10
.equ LCD_WF8B_BPELCD7_SHIFT, 4
.equ LCD_WF8B_BPELCD45_MASK, 0x10
.equ LCD_WF8B_BPELCD45_SHIFT, 4
.equ LCD_WF8B_BPELCD28_MASK, 0x10
.equ LCD_WF8B_BPELCD28_SHIFT, 4
.equ LCD_WF8B_BPELCD2_MASK, 0x10
.equ LCD_WF8B_BPELCD2_SHIFT, 4
.equ LCD_WF8B_BPELCD27_MASK, 0x10
.equ LCD_WF8B_BPELCD27_SHIFT, 4
.equ LCD_WF8B_BPELCD46_MASK, 0x10
.equ LCD_WF8B_BPELCD46_SHIFT, 4
.equ LCD_WF8B_BPELCD26_MASK, 0x10
.equ LCD_WF8B_BPELCD26_SHIFT, 4
.equ LCD_WF8B_BPELCD10_MASK, 0x10
.equ LCD_WF8B_BPELCD10_SHIFT, 4
.equ LCD_WF8B_BPELCD13_MASK, 0x10
.equ LCD_WF8B_BPELCD13_SHIFT, 4
.equ LCD_WF8B_BPELCD25_MASK, 0x10
.equ LCD_WF8B_BPELCD25_SHIFT, 4
.equ LCD_WF8B_BPELCD5_MASK, 0x10
.equ LCD_WF8B_BPELCD5_SHIFT, 4
.equ LCD_WF8B_BPELCD24_MASK, 0x10
.equ LCD_WF8B_BPELCD24_SHIFT, 4
.equ LCD_WF8B_BPELCD47_MASK, 0x10
.equ LCD_WF8B_BPELCD47_SHIFT, 4
.equ LCD_WF8B_BPELCD23_MASK, 0x10
.equ LCD_WF8B_BPELCD23_SHIFT, 4
.equ LCD_WF8B_BPELCD22_MASK, 0x10
.equ LCD_WF8B_BPELCD22_SHIFT, 4
.equ LCD_WF8B_BPELCD48_MASK, 0x10
.equ LCD_WF8B_BPELCD48_SHIFT, 4
.equ LCD_WF8B_BPELCD21_MASK, 0x10
.equ LCD_WF8B_BPELCD21_SHIFT, 4
.equ LCD_WF8B_BPELCD49_MASK, 0x10
.equ LCD_WF8B_BPELCD49_SHIFT, 4
.equ LCD_WF8B_BPELCD20_MASK, 0x10
.equ LCD_WF8B_BPELCD20_SHIFT, 4
.equ LCD_WF8B_BPELCD19_MASK, 0x10
.equ LCD_WF8B_BPELCD19_SHIFT, 4
.equ LCD_WF8B_BPELCD9_MASK, 0x10
.equ LCD_WF8B_BPELCD9_SHIFT, 4
.equ LCD_WF8B_BPELCD50_MASK, 0x10
.equ LCD_WF8B_BPELCD50_SHIFT, 4
.equ LCD_WF8B_BPELCD18_MASK, 0x10
.equ LCD_WF8B_BPELCD18_SHIFT, 4
.equ LCD_WF8B_BPELCD6_MASK, 0x10
.equ LCD_WF8B_BPELCD6_SHIFT, 4
.equ LCD_WF8B_BPELCD17_MASK, 0x10
.equ LCD_WF8B_BPELCD17_SHIFT, 4
.equ LCD_WF8B_BPELCD51_MASK, 0x10
.equ LCD_WF8B_BPELCD51_SHIFT, 4
.equ LCD_WF8B_BPELCD16_MASK, 0x10
.equ LCD_WF8B_BPELCD16_SHIFT, 4
.equ LCD_WF8B_BPELCD56_MASK, 0x10
.equ LCD_WF8B_BPELCD56_SHIFT, 4
.equ LCD_WF8B_BPELCD57_MASK, 0x10
.equ LCD_WF8B_BPELCD57_SHIFT, 4
.equ LCD_WF8B_BPELCD52_MASK, 0x10
.equ LCD_WF8B_BPELCD52_SHIFT, 4
.equ LCD_WF8B_BPELCD1_MASK, 0x10
.equ LCD_WF8B_BPELCD1_SHIFT, 4
.equ LCD_WF8B_BPELCD58_MASK, 0x10
.equ LCD_WF8B_BPELCD58_SHIFT, 4
.equ LCD_WF8B_BPELCD59_MASK, 0x10
.equ LCD_WF8B_BPELCD59_SHIFT, 4
.equ LCD_WF8B_BPELCD53_MASK, 0x10
.equ LCD_WF8B_BPELCD53_SHIFT, 4
.equ LCD_WF8B_BPELCD14_MASK, 0x10
.equ LCD_WF8B_BPELCD14_SHIFT, 4
.equ LCD_WF8B_BPELCD0_MASK, 0x10
.equ LCD_WF8B_BPELCD0_SHIFT, 4
.equ LCD_WF8B_BPELCD60_MASK, 0x10
.equ LCD_WF8B_BPELCD60_SHIFT, 4
.equ LCD_WF8B_BPELCD15_MASK, 0x10
.equ LCD_WF8B_BPELCD15_SHIFT, 4
.equ LCD_WF8B_BPELCD61_MASK, 0x10
.equ LCD_WF8B_BPELCD61_SHIFT, 4
.equ LCD_WF8B_BPELCD54_MASK, 0x10
.equ LCD_WF8B_BPELCD54_SHIFT, 4
.equ LCD_WF8B_BPELCD62_MASK, 0x10
.equ LCD_WF8B_BPELCD62_SHIFT, 4
.equ LCD_WF8B_BPELCD63_MASK, 0x10
.equ LCD_WF8B_BPELCD63_SHIFT, 4
.equ LCD_WF8B_BPELCD55_MASK, 0x10
.equ LCD_WF8B_BPELCD55_SHIFT, 4
.equ LCD_WF8B_BPELCD4_MASK, 0x10
.equ LCD_WF8B_BPELCD4_SHIFT, 4
.equ LCD_WF8B_BPFLCD13_MASK, 0x20
.equ LCD_WF8B_BPFLCD13_SHIFT, 5
.equ LCD_WF8B_BPFLCD39_MASK, 0x20
.equ LCD_WF8B_BPFLCD39_SHIFT, 5
.equ LCD_WF8B_BPFLCD55_MASK, 0x20
.equ LCD_WF8B_BPFLCD55_SHIFT, 5
.equ LCD_WF8B_BPFLCD47_MASK, 0x20
.equ LCD_WF8B_BPFLCD47_SHIFT, 5
.equ LCD_WF8B_BPFLCD63_MASK, 0x20
.equ LCD_WF8B_BPFLCD63_SHIFT, 5
.equ LCD_WF8B_BPFLCD43_MASK, 0x20
.equ LCD_WF8B_BPFLCD43_SHIFT, 5
.equ LCD_WF8B_BPFLCD5_MASK, 0x20
.equ LCD_WF8B_BPFLCD5_SHIFT, 5
.equ LCD_WF8B_BPFLCD62_MASK, 0x20
.equ LCD_WF8B_BPFLCD62_SHIFT, 5
.equ LCD_WF8B_BPFLCD14_MASK, 0x20
.equ LCD_WF8B_BPFLCD14_SHIFT, 5
.equ LCD_WF8B_BPFLCD24_MASK, 0x20
.equ LCD_WF8B_BPFLCD24_SHIFT, 5
.equ LCD_WF8B_BPFLCD54_MASK, 0x20
.equ LCD_WF8B_BPFLCD54_SHIFT, 5
.equ LCD_WF8B_BPFLCD15_MASK, 0x20
.equ LCD_WF8B_BPFLCD15_SHIFT, 5
.equ LCD_WF8B_BPFLCD32_MASK, 0x20
.equ LCD_WF8B_BPFLCD32_SHIFT, 5
.equ LCD_WF8B_BPFLCD61_MASK, 0x20
.equ LCD_WF8B_BPFLCD61_SHIFT, 5
.equ LCD_WF8B_BPFLCD25_MASK, 0x20
.equ LCD_WF8B_BPFLCD25_SHIFT, 5
.equ LCD_WF8B_BPFLCD60_MASK, 0x20
.equ LCD_WF8B_BPFLCD60_SHIFT, 5
.equ LCD_WF8B_BPFLCD41_MASK, 0x20
.equ LCD_WF8B_BPFLCD41_SHIFT, 5
.equ LCD_WF8B_BPFLCD33_MASK, 0x20
.equ LCD_WF8B_BPFLCD33_SHIFT, 5
.equ LCD_WF8B_BPFLCD53_MASK, 0x20
.equ LCD_WF8B_BPFLCD53_SHIFT, 5
.equ LCD_WF8B_BPFLCD59_MASK, 0x20
.equ LCD_WF8B_BPFLCD59_SHIFT, 5
.equ LCD_WF8B_BPFLCD0_MASK, 0x20
.equ LCD_WF8B_BPFLCD0_SHIFT, 5
.equ LCD_WF8B_BPFLCD46_MASK, 0x20
.equ LCD_WF8B_BPFLCD46_SHIFT, 5
.equ LCD_WF8B_BPFLCD58_MASK, 0x20
.equ LCD_WF8B_BPFLCD58_SHIFT, 5
.equ LCD_WF8B_BPFLCD26_MASK, 0x20
.equ LCD_WF8B_BPFLCD26_SHIFT, 5
.equ LCD_WF8B_BPFLCD36_MASK, 0x20
.equ LCD_WF8B_BPFLCD36_SHIFT, 5
.equ LCD_WF8B_BPFLCD10_MASK, 0x20
.equ LCD_WF8B_BPFLCD10_SHIFT, 5
.equ LCD_WF8B_BPFLCD52_MASK, 0x20
.equ LCD_WF8B_BPFLCD52_SHIFT, 5
.equ LCD_WF8B_BPFLCD57_MASK, 0x20
.equ LCD_WF8B_BPFLCD57_SHIFT, 5
.equ LCD_WF8B_BPFLCD27_MASK, 0x20
.equ LCD_WF8B_BPFLCD27_SHIFT, 5
.equ LCD_WF8B_BPFLCD11_MASK, 0x20
.equ LCD_WF8B_BPFLCD11_SHIFT, 5
.equ LCD_WF8B_BPFLCD56_MASK, 0x20
.equ LCD_WF8B_BPFLCD56_SHIFT, 5
.equ LCD_WF8B_BPFLCD1_MASK, 0x20
.equ LCD_WF8B_BPFLCD1_SHIFT, 5
.equ LCD_WF8B_BPFLCD8_MASK, 0x20
.equ LCD_WF8B_BPFLCD8_SHIFT, 5
.equ LCD_WF8B_BPFLCD40_MASK, 0x20
.equ LCD_WF8B_BPFLCD40_SHIFT, 5
.equ LCD_WF8B_BPFLCD51_MASK, 0x20
.equ LCD_WF8B_BPFLCD51_SHIFT, 5
.equ LCD_WF8B_BPFLCD16_MASK, 0x20
.equ LCD_WF8B_BPFLCD16_SHIFT, 5
.equ LCD_WF8B_BPFLCD45_MASK, 0x20
.equ LCD_WF8B_BPFLCD45_SHIFT, 5
.equ LCD_WF8B_BPFLCD6_MASK, 0x20
.equ LCD_WF8B_BPFLCD6_SHIFT, 5
.equ LCD_WF8B_BPFLCD17_MASK, 0x20
.equ LCD_WF8B_BPFLCD17_SHIFT, 5
.equ LCD_WF8B_BPFLCD28_MASK, 0x20
.equ LCD_WF8B_BPFLCD28_SHIFT, 5
.equ LCD_WF8B_BPFLCD42_MASK, 0x20
.equ LCD_WF8B_BPFLCD42_SHIFT, 5
.equ LCD_WF8B_BPFLCD29_MASK, 0x20
.equ LCD_WF8B_BPFLCD29_SHIFT, 5
.equ LCD_WF8B_BPFLCD50_MASK, 0x20
.equ LCD_WF8B_BPFLCD50_SHIFT, 5
.equ LCD_WF8B_BPFLCD18_MASK, 0x20
.equ LCD_WF8B_BPFLCD18_SHIFT, 5
.equ LCD_WF8B_BPFLCD34_MASK, 0x20
.equ LCD_WF8B_BPFLCD34_SHIFT, 5
.equ LCD_WF8B_BPFLCD19_MASK, 0x20
.equ LCD_WF8B_BPFLCD19_SHIFT, 5
.equ LCD_WF8B_BPFLCD2_MASK, 0x20
.equ LCD_WF8B_BPFLCD2_SHIFT, 5
.equ LCD_WF8B_BPFLCD9_MASK, 0x20
.equ LCD_WF8B_BPFLCD9_SHIFT, 5
.equ LCD_WF8B_BPFLCD3_MASK, 0x20
.equ LCD_WF8B_BPFLCD3_SHIFT, 5
.equ LCD_WF8B_BPFLCD37_MASK, 0x20
.equ LCD_WF8B_BPFLCD37_SHIFT, 5
.equ LCD_WF8B_BPFLCD49_MASK, 0x20
.equ LCD_WF8B_BPFLCD49_SHIFT, 5
.equ LCD_WF8B_BPFLCD20_MASK, 0x20
.equ LCD_WF8B_BPFLCD20_SHIFT, 5
.equ LCD_WF8B_BPFLCD44_MASK, 0x20
.equ LCD_WF8B_BPFLCD44_SHIFT, 5
.equ LCD_WF8B_BPFLCD30_MASK, 0x20
.equ LCD_WF8B_BPFLCD30_SHIFT, 5
.equ LCD_WF8B_BPFLCD21_MASK, 0x20
.equ LCD_WF8B_BPFLCD21_SHIFT, 5
.equ LCD_WF8B_BPFLCD35_MASK, 0x20
.equ LCD_WF8B_BPFLCD35_SHIFT, 5
.equ LCD_WF8B_BPFLCD4_MASK, 0x20
.equ LCD_WF8B_BPFLCD4_SHIFT, 5
.equ LCD_WF8B_BPFLCD31_MASK, 0x20
.equ LCD_WF8B_BPFLCD31_SHIFT, 5
.equ LCD_WF8B_BPFLCD48_MASK, 0x20
.equ LCD_WF8B_BPFLCD48_SHIFT, 5
.equ LCD_WF8B_BPFLCD7_MASK, 0x20
.equ LCD_WF8B_BPFLCD7_SHIFT, 5
.equ LCD_WF8B_BPFLCD22_MASK, 0x20
.equ LCD_WF8B_BPFLCD22_SHIFT, 5
.equ LCD_WF8B_BPFLCD38_MASK, 0x20
.equ LCD_WF8B_BPFLCD38_SHIFT, 5
.equ LCD_WF8B_BPFLCD12_MASK, 0x20
.equ LCD_WF8B_BPFLCD12_SHIFT, 5
.equ LCD_WF8B_BPFLCD23_MASK, 0x20
.equ LCD_WF8B_BPFLCD23_SHIFT, 5
.equ LCD_WF8B_BPGLCD14_MASK, 0x40
.equ LCD_WF8B_BPGLCD14_SHIFT, 6
.equ LCD_WF8B_BPGLCD55_MASK, 0x40
.equ LCD_WF8B_BPGLCD55_SHIFT, 6
.equ LCD_WF8B_BPGLCD63_MASK, 0x40
.equ LCD_WF8B_BPGLCD63_SHIFT, 6
.equ LCD_WF8B_BPGLCD15_MASK, 0x40
.equ LCD_WF8B_BPGLCD15_SHIFT, 6
.equ LCD_WF8B_BPGLCD62_MASK, 0x40
.equ LCD_WF8B_BPGLCD62_SHIFT, 6
.equ LCD_WF8B_BPGLCD54_MASK, 0x40
.equ LCD_WF8B_BPGLCD54_SHIFT, 6
.equ LCD_WF8B_BPGLCD61_MASK, 0x40
.equ LCD_WF8B_BPGLCD61_SHIFT, 6
.equ LCD_WF8B_BPGLCD60_MASK, 0x40
.equ LCD_WF8B_BPGLCD60_SHIFT, 6
.equ LCD_WF8B_BPGLCD59_MASK, 0x40
.equ LCD_WF8B_BPGLCD59_SHIFT, 6
.equ LCD_WF8B_BPGLCD53_MASK, 0x40
.equ LCD_WF8B_BPGLCD53_SHIFT, 6
.equ LCD_WF8B_BPGLCD58_MASK, 0x40
.equ LCD_WF8B_BPGLCD58_SHIFT, 6
.equ LCD_WF8B_BPGLCD0_MASK, 0x40
.equ LCD_WF8B_BPGLCD0_SHIFT, 6
.equ LCD_WF8B_BPGLCD57_MASK, 0x40
.equ LCD_WF8B_BPGLCD57_SHIFT, 6
.equ LCD_WF8B_BPGLCD52_MASK, 0x40
.equ LCD_WF8B_BPGLCD52_SHIFT, 6
.equ LCD_WF8B_BPGLCD7_MASK, 0x40
.equ LCD_WF8B_BPGLCD7_SHIFT, 6
.equ LCD_WF8B_BPGLCD56_MASK, 0x40
.equ LCD_WF8B_BPGLCD56_SHIFT, 6
.equ LCD_WF8B_BPGLCD6_MASK, 0x40
.equ LCD_WF8B_BPGLCD6_SHIFT, 6
.equ LCD_WF8B_BPGLCD51_MASK, 0x40
.equ LCD_WF8B_BPGLCD51_SHIFT, 6
.equ LCD_WF8B_BPGLCD16_MASK, 0x40
.equ LCD_WF8B_BPGLCD16_SHIFT, 6
.equ LCD_WF8B_BPGLCD1_MASK, 0x40
.equ LCD_WF8B_BPGLCD1_SHIFT, 6
.equ LCD_WF8B_BPGLCD17_MASK, 0x40
.equ LCD_WF8B_BPGLCD17_SHIFT, 6
.equ LCD_WF8B_BPGLCD50_MASK, 0x40
.equ LCD_WF8B_BPGLCD50_SHIFT, 6
.equ LCD_WF8B_BPGLCD18_MASK, 0x40
.equ LCD_WF8B_BPGLCD18_SHIFT, 6
.equ LCD_WF8B_BPGLCD19_MASK, 0x40
.equ LCD_WF8B_BPGLCD19_SHIFT, 6
.equ LCD_WF8B_BPGLCD8_MASK, 0x40
.equ LCD_WF8B_BPGLCD8_SHIFT, 6
.equ LCD_WF8B_BPGLCD49_MASK, 0x40
.equ LCD_WF8B_BPGLCD49_SHIFT, 6
.equ LCD_WF8B_BPGLCD20_MASK, 0x40
.equ LCD_WF8B_BPGLCD20_SHIFT, 6
.equ LCD_WF8B_BPGLCD9_MASK, 0x40
.equ LCD_WF8B_BPGLCD9_SHIFT, 6
.equ LCD_WF8B_BPGLCD21_MASK, 0x40
.equ LCD_WF8B_BPGLCD21_SHIFT, 6
.equ LCD_WF8B_BPGLCD13_MASK, 0x40
.equ LCD_WF8B_BPGLCD13_SHIFT, 6
.equ LCD_WF8B_BPGLCD48_MASK, 0x40
.equ LCD_WF8B_BPGLCD48_SHIFT, 6
.equ LCD_WF8B_BPGLCD22_MASK, 0x40
.equ LCD_WF8B_BPGLCD22_SHIFT, 6
.equ LCD_WF8B_BPGLCD5_MASK, 0x40
.equ LCD_WF8B_BPGLCD5_SHIFT, 6
.equ LCD_WF8B_BPGLCD47_MASK, 0x40
.equ LCD_WF8B_BPGLCD47_SHIFT, 6
.equ LCD_WF8B_BPGLCD23_MASK, 0x40
.equ LCD_WF8B_BPGLCD23_SHIFT, 6
.equ LCD_WF8B_BPGLCD24_MASK, 0x40
.equ LCD_WF8B_BPGLCD24_SHIFT, 6
.equ LCD_WF8B_BPGLCD25_MASK, 0x40
.equ LCD_WF8B_BPGLCD25_SHIFT, 6
.equ LCD_WF8B_BPGLCD46_MASK, 0x40
.equ LCD_WF8B_BPGLCD46_SHIFT, 6
.equ LCD_WF8B_BPGLCD26_MASK, 0x40
.equ LCD_WF8B_BPGLCD26_SHIFT, 6
.equ LCD_WF8B_BPGLCD27_MASK, 0x40
.equ LCD_WF8B_BPGLCD27_SHIFT, 6
.equ LCD_WF8B_BPGLCD10_MASK, 0x40
.equ LCD_WF8B_BPGLCD10_SHIFT, 6
.equ LCD_WF8B_BPGLCD45_MASK, 0x40
.equ LCD_WF8B_BPGLCD45_SHIFT, 6
.equ LCD_WF8B_BPGLCD28_MASK, 0x40
.equ LCD_WF8B_BPGLCD28_SHIFT, 6
.equ LCD_WF8B_BPGLCD29_MASK, 0x40
.equ LCD_WF8B_BPGLCD29_SHIFT, 6
.equ LCD_WF8B_BPGLCD4_MASK, 0x40
.equ LCD_WF8B_BPGLCD4_SHIFT, 6
.equ LCD_WF8B_BPGLCD44_MASK, 0x40
.equ LCD_WF8B_BPGLCD44_SHIFT, 6
.equ LCD_WF8B_BPGLCD30_MASK, 0x40
.equ LCD_WF8B_BPGLCD30_SHIFT, 6
.equ LCD_WF8B_BPGLCD2_MASK, 0x40
.equ LCD_WF8B_BPGLCD2_SHIFT, 6
.equ LCD_WF8B_BPGLCD31_MASK, 0x40
.equ LCD_WF8B_BPGLCD31_SHIFT, 6
.equ LCD_WF8B_BPGLCD43_MASK, 0x40
.equ LCD_WF8B_BPGLCD43_SHIFT, 6
.equ LCD_WF8B_BPGLCD32_MASK, 0x40
.equ LCD_WF8B_BPGLCD32_SHIFT, 6
.equ LCD_WF8B_BPGLCD33_MASK, 0x40
.equ LCD_WF8B_BPGLCD33_SHIFT, 6
.equ LCD_WF8B_BPGLCD42_MASK, 0x40
.equ LCD_WF8B_BPGLCD42_SHIFT, 6
.equ LCD_WF8B_BPGLCD34_MASK, 0x40
.equ LCD_WF8B_BPGLCD34_SHIFT, 6
.equ LCD_WF8B_BPGLCD11_MASK, 0x40
.equ LCD_WF8B_BPGLCD11_SHIFT, 6
.equ LCD_WF8B_BPGLCD35_MASK, 0x40
.equ LCD_WF8B_BPGLCD35_SHIFT, 6
.equ LCD_WF8B_BPGLCD12_MASK, 0x40
.equ LCD_WF8B_BPGLCD12_SHIFT, 6
.equ LCD_WF8B_BPGLCD41_MASK, 0x40
.equ LCD_WF8B_BPGLCD41_SHIFT, 6
.equ LCD_WF8B_BPGLCD36_MASK, 0x40
.equ LCD_WF8B_BPGLCD36_SHIFT, 6
.equ LCD_WF8B_BPGLCD3_MASK, 0x40
.equ LCD_WF8B_BPGLCD3_SHIFT, 6
.equ LCD_WF8B_BPGLCD37_MASK, 0x40
.equ LCD_WF8B_BPGLCD37_SHIFT, 6
.equ LCD_WF8B_BPGLCD40_MASK, 0x40
.equ LCD_WF8B_BPGLCD40_SHIFT, 6
.equ LCD_WF8B_BPGLCD38_MASK, 0x40
.equ LCD_WF8B_BPGLCD38_SHIFT, 6
.equ LCD_WF8B_BPGLCD39_MASK, 0x40
.equ LCD_WF8B_BPGLCD39_SHIFT, 6
.equ LCD_WF8B_BPHLCD63_MASK, 0x80
.equ LCD_WF8B_BPHLCD63_SHIFT, 7
.equ LCD_WF8B_BPHLCD62_MASK, 0x80
.equ LCD_WF8B_BPHLCD62_SHIFT, 7
.equ LCD_WF8B_BPHLCD61_MASK, 0x80
.equ LCD_WF8B_BPHLCD61_SHIFT, 7
.equ LCD_WF8B_BPHLCD60_MASK, 0x80
.equ LCD_WF8B_BPHLCD60_SHIFT, 7
.equ LCD_WF8B_BPHLCD59_MASK, 0x80
.equ LCD_WF8B_BPHLCD59_SHIFT, 7
.equ LCD_WF8B_BPHLCD58_MASK, 0x80
.equ LCD_WF8B_BPHLCD58_SHIFT, 7
.equ LCD_WF8B_BPHLCD57_MASK, 0x80
.equ LCD_WF8B_BPHLCD57_SHIFT, 7
.equ LCD_WF8B_BPHLCD0_MASK, 0x80
.equ LCD_WF8B_BPHLCD0_SHIFT, 7
.equ LCD_WF8B_BPHLCD56_MASK, 0x80
.equ LCD_WF8B_BPHLCD56_SHIFT, 7
.equ LCD_WF8B_BPHLCD55_MASK, 0x80
.equ LCD_WF8B_BPHLCD55_SHIFT, 7
.equ LCD_WF8B_BPHLCD54_MASK, 0x80
.equ LCD_WF8B_BPHLCD54_SHIFT, 7
.equ LCD_WF8B_BPHLCD53_MASK, 0x80
.equ LCD_WF8B_BPHLCD53_SHIFT, 7
.equ LCD_WF8B_BPHLCD52_MASK, 0x80
.equ LCD_WF8B_BPHLCD52_SHIFT, 7
.equ LCD_WF8B_BPHLCD51_MASK, 0x80
.equ LCD_WF8B_BPHLCD51_SHIFT, 7
.equ LCD_WF8B_BPHLCD50_MASK, 0x80
.equ LCD_WF8B_BPHLCD50_SHIFT, 7
.equ LCD_WF8B_BPHLCD1_MASK, 0x80
.equ LCD_WF8B_BPHLCD1_SHIFT, 7
.equ LCD_WF8B_BPHLCD49_MASK, 0x80
.equ LCD_WF8B_BPHLCD49_SHIFT, 7
.equ LCD_WF8B_BPHLCD48_MASK, 0x80
.equ LCD_WF8B_BPHLCD48_SHIFT, 7
.equ LCD_WF8B_BPHLCD47_MASK, 0x80
.equ LCD_WF8B_BPHLCD47_SHIFT, 7
.equ LCD_WF8B_BPHLCD46_MASK, 0x80
.equ LCD_WF8B_BPHLCD46_SHIFT, 7
.equ LCD_WF8B_BPHLCD45_MASK, 0x80
.equ LCD_WF8B_BPHLCD45_SHIFT, 7
.equ LCD_WF8B_BPHLCD44_MASK, 0x80
.equ LCD_WF8B_BPHLCD44_SHIFT, 7
.equ LCD_WF8B_BPHLCD43_MASK, 0x80
.equ LCD_WF8B_BPHLCD43_SHIFT, 7
.equ LCD_WF8B_BPHLCD2_MASK, 0x80
.equ LCD_WF8B_BPHLCD2_SHIFT, 7
.equ LCD_WF8B_BPHLCD42_MASK, 0x80
.equ LCD_WF8B_BPHLCD42_SHIFT, 7
.equ LCD_WF8B_BPHLCD41_MASK, 0x80
.equ LCD_WF8B_BPHLCD41_SHIFT, 7
.equ LCD_WF8B_BPHLCD40_MASK, 0x80
.equ LCD_WF8B_BPHLCD40_SHIFT, 7
.equ LCD_WF8B_BPHLCD39_MASK, 0x80
.equ LCD_WF8B_BPHLCD39_SHIFT, 7
.equ LCD_WF8B_BPHLCD38_MASK, 0x80
.equ LCD_WF8B_BPHLCD38_SHIFT, 7
.equ LCD_WF8B_BPHLCD37_MASK, 0x80
.equ LCD_WF8B_BPHLCD37_SHIFT, 7
.equ LCD_WF8B_BPHLCD36_MASK, 0x80
.equ LCD_WF8B_BPHLCD36_SHIFT, 7
.equ LCD_WF8B_BPHLCD3_MASK, 0x80
.equ LCD_WF8B_BPHLCD3_SHIFT, 7
.equ LCD_WF8B_BPHLCD35_MASK, 0x80
.equ LCD_WF8B_BPHLCD35_SHIFT, 7
.equ LCD_WF8B_BPHLCD34_MASK, 0x80
.equ LCD_WF8B_BPHLCD34_SHIFT, 7
.equ LCD_WF8B_BPHLCD33_MASK, 0x80
.equ LCD_WF8B_BPHLCD33_SHIFT, 7
.equ LCD_WF8B_BPHLCD32_MASK, 0x80
.equ LCD_WF8B_BPHLCD32_SHIFT, 7
.equ LCD_WF8B_BPHLCD31_MASK, 0x80
.equ LCD_WF8B_BPHLCD31_SHIFT, 7
.equ LCD_WF8B_BPHLCD30_MASK, 0x80
.equ LCD_WF8B_BPHLCD30_SHIFT, 7
.equ LCD_WF8B_BPHLCD29_MASK, 0x80
.equ LCD_WF8B_BPHLCD29_SHIFT, 7
.equ LCD_WF8B_BPHLCD4_MASK, 0x80
.equ LCD_WF8B_BPHLCD4_SHIFT, 7
.equ LCD_WF8B_BPHLCD28_MASK, 0x80
.equ LCD_WF8B_BPHLCD28_SHIFT, 7
.equ LCD_WF8B_BPHLCD27_MASK, 0x80
.equ LCD_WF8B_BPHLCD27_SHIFT, 7
.equ LCD_WF8B_BPHLCD26_MASK, 0x80
.equ LCD_WF8B_BPHLCD26_SHIFT, 7
.equ LCD_WF8B_BPHLCD25_MASK, 0x80
.equ LCD_WF8B_BPHLCD25_SHIFT, 7
.equ LCD_WF8B_BPHLCD24_MASK, 0x80
.equ LCD_WF8B_BPHLCD24_SHIFT, 7
.equ LCD_WF8B_BPHLCD23_MASK, 0x80
.equ LCD_WF8B_BPHLCD23_SHIFT, 7
.equ LCD_WF8B_BPHLCD22_MASK, 0x80
.equ LCD_WF8B_BPHLCD22_SHIFT, 7
.equ LCD_WF8B_BPHLCD5_MASK, 0x80
.equ LCD_WF8B_BPHLCD5_SHIFT, 7
.equ LCD_WF8B_BPHLCD21_MASK, 0x80
.equ LCD_WF8B_BPHLCD21_SHIFT, 7
.equ LCD_WF8B_BPHLCD20_MASK, 0x80
.equ LCD_WF8B_BPHLCD20_SHIFT, 7
.equ LCD_WF8B_BPHLCD19_MASK, 0x80
.equ LCD_WF8B_BPHLCD19_SHIFT, 7
.equ LCD_WF8B_BPHLCD18_MASK, 0x80
.equ LCD_WF8B_BPHLCD18_SHIFT, 7
.equ LCD_WF8B_BPHLCD17_MASK, 0x80
.equ LCD_WF8B_BPHLCD17_SHIFT, 7
.equ LCD_WF8B_BPHLCD16_MASK, 0x80
.equ LCD_WF8B_BPHLCD16_SHIFT, 7
.equ LCD_WF8B_BPHLCD15_MASK, 0x80
.equ LCD_WF8B_BPHLCD15_SHIFT, 7
.equ LCD_WF8B_BPHLCD6_MASK, 0x80
.equ LCD_WF8B_BPHLCD6_SHIFT, 7
.equ LCD_WF8B_BPHLCD14_MASK, 0x80
.equ LCD_WF8B_BPHLCD14_SHIFT, 7
.equ LCD_WF8B_BPHLCD13_MASK, 0x80
.equ LCD_WF8B_BPHLCD13_SHIFT, 7
.equ LCD_WF8B_BPHLCD12_MASK, 0x80
.equ LCD_WF8B_BPHLCD12_SHIFT, 7
.equ LCD_WF8B_BPHLCD11_MASK, 0x80
.equ LCD_WF8B_BPHLCD11_SHIFT, 7
.equ LCD_WF8B_BPHLCD10_MASK, 0x80
.equ LCD_WF8B_BPHLCD10_SHIFT, 7
.equ LCD_WF8B_BPHLCD9_MASK, 0x80
.equ LCD_WF8B_BPHLCD9_SHIFT, 7
.equ LCD_WF8B_BPHLCD8_MASK, 0x80
.equ LCD_WF8B_BPHLCD8_SHIFT, 7
.equ LCD_WF8B_BPHLCD7_MASK, 0x80
.equ LCD_WF8B_BPHLCD7_SHIFT, 7
//---------------------------------------------------------------
//Multipurpose clock generator (MCG)
.equ MCG_BASE, 0x40064000
.equ MCG_C1_OFFSET, 0x00
.equ MCG_C2_OFFSET, 0x01
.equ MCG_C4_OFFSET, 0x03
.equ MCG_C5_OFFSET, 0x04
.equ MCG_C6_OFFSET, 0x05
.equ MCG_S_OFFSET, 0x06
.equ MCG_C1, (MCG_BASE + MCG_C1_OFFSET)
.equ MCG_C2, (MCG_BASE + MCG_C2_OFFSET)
.equ MCG_C4, (MCG_BASE + MCG_C4_OFFSET)
.equ MCG_C5, (MCG_BASE + MCG_C5_OFFSET)
.equ MCG_C6, (MCG_BASE + MCG_C6_OFFSET)
.equ MCG_S, (MCG_BASE + MCG_S_OFFSET)
//---------------------------------------------------------------
//MCG_C1 (0x04)
//7-6:CLKS=clock source select (00)
//        :00=output of FLL of PLL (depends on MCG_C6.PLLS)
//        :01=internal reference clock
//        :10=external reference clock
//        :11=(reserved)
//5-3:FRDIV=FLL external reference divider (000)
//    (depends on MCG_C2.RANGE0)
//         :first divider is for RANGE0=0
//         :second divider is for all other RANGE0 values
//         :000=  1 or   32
//         :001=  2 or   64
//         :010=  4 or  128
//         :011=  8 or  256
//         :100= 16 or  512
//         :101= 32 or 1024
//         :110= 64 or 1280
//         :111=128 or 1536
//  2:IREFS=internal reference select (for FLL) (1)
//         :0=external reference clock
//         :1=slow internal reference clock
//  1:IRCLKEN=internal reference clock (MCGIRCLK) enable (0)
//  0:IREFSTEN=internal reference stop enable (0)
.equ MCG_C1_CLKS_MASK, 0xC0
.equ MCG_C1_CLKS_SHIFT, 6
.equ MCG_C1_FRDIV_MASK, 0x38
.equ MCG_C1_FRDIV_SHIFT, 3
.equ MCG_C1_IREFS_MASK, 0x04
.equ MCG_C1_IREFS_SHIFT, 2
.equ MCG_C1_IRCLKEN_MASK, 0x02
.equ MCG_C1_IRCLKEN_SHIFT, 1
.equ MCG_C1_IREFSTEN_MASK, 0x01
.equ MCG_C1_IREFSTEN_SHIFT, 0
//---------------------------------------------------------------
//MCG_C2 (0xC0)
//  7:LOCRE0=loss of clock reset enable (1)
//          :0=interrupt request on loss of OCS0 external reference clock
//          :1=reset request on loss of OCS0 external reference clock
//  6:FCFTRIM=fast internal reference clock fine trim (1)
//5-4:RANGE0=frequency range select (00)
//          :00=low frequency range for crystal oscillator
//          :01=high frequency range for crystal oscillator
//          :1X=very high frequency range for crystal oscillator
//  3:HGO0=high gain oscillator select (0)
//        :0=low-power operation
//        :1=high-gain operation
//  2:EREFS0=external reference select (0)
//          :0=external reference clock
//          :1=oscillator
//  1:LP=low power select (0)
//      :0=FLL or PLL not disabled in bypass modes
//      :1=FLL or PLL disabled in bypass modes (lower power)
//  0:IRCS=internal reference clock select (0)
//        :0=slow internal reference clock
//        :1=fast internal reference clock
.equ MCG_C2_LOCRE0_MASK, 0x80
.equ MCG_C2_LOCRE0_SHIFT, 7
.equ MCG_C2_FCFTRIM_MASK, 0x40
.equ MCG_C2_FCFTRIM_SHIFT, 6
.equ MCG_C2_RANGE0_MASK, 0x30
.equ MCG_C2_RANGE0_SHIFT, 4
.equ MCG_C2_HGO0_MASK, 0x08
.equ MCG_C2_HGO0_SHIFT, 3
.equ MCG_C2_EREFS0_MASK, 0x04
.equ MCG_C2_EREFS0_SHIFT, 2
.equ MCG_C2_LP_MASK, 0x02
.equ MCG_C2_LP_SHIFT, 1
.equ MCG_C2_IRCS_MASK, 0x01
.equ MCG_C2_IRCS_SHIFT, 0
//---------------------------------------------------------------
//MCG_C3 (0xXX)
//7-0:SCTRIM=slow internal reference clock trim setting//
//           on reset, loaded with a factory trim value
.equ MCG_C3_SCTRIM_MASK, 0xFF
.equ MCG_C3_SCTRIM_SHIFT, 0
//---------------------------------------------------------------
//MCG_C4 (2_000XXXXX)
//  7:DMX32=DCO maximum frequency with 32.768 kHz reference (0)
//         :0=default range of 25%
//         :1=fine-tuned for 32.768 kHz reference
//6-5:DRST_DRS=DCO range select (00)
//            :00=low range (default)
//            :01=mid range
//            :10=mid-high range
//            :11=high range
//4-1:FCTRIM=fast internal reference clock trim setting (XXXX)
//           on reset, loaded with a factory trim value
//  0:SCFTRIM=slow internal reference clock fine trim (X)
//           on reset, loaded with a factory trim value
.equ MCG_C4_DMX32_MASK, 0x80
.equ MCG_C4_DMX32_SHIFT, 7
.equ MCG_C4_DRST_DRS_MASK, 0x60
.equ MCG_C4_DRST_DRS_SHIFT, 5
.equ MCG_C4_FCTRIM_MASK, 0x1E
.equ MCG_C4_FCTRIM_SHIFT, 1
.equ MCG_C4_SCFTRIM_MASK, 0x1
.equ MCG_C4_SCFTRIM_SHIFT, 0
//---------------------------------------------------------------
//MCG_C5 (0x00)
//  7:(reserved): read-only// always 0
//  6:PLLCLKEN0=PLL clock (MCGPLLCLK) enable (0)
//  5:PLLSTEN0=PLL stop enable (in normal stop mode) (0)
//4-0:PRDIV0=PLL external reference divider (2_00000)
//          :00000-11000=1-25 (PRDIV0 + 1)
//          :others=(reserved)
.equ MCG_C5_PLLCLKEN0_MASK, 0x40
.equ MCG_C5_PLLCLKEN0_SHIFT, 6
.equ MCG_C5_PLLSTEN0_MASK, 0x20
.equ MCG_C5_PLLSTEN0_SHIFT, 5
.equ MCG_C5_PRDIV0_MASK, 0x1F
.equ MCG_C5_PRDIV0_SHIFT, 0
.equ MCG_C5_PRDIV0_DIV2, 0x01
//---------------------------------------------------------------
//MCG_C6 (0x00)
//  7:LOLIE0=loss of lock interrupt enable (0)
//  6:PLLS=PLL select (0)
//        :0=FLL
//        :1=PLL
//  5:CME0=clock monitor enable (0)
//4-0:VDIV0=VCO 0 divider (2_00000)
//         :Muliply factor for reference clock is VDIV0 + 24
.equ MCG_C6_LOLIE0_MASK, 0x80
.equ MCG_C6_LOLIE0_SHIFT, 7
.equ MCG_C6_PLLS_MASK, 0x40
.equ MCG_C6_PLLS_SHIFT, 6
.equ MCG_C6_CME0_MASK, 0x20
.equ MCG_C6_CME0_SHIFT, 5
.equ MCG_C6_VDIV0_MASK, 0x1F
.equ MCG_C6_VDIV0_SHIFT, 0
//---------------------------------------------------------------
//MCG_S
//  7:LOLS=loss of lock status
//  6:LOCK0=lock status
//  5:PLLST=PLL select status
//         :0=FLL
//         :1=PLL
//  4:IREFST=internal reference status
//          :0=FLL source external
//          :1=FLL source internal
//3-2:CLKST=clock mode status
//         :00=FLL
//         :01=internal reference
//         :10=external reference
//         :11=PLL
//  1:OSCINIT0=OSC initialization (complete)
//  0:IRCST=internal reference clock status
//         :0=slow (32 kHz)
//         :1=fast (4 MHz)
.equ MCG_S_LOLS_MASK, 0x80
.equ MCG_S_LOLS_SHIFT, 7
.equ MCG_S_LOCK0_MASK, 0x40
.equ MCG_S_LOCK0_SHIFT, 6
.equ MCG_S_PLLST_MASK, 0x20
.equ MCG_S_PLLST_SHIFT, 5
.equ MCG_S_IREFST_MASK, 0x10
.equ MCG_S_IREFST_SHIFT, 4
.equ MCG_S_CLKST_MASK, 0x0C
.equ MCG_S_CLKST_SHIFT, 2
.equ MCG_S_OSCINIT0_MASK, 0x02
.equ MCG_S_OSCINIT0_SHIFT, 1
.equ MCG_S_IRCST_MASK, 0x01
.equ MCG_S_IRCST_SHIFT, 0
//---------------------------------------------------------------
//Nested vectored interrupt controller (NVIC)
//Part of system control space (SCS)
.equ NVIC_BASE, 0xE000E100
.equ NVIC_ISER_OFFSET, 0x00
.equ NVIC_ICER_OFFSET, 0x80
.equ NVIC_ISPR_OFFSET, 0x100
.equ NVIC_ICPR_OFFSET, 0x180
.equ NVIC_IPR0_OFFSET, 0x300
.equ NVIC_IPR1_OFFSET, 0x304
.equ NVIC_IPR2_OFFSET, 0x308
.equ NVIC_IPR3_OFFSET, 0x30C
.equ NVIC_IPR4_OFFSET, 0x310
.equ NVIC_IPR5_OFFSET, 0x314
.equ NVIC_IPR6_OFFSET, 0x318
.equ NVIC_IPR7_OFFSET, 0x31C
.equ NVIC_ISER, (NVIC_BASE + NVIC_ISER_OFFSET)
.equ NVIC_ICER, (NVIC_BASE + NVIC_ICER_OFFSET)
.equ NVIC_ISPR, (NVIC_BASE + NVIC_ISPR_OFFSET)
.equ NVIC_ICPR, (NVIC_BASE + NVIC_ICPR_OFFSET)
.equ NVIC_IPR0, (NVIC_BASE + NVIC_IPR0_OFFSET)
.equ NVIC_IPR1, (NVIC_BASE + NVIC_IPR1_OFFSET)
.equ NVIC_IPR2, (NVIC_BASE + NVIC_IPR2_OFFSET)
.equ NVIC_IPR3, (NVIC_BASE + NVIC_IPR3_OFFSET)
.equ NVIC_IPR4, (NVIC_BASE + NVIC_IPR4_OFFSET)
.equ NVIC_IPR5, (NVIC_BASE + NVIC_IPR5_OFFSET)
.equ NVIC_IPR6, (NVIC_BASE + NVIC_IPR6_OFFSET)
.equ NVIC_IPR7, (NVIC_BASE + NVIC_IPR7_OFFSET)
//---------------------------------------------------------------
//NVIC IPR assignments
.equ DMA0_IPR, NVIC_IPR0  //DMA channel 0 xfer complete/error
.equ DMA1_IPR, NVIC_IPR0  //DMA channel 1 xfer complete/error
.equ DMA2_IPR, NVIC_IPR0  //DMA channel 2 xfer complete/error
.equ DMA3_IPR, NVIC_IPR0  //DMA channel 3 xfer complete/error
.equ Reserved20_IPR, NVIC_IPR1  //(reserved)
.equ FTFA_IPR, NVIC_IPR1  //command complete// read collision
.equ PMC_IPR, NVIC_IPR1  //low-voltage detect//low-voltage warning
.equ LLWU_IPR, NVIC_IPR1  //low leakage wakeup
.equ I2C0_IPR, NVIC_IPR2  //I2C0
.equ I2C1_IPR, NVIC_IPR2  //I2C1
.equ SPI0_IPR, NVIC_IPR2  //SPI0 (all IRQ sources)
.equ SPI1_IPR, NVIC_IPR2  //SPI1 (all IRQ sources)
.equ UART0_IPR, NVIC_IPR3  //UART0 (status// error)
.equ UART1_IPR, NVIC_IPR3  //UART1 (status// error)
.equ UART2_IPR, NVIC_IPR3  //UART2 (status// error)
.equ ADC0_IPR, NVIC_IPR3  //ADC0
.equ CMP0_IPR, NVIC_IPR4  //CMP0
.equ TMP0_IPR, NVIC_IPR4  //TPM0
.equ TPM1_IPR, NVIC_IPR4  //TPM1
.equ TPM2_IPR, NVIC_IPR4  //TPM2
.equ RTC_IPR, NVIC_IPR5  //RTC (alarm)
.equ RTC_Seconds_IPR, NVIC_IPR5  //RTC (seconds)
.equ PIT_IPR, NVIC_IPR5  //PIT (all IRQ sources)
.equ I2S0_IPR, NVIC_IPR5  //(reserved)
.equ USB0_IPR, NVIC_IPR6  //USB OTG
.equ DAC0_IPR, NVIC_IPR6  //DAC0
.equ TSI0_IPR, NVIC_IPR6  //TSI0
.equ MCG_IPR, NVIC_IPR6  //MCG
.equ LPTMR0_IPR, NVIC_IPR7  //LPTMR0
.equ LCD_IPR, NVIC_IPR7  //LCD
.equ PORTA_IPR, NVIC_IPR7  //PORTA pin detect
.equ PORTC_PORTD_IPR, NVIC_IPR7  //PORTC and PORTD pin detect
//---------------------------------------------------------------
//NVIC IPR position
//priority is a 2-bit value (0-3)
//.equ position, ates are for LSB of priority
.equ DMA0_PRI_POS, 6  //DMA channel 0 xfer complete/error
.equ DMA1_PRI_POS, 14  //DMA channel 1 xfer complete/error
.equ DMA2_PRI_POS, 22  //DMA channel 2 xfer complete/error
.equ DMA3_PRI_POS, 30  //DMA channel 3 xfer complete/error
.equ Reserved20_PRI_POS, 6  //(reserved)
.equ FTFA_PRI_POS, 14  //command complete// read collision
.equ PMC_PRI_POS, 22  //low-voltage detect//low-voltage warning
.equ LLWU_PRI_POS, 30  //low leakage wakeup
.equ I2C0_PRI_POS, 6  //I2C0
.equ I2C1_PRI_POS, 14  //I2C1
.equ SPI0_PRI_POS, 22  //SPI0 (all IRQ sources)
.equ SPI1_PRI_POS, 30  //SPI1 (all IRQ sources)
.equ UART0_PRI_POS, 6  //UART0 (status// error)
.equ UART1_PRI_POS, 14  //UART1 (status// error)
.equ UART2_PRI_POS, 22  //UART2 (status// error)
.equ ADC0_PRI_POS, 30  //ADC0
.equ CMP0_PRI_POS, 6  //CMP0
.equ TMP0_PRI_POS, 14  //TPM0
.equ TPM1_PRI_POS, 22  //TPM1
.equ TPM2_PRI_POS, 30  //TPM2
.equ RTC_PRI_POS, 6  //RTC (alarm)
.equ RTC_Seconds_PRI_POS, 14  //RTC (seconds)
.equ PIT_PRI_POS, 22  //PIT (all IRQ sources)
.equ I2S0_PRI_POS, 30  //I2S0
.equ USB0_PRI_POS, 6  //USB OTG
.equ DAC0_PRI_POS, 14  //DAC0
.equ TSI0_PRI_POS, 22  //TSI0
.equ MCG_PRI_POS, 30  //MCG
.equ LPTMR0_PRI_POS, 6  //LPTMR0
.equ LCD_PRI_POS, 14  //LCD
.equ PORTA_PRI_POS, 22  //PORTA pin detect
.equ PORTC_PORTD_PRI_POS, 30  //PORTC and PORTD pin detect
//---------------------------------------------------------------
//NVIC IRQ assignments
.equ DMA0_IRQ, 00  //DMA channel 0 xfer complete/error
.equ DMA1_IRQ, 01  //DMA channel 1 xfer complete/error
.equ DMA2_IRQ, 02  //DMA channel 2 xfer complete/error
.equ DMA3_IRQ, 03  //DMA channel 3 xfer complete/error
.equ Reserved20_IRQ, 04  //(reserved)
.equ FTFA_IRQ, 05  //command complete// read collision
.equ PMC_IRQ, 06  //low-voltage detect//low-voltage warning
.equ LLWU_IRQ, 07  //low leakage wakeup
.equ I2C0_IRQ, 8  //I2C0
.equ I2C1_IRQ, 9  //I2C1
.equ SPI0_IRQ, 10  //SPI0 (all IRQ sources)
.equ SPI1_IRQ, 11  //SPI1 (all IRQ sources)
.equ UART0_IRQ, 12  //UART0 (status// error)
.equ UART1_IRQ, 13  //UART1 (status// error)
.equ UART2_IRQ, 14  //UART2 (status// error)
.equ ADC0_IRQ, 15  //ADC0
.equ CMP0_IRQ, 16  //CMP0
.equ TMP0_IRQ, 15  //TPM0
.equ TPM1_IRQ, 18  //TPM1
.equ TPM2_IRQ, 19  //TPM2
.equ RTC_IRQ, 20  //RTC (alarm)
.equ RTC_Seconds_IRQ, 21  //RTC (seconds)
.equ PIT_IRQ, 22  //PIT (all IRQ sources)
.equ I2S0_IRQ, 23  //I2S0
.equ USB0_IRQ, 24  //USB OTG
.equ DAC0_IRQ, 25  //DAC0
.equ TSI0_IRQ, 26  //TSI0
.equ MCG_IRQ, 27  //MCG
.equ LPTMR0_IRQ, 28  //LPTMR0
.equ LCD_IRQ, 29  //LCD
.equ PORTA_IRQ, 30  //PORTA pin detect
.equ PORTC_PORTD_IRQ, 31  //PORTC and PORTD pin detect
//---------------------------------------------------------------
//NVIC IRQ masks for ICER, ISER, ICPR, and ISPR
.equ DMA0_IRQ_MASK, (1 << DMA0_IRQ       )  //DMA channel 0 xfer complete/error
.equ DMA1_IRQ_MASK, (1 << DMA1_IRQ       )  //DMA channel 1 xfer complete/error
.equ DMA2_IRQ_MASK, (1 << DMA2_IRQ       )  //DMA channel 2 xfer complete/error
.equ DMA3_IRQ_MASK, (1 << DMA3_IRQ       )  //DMA channel 3 xfer complete/error
.equ Reserved20_IRQ_MASK, (1 << Reserved20_IRQ )  //(reserved)
.equ FTFA_IRQ_MASK, (1 << FTFA_IRQ       )  //command complete// read collision
.equ PMC_IRQ_MASK, (1 << PMC_IRQ        )  //low-voltage detect//low-voltage warning
.equ LLWU_IRQ_MASK, (1 << LLWU_IRQ       )  //low leakage wakeup
.equ I2C0_IRQ_MASK, (1 << I2C0_IRQ       )  //I2C0
.equ I2C1_IRQ_MASK, (1 << I2C1_IRQ       )  //I2C1
.equ SPI0_IRQ_MASK, (1 << SPI0_IRQ       )  //SPI0 (all IRQ sources)
.equ SPI1_IRQ_MASK, (1 << SPI1_IRQ       )  //SPI1 (all IRQ sources)
.equ UART0_IRQ_MASK, (1 << UART0_IRQ      )  //UART0 (status// error)
.equ UART1_IRQ_MASK, (1 << UART1_IRQ      )  //UART1 (status// error)
.equ UART2_IRQ_MASK, (1 << UART2_IRQ      )  //UART2 (status// error)
.equ ADC0_IRQ_MASK, (1 << ADC0_IRQ       )  //ADC0
.equ CMP0_IRQ_MASK, (1 << CMP0_IRQ       )  //CMP0
.equ TMP0_IRQ_MASK, (1 << TMP0_IRQ       )  //TPM0
.equ TPM1_IRQ_MASK, (1 << TPM1_IRQ       )  //TPM1
.equ TPM2_IRQ_MASK, (1 << TPM2_IRQ       )  //TPM2
.equ RTC_IRQ_MASK, (1 << RTC_IRQ        )  //RTC (alarm)
.equ RTC_Seconds_IRQ_MASK, (1 << RTC_Seconds_IRQ)  //RTC (seconds)
.equ PIT_IRQ_MASK, (1 << PIT_IRQ        )  //PIT (all IRQ sources)
.equ I2S0_IRQ_MASK, (1 << I2S0_IRQ       )  //I2S0
.equ USB0_IRQ_MASK, (1 << USB0_IRQ       )  //USB OTG
.equ DAC0_IRQ_MASK, (1 << DAC0_IRQ       )  //DAC0
.equ TSI0_IRQ_MASK, (1 << TSI0_IRQ       )  //TSI0
.equ MCG_IRQ_MASK, (1 << MCG_IRQ        )  //MCG
.equ LPTMR0_IRQ_MASK, (1 << LPTMR0_IRQ     )  //LPTMR0
.equ LCD_IRQ_MASK, (1 << LCD_IRQ        )  //LCD
.equ PORTA_IRQ_MASK, (1 << PORTA_IRQ      )  //PORTA pin detect
.equ PORTC_PORTD_IRQ_MASK, (1 << PORTC_PORTD_IRQ)  //PORTC and PORTD pin detect
//---------------------------------------------------------------
//NVIC vectors
.equ Init_SP_Vector, 00  //end of stack
.equ Reset_Vector, 01  //reset vector
.equ NMI_Vector02, 02  //NMI
.equ Hard_Fault_Vector, 03  //hard fault
.equ Reserved04_Vector, 04  //(reserved)
.equ Reserved05_Vector, 05  //(reserved)
.equ Reserved06_Vector, 06  //(reserved)
.equ Reserved07_Vector, 07  //(reserved)
.equ Reserved08_Vector, 8  //(reserved)
.equ Reserved09_Vector, 9  //(reserved)
.equ Reserved10_Vector, 10  //(reserved)
.equ SVCall_Vector, 11  //SVCall (supervisor call)
.equ Reserved12_Vector, 12  //(reserved)
.equ Reserved13_Vector, 13  //(reserved)
.equ PendSR_Vector, 14  //PendableSrvReq (pendable request for system service)
.equ SysTick_Vector, 15  //SysTick (system tick timer)
.equ DMA0_Vector, 16  //DMA channel 0 xfer complete/error
.equ DMA1_Vector, 17  //DMA channel 1 xfer complete/error
.equ DMA2_Vector, 18  //DMA channel 2 xfer complete/error
.equ DMA3_Vector, 19  //DMA channel 3 xfer complete/error
.equ Reserved20_Vector, 20  //(reserved)
.equ FTFA_Vector, 21  //command complete// read collision
.equ PMC_Vector, 22  //low-voltage detect//low-voltage warning
.equ LLWU_Vector, 23  //low leakage wakeup
.equ I2C0_Vector, 24  //I2C0
.equ I2C1_Vector, 25  //I2C1
.equ SPI0_Vector, 26  //SPI0 (all IRQ sources)
.equ SPI1_Vector, 27  //SPI1 (all IRQ sources)
.equ UART0_Vector, 28  //UART0 (status// error)
.equ UART1_Vector, 29  //UART1 (status// error)
.equ UART2_Vector, 30  //UART2 (status// error)
.equ ADC0_Vector, 31  //ADC0
.equ CMP0_Vector, 32  //CMP0
.equ TMP0_Vector, 33  //TPM0
.equ TPM1_Vector, 34  //TPM1
.equ TPM2_Vector, 35  //TPM2
.equ RTC_Vector, 36  //RTC (alarm)
.equ RTC_Seconds_Vector, 37  //RTC (seconds)
.equ PIT_Vector, 38  //PIT (all IRQ sources)
.equ I2S0_Vector, 39  //I2S0
.equ USB0_Vector, 40  //USB OTG
.equ DAC0_Vector, 41  //DAC0
.equ TSI0_Vector, 42  //TSI0
.equ MCG_Vector, 43  //MCG
.equ LPTMR0_Vector, 44  //LPTMR0
.equ LCD_Vector, 45  //LCD
.equ PORTA_Vector, 46  //PORTA pin detect
.equ PORTD_Vector, 47  //PORTD pin detect
//---------------------------------------------------------------
//OSC
.equ OSC0_BASE, 0x40065000
.equ OSC0_CR_OFFSET, 0
.equ OSC0_CR, (OSC0_BASE + OSC0_CR_OFFSET)
//---------------------------------------------------------------
//OSC0_CR (0x00)
//7:ERCLKEN=external reference enable, OSCERCLK (0)
//6:(reserved):read-only:0
//5:EREFSTEN=external reference stop enable (0)
//4:(reserved):read-only:0
//3:SC2P=oscillator 2-pF capacitor load configure (0)
//2:SC4P=oscillator 4-pF capacitor load configure (0)
//1:SC8P=oscillator 8-pF capacitor load configure (0)
//0:SC16P=oscillator 16-pF capacitor load configure (0)
.equ OSC_CR_SC16P_MASK, 0x1
.equ OSC_CR_SC16P_SHIFT, 0
.equ OSC_CR_SC8P_MASK, 0x2
.equ OSC_CR_SC8P_SHIFT, 1
.equ OSC_CR_SC4P_MASK, 0x4
.equ OSC_CR_SC4P_SHIFT, 2
.equ OSC_CR_SC2P_MASK, 0x8
.equ OSC_CR_SC2P_SHIFT, 3
.equ OSC_CR_EREFSTEN_MASK, 0x20
.equ OSC_CR_EREFSTEN_SHIFT, 5
.equ OSC_CR_ERCLKEN_MASK, 0x80
.equ OSC_CR_ERCLKEN_SHIFT, 7
//---------------------------------------------------------------
//PIT
.equ PIT_BASE, 0x40037000
.equ PIT_CH0_BASE, 0x40037100
.equ PIT_CH1_BASE, 0x40037110
.equ PIT_LDVAL_OFFSET, 0x00
.equ PIT_CVAL_OFFSET, 0x04
.equ PIT_TCTRL_OFFSET, 0x08
.equ PIT_TFLG_OFFSET, 0x0C
.equ PIT_MCR_OFFSET, 0x00
.equ PIT_LTMR64H_OFFSET, 0xE0
.equ PIT_LTMR64L_OFFSET, 0xE4
.equ PIT_LDVAL0_OFFSET, 0x100
.equ PIT_CVAL0_OFFSET, 0x104
.equ PIT_TCTRL0_OFFSET, 0x108
.equ PIT_TFLG0_OFFSET, 0x10C
.equ PIT_LDVAL1_OFFSET, 0x110
.equ PIT_CVAL1_OFFSET, 0x114
.equ PIT_TCTRL1_OFFSET, 0x118
.equ PIT_TFLG1_OFFSET, 0x11C
.equ PIT_MCR, (PIT_BASE + PIT_MCR_OFFSET)
.equ PIT_LTMR64H, (PIT_BASE + PIT_LTMR64H_OFFSET)
.equ PIT_LTMR64L, (PIT_BASE + PIT_LTMR64L_OFFSET)
.equ PIT_LDVAL0, (PIT_BASE + PIT_LDVAL0_OFFSET)
.equ PIT_CVAL0, (PIT_BASE + PIT_CVAL0_OFFSET)
.equ PIT_TCTRL0, (PIT_BASE + PIT_TCTRL0_OFFSET)
.equ PIT_TFLG0, (PIT_BASE + PIT_TFLG0_OFFSET)
.equ PIT_LDVAL1, (PIT_BASE + PIT_LDVAL1_OFFSET)
.equ PIT_CVAL1, (PIT_BASE + PIT_CVAL1_OFFSET)
.equ PIT_TCTRL1, (PIT_BASE + PIT_TCTRL1_OFFSET)
.equ PIT_TFLG1, (PIT_BASE + PIT_TFLG1_OFFSET)
//---------------------------------------------------------------
//PIT_CVALn:  current timer value register (channel n)
//31-0:TVL=current timer value
//---------------------------------------------------------------
//PIT_LDVALn:  timer load value register (channel n)
//31-0:TSV=timer start value
//         PIT chan. n counts down from this value to 0,
//         then sets TIF and loads LDVALn
//---------------------------------------------------------------
//PIT_LTMR64H:  PIT upper lifetime timer register
//for applications chaining timer 0 and timer 1 for 64-bit timer
//31-0:LTH=life timer value (high word)
//         value of timer 1 (CVAL1)// read before PIT_LTMR64L
//---------------------------------------------------------------
//PIT_LTMR64L:  PIT lower lifetime timer register
//for applications chaining timer 0 and timer 1 for 64-bit timer
//31-0:LTL=life timer value (low word)
//         value of timer 0 (CVAL0)// read after PIT_LTMR64H
//---------------------------------------------------------------
//PIT_MCR:  PIT module control register
//31-3:(reserved):read-only:0
//   2:(reserved)
//   1:MDIS=module disable (PIT section)
//          RTI timer not affected by this field
//          must be enabled before any other setup
//   0:FRZ=freeze:  continue'/stop timers in debug mode
.equ PIT_MCR_MDIS_MASK, 0x00000002
.equ PIT_MCR_MDIS_SHIFT, 1
.equ PIT_MCR_FRZ_MASK, 0x00000001
.equ PIT_MCR_FRZ_SHIFT, 0
//---------------------------------------------------------------
//PIT_TCTRLn:  timer control register (channel n)
//31-3:(reserved):read-only:0
//   2:CHN=chain mode (enable)
//          in chain mode, channel n-1 must expire before
//                         channel n counts
//          timer 0 cannot be changed
//   1:TIE=timer interrupt enable (on TIF)
//   0:TEN=timer enable
.equ PIT_TCTRL_CHN_MASK, 0x00000004
.equ PIT_TCTRL_CHN_SHIFT, 2
.equ PIT_TCTRL_TIE_MASK, 0x00000002
.equ PIT_TCTRL_TIE_SHIFT, 1
.equ PIT_TCTRL_TEN_MASK, 0x00000001
.equ PIT_TCTRL_TEN_SHIFT, 0
//---------------------------------------------------------------
//PIT_TFLGn:  timer flag register (channel n)
//31-1:(reserved):read-only:0
//   0:TIF=timer interrupt flag
//         write 1 to clear
.equ PIT_TFLG_TIF_MASK, 0x00000001
.equ PIT_TFLG_TIF_SHIFT, 0
//---------------------------------------------------------------
//PORTx_PCRn (Port x pin control register n [for pin n])
//31-25:(reserved):read-only:0
//   24:ISF=interrupt status flag// write 1 clears
//23-20:(reserved):read-only:0
//19-16:IRCQ=interrupt configuration
//          :0000=interrupt/DMA request disabled
//          :0001=DMA request on rising edge
//          :0010=DMA request on falling edge
//          :0011=DMA request on either edge
//          :1000=interrupt when logic zero
//          :1001=interrupt on rising edge
//          :1010=interrupt on falling edge
//          :1011=interrupt on either edge
//          :1100=interrupt when logic one
//          :others=reserved
//15-11:(reserved):read-only:0
//10-08:MUX=Pin mux control
//         :000=pin disabled (analog)
//         :001=alternative 1 (GPIO)
//         :010-111=alternatives 2-7 (chip-specific)
//    7:(reserved):read-only:0
//    6:DSE=Drive strength enable
//         :0=low
//         :1=high
//    5:(reserved):read-only:0
//    4:PFE=Passive filter enable
//    3:(reserved):read-only:0
//    2:SRE=Slew rate enable
//         :0=fast
//         :1=slow
//    1:PE=Pull enable
//    0:PS=Pull select (if PE=1)
//        :0=internal pulldown
//        :1=internal pullup
.equ PORT_PCR_ISF_MASK, 0x01000000
.equ PORT_PCR_ISF_SHIFT, 24
.equ PORT_PCR_IRCQ_MASK, 0x000F0000
.equ PORT_PCR_IRCQ_SHIFT, 16
.equ PORT_PCR_MUX_MASK, 0x00000700
.equ PORT_PCR_MUX_SHIFT, 8
.equ PORT_PCR_DSE_MASK, 0x40
.equ PORT_PCR_DSE_SHIFT, 6
.equ PORT_PCR_PFE_MASK, 0x10
.equ PORT_PCR_PFE_SHIFT, 4
.equ PORT_PCR_SRE_MASK, 0x04
.equ PORT_PCR_SRE_SHIFT, 2
.equ PORT_PCR_PE_MASK, 0x02
.equ PORT_PCR_PE_SHIFT, 1
.equ PORT_PCR_PS_MASK, 0x01
.equ PORT_PCR_PS_SHIFT, 0
.equ PORT_PCR_MUX_SELECT_0_MASK, 0x00000000 //analog
.equ PORT_PCR_MUX_SELECT_1_MASK, 0x00000100 //GPIO
.equ PORT_PCR_MUX_SELECT_2_MASK, 0x00000200
.equ PORT_PCR_MUX_SELECT_3_MASK, 0x00000300
.equ PORT_PCR_MUX_SELECT_4_MASK, 0x00000400
.equ PORT_PCR_MUX_SELECT_5_MASK, 0x00000500
.equ PORT_PCR_MUX_SELECT_6_MASK, 0x00000600
.equ PORT_PCR_MUX_SELECT_7_MASK, 0x00000700
//---------------------------------------------------------------
//Port A
.equ PORTA_BASE, 0x40049000
.equ PORTA_PCR0_OFFSET, 0x00
.equ PORTA_PCR1_OFFSET, 0x04
.equ PORTA_PCR2_OFFSET, 0x08
.equ PORTA_PCR3_OFFSET, 0x0C
.equ PORTA_PCR4_OFFSET, 0x10
.equ PORTA_PCR5_OFFSET, 0x14
.equ PORTA_PCR6_OFFSET, 0x18
.equ PORTA_PCR7_OFFSET, 0x1C
.equ PORTA_PCR8_OFFSET, 0x20
.equ PORTA_PCR9_OFFSET, 0x24
.equ PORTA_PCR10_OFFSET, 0x28
.equ PORTA_PCR11_OFFSET, 0x2C
.equ PORTA_PCR12_OFFSET, 0x30
.equ PORTA_PCR13_OFFSET, 0x34
.equ PORTA_PCR14_OFFSET, 0x38
.equ PORTA_PCR15_OFFSET, 0x3C
.equ PORTA_PCR16_OFFSET, 0x40
.equ PORTA_PCR17_OFFSET, 0x44
.equ PORTA_PCR18_OFFSET, 0x48
.equ PORTA_PCR19_OFFSET, 0x4C
.equ PORTA_PCR20_OFFSET, 0x50
.equ PORTA_PCR21_OFFSET, 0x54
.equ PORTA_PCR22_OFFSET, 0x58
.equ PORTA_PCR23_OFFSET, 0x5C
.equ PORTA_PCR24_OFFSET, 0x60
.equ PORTA_PCR25_OFFSET, 0x64
.equ PORTA_PCR26_OFFSET, 0x68
.equ PORTA_PCR27_OFFSET, 0x6C
.equ PORTA_PCR28_OFFSET, 0x70
.equ PORTA_PCR29_OFFSET, 0x74
.equ PORTA_PCR30_OFFSET, 0x78
.equ PORTA_PCR31_OFFSET, 0x7C
.equ PORTA_GPCLR_OFFSET, 0x80
.equ PORTA_GPCHR_OFFSET, 0x84
.equ PORTA_ISFR_OFFSET, 0xA0
.equ PORTA_PCR0, (PORTA_BASE + PORTA_PCR0_OFFSET)
.equ PORTA_PCR1, (PORTA_BASE + PORTA_PCR1_OFFSET)
.equ PORTA_PCR2, (PORTA_BASE + PORTA_PCR2_OFFSET)
.equ PORTA_PCR3, (PORTA_BASE + PORTA_PCR3_OFFSET)
.equ PORTA_PCR4, (PORTA_BASE + PORTA_PCR4_OFFSET)
.equ PORTA_PCR5, (PORTA_BASE + PORTA_PCR5_OFFSET)
.equ PORTA_PCR6, (PORTA_BASE + PORTA_PCR6_OFFSET)
.equ PORTA_PCR7, (PORTA_BASE + PORTA_PCR7_OFFSET)
.equ PORTA_PCR8, (PORTA_BASE + PORTA_PCR8_OFFSET)
.equ PORTA_PCR9, (PORTA_BASE + PORTA_PCR9_OFFSET)
.equ PORTA_PCR10, (PORTA_BASE + PORTA_PCR10_OFFSET)
.equ PORTA_PCR11, (PORTA_BASE + PORTA_PCR11_OFFSET)
.equ PORTA_PCR12, (PORTA_BASE + PORTA_PCR12_OFFSET)
.equ PORTA_PCR13, (PORTA_BASE + PORTA_PCR13_OFFSET)
.equ PORTA_PCR14, (PORTA_BASE + PORTA_PCR14_OFFSET)
.equ PORTA_PCR15, (PORTA_BASE + PORTA_PCR15_OFFSET)
.equ PORTA_PCR16, (PORTA_BASE + PORTA_PCR16_OFFSET)
.equ PORTA_PCR17, (PORTA_BASE + PORTA_PCR17_OFFSET)
.equ PORTA_PCR18, (PORTA_BASE + PORTA_PCR18_OFFSET)
.equ PORTA_PCR19, (PORTA_BASE + PORTA_PCR19_OFFSET)
.equ PORTA_PCR20, (PORTA_BASE + PORTA_PCR20_OFFSET)
.equ PORTA_PCR21, (PORTA_BASE + PORTA_PCR21_OFFSET)
.equ PORTA_PCR22, (PORTA_BASE + PORTA_PCR22_OFFSET)
.equ PORTA_PCR23, (PORTA_BASE + PORTA_PCR23_OFFSET)
.equ PORTA_PCR24, (PORTA_BASE + PORTA_PCR24_OFFSET)
.equ PORTA_PCR25, (PORTA_BASE + PORTA_PCR25_OFFSET)
.equ PORTA_PCR26, (PORTA_BASE + PORTA_PCR26_OFFSET)
.equ PORTA_PCR27, (PORTA_BASE + PORTA_PCR27_OFFSET)
.equ PORTA_PCR28, (PORTA_BASE + PORTA_PCR28_OFFSET)
.equ PORTA_PCR29, (PORTA_BASE + PORTA_PCR29_OFFSET)
.equ PORTA_PCR30, (PORTA_BASE + PORTA_PCR30_OFFSET)
.equ PORTA_PCR31, (PORTA_BASE + PORTA_PCR31_OFFSET)
.equ PORTA_GPCLR, (PORTA_BASE + PORTA_GPCLR_OFFSET)
.equ PORTA_GPCHR, (PORTA_BASE + PORTA_GPCHR_OFFSET)
.equ PORTA_ISFR, (PORTA_BASE + PORTA_ISFR_OFFSET)
//---------------------------------------------------------------
//Port B
.equ PORTB_BASE, 0x4004A000
.equ PORTB_PCR0_OFFSET, 0x00
.equ PORTB_PCR1_OFFSET, 0x04
.equ PORTB_PCR2_OFFSET, 0x08
.equ PORTB_PCR3_OFFSET, 0x0C
.equ PORTB_PCR4_OFFSET, 0x10
.equ PORTB_PCR5_OFFSET, 0x14
.equ PORTB_PCR6_OFFSET, 0x18
.equ PORTB_PCR7_OFFSET, 0x1C
.equ PORTB_PCR8_OFFSET, 0x20
.equ PORTB_PCR9_OFFSET, 0x24
.equ PORTB_PCR10_OFFSET, 0x28
.equ PORTB_PCR11_OFFSET, 0x2C
.equ PORTB_PCR12_OFFSET, 0x30
.equ PORTB_PCR13_OFFSET, 0x34
.equ PORTB_PCR14_OFFSET, 0x38
.equ PORTB_PCR15_OFFSET, 0x3C
.equ PORTB_PCR16_OFFSET, 0x40
.equ PORTB_PCR17_OFFSET, 0x44
.equ PORTB_PCR18_OFFSET, 0x48
.equ PORTB_PCR19_OFFSET, 0x4C
.equ PORTB_PCR20_OFFSET, 0x50
.equ PORTB_PCR21_OFFSET, 0x54
.equ PORTB_PCR22_OFFSET, 0x58
.equ PORTB_PCR23_OFFSET, 0x5C
.equ PORTB_PCR24_OFFSET, 0x60
.equ PORTB_PCR25_OFFSET, 0x64
.equ PORTB_PCR26_OFFSET, 0x68
.equ PORTB_PCR27_OFFSET, 0x6C
.equ PORTB_PCR28_OFFSET, 0x70
.equ PORTB_PCR29_OFFSET, 0x74
.equ PORTB_PCR30_OFFSET, 0x78
.equ PORTB_PCR31_OFFSET, 0x7C
.equ PORTB_GPCLR_OFFSET, 0x80
.equ PORTB_GPCHR_OFFSET, 0x84
.equ PORTB_ISFR_OFFSET, 0xA0
.equ PORTB_PCR0, (PORTB_BASE + PORTB_PCR0_OFFSET)
.equ PORTB_PCR1, (PORTB_BASE + PORTB_PCR1_OFFSET)
.equ PORTB_PCR2, (PORTB_BASE + PORTB_PCR2_OFFSET)
.equ PORTB_PCR3, (PORTB_BASE + PORTB_PCR3_OFFSET)
.equ PORTB_PCR4, (PORTB_BASE + PORTB_PCR4_OFFSET)
.equ PORTB_PCR5, (PORTB_BASE + PORTB_PCR5_OFFSET)
.equ PORTB_PCR6, (PORTB_BASE + PORTB_PCR6_OFFSET)
.equ PORTB_PCR7, (PORTB_BASE + PORTB_PCR7_OFFSET)
.equ PORTB_PCR8, (PORTB_BASE + PORTB_PCR8_OFFSET)
.equ PORTB_PCR9, (PORTB_BASE + PORTB_PCR9_OFFSET)
.equ PORTB_PCR10, (PORTB_BASE + PORTB_PCR10_OFFSET)
.equ PORTB_PCR11, (PORTB_BASE + PORTB_PCR11_OFFSET)
.equ PORTB_PCR12, (PORTB_BASE + PORTB_PCR12_OFFSET)
.equ PORTB_PCR13, (PORTB_BASE + PORTB_PCR13_OFFSET)
.equ PORTB_PCR14, (PORTB_BASE + PORTB_PCR14_OFFSET)
.equ PORTB_PCR15, (PORTB_BASE + PORTB_PCR15_OFFSET)
.equ PORTB_PCR16, (PORTB_BASE + PORTB_PCR16_OFFSET)
.equ PORTB_PCR17, (PORTB_BASE + PORTB_PCR17_OFFSET)
.equ PORTB_PCR18, (PORTB_BASE + PORTB_PCR18_OFFSET)
.equ PORTB_PCR19, (PORTB_BASE + PORTB_PCR19_OFFSET)
.equ PORTB_PCR20, (PORTB_BASE + PORTB_PCR20_OFFSET)
.equ PORTB_PCR21, (PORTB_BASE + PORTB_PCR21_OFFSET)
.equ PORTB_PCR22, (PORTB_BASE + PORTB_PCR22_OFFSET)
.equ PORTB_PCR23, (PORTB_BASE + PORTB_PCR23_OFFSET)
.equ PORTB_PCR24, (PORTB_BASE + PORTB_PCR24_OFFSET)
.equ PORTB_PCR25, (PORTB_BASE + PORTB_PCR25_OFFSET)
.equ PORTB_PCR26, (PORTB_BASE + PORTB_PCR26_OFFSET)
.equ PORTB_PCR27, (PORTB_BASE + PORTB_PCR27_OFFSET)
.equ PORTB_PCR28, (PORTB_BASE + PORTB_PCR28_OFFSET)
.equ PORTB_PCR29, (PORTB_BASE + PORTB_PCR29_OFFSET)
.equ PORTB_PCR30, (PORTB_BASE + PORTB_PCR30_OFFSET)
.equ PORTB_PCR31, (PORTB_BASE + PORTB_PCR31_OFFSET)
.equ PORTB_GPCLR, (PORTB_BASE + PORTB_GPCLR_OFFSET)
.equ PORTB_GPCHR, (PORTB_BASE + PORTB_GPCHR_OFFSET)
.equ PORTB_ISFR, (PORTB_BASE + PORTB_ISFR_OFFSET)
//---------------------------------------------------------------
//Port C
.equ PORTC_BASE, 0x4004B000
.equ PORTC_PCR0_OFFSET, 0x00
.equ PORTC_PCR1_OFFSET, 0x04
.equ PORTC_PCR2_OFFSET, 0x08
.equ PORTC_PCR3_OFFSET, 0x0C
.equ PORTC_PCR4_OFFSET, 0x10
.equ PORTC_PCR5_OFFSET, 0x14
.equ PORTC_PCR6_OFFSET, 0x18
.equ PORTC_PCR7_OFFSET, 0x1C
.equ PORTC_PCR8_OFFSET, 0x20
.equ PORTC_PCR9_OFFSET, 0x24
.equ PORTC_PCR10_OFFSET, 0x28
.equ PORTC_PCR11_OFFSET, 0x2C
.equ PORTC_PCR12_OFFSET, 0x30
.equ PORTC_PCR13_OFFSET, 0x34
.equ PORTC_PCR14_OFFSET, 0x38
.equ PORTC_PCR15_OFFSET, 0x3C
.equ PORTC_PCR16_OFFSET, 0x40
.equ PORTC_PCR17_OFFSET, 0x44
.equ PORTC_PCR18_OFFSET, 0x48
.equ PORTC_PCR19_OFFSET, 0x4C
.equ PORTC_PCR20_OFFSET, 0x50
.equ PORTC_PCR21_OFFSET, 0x54
.equ PORTC_PCR22_OFFSET, 0x58
.equ PORTC_PCR23_OFFSET, 0x5C
.equ PORTC_PCR24_OFFSET, 0x60
.equ PORTC_PCR25_OFFSET, 0x64
.equ PORTC_PCR26_OFFSET, 0x68
.equ PORTC_PCR27_OFFSET, 0x6C
.equ PORTC_PCR28_OFFSET, 0x70
.equ PORTC_PCR29_OFFSET, 0x74
.equ PORTC_PCR30_OFFSET, 0x78
.equ PORTC_PCR31_OFFSET, 0x7C
.equ PORTC_GPCLR_OFFSET, 0x80
.equ PORTC_GPCHR_OFFSET, 0x84
.equ PORTC_ISFR_OFFSET, 0xA0
.equ PORTC_PCR0, (PORTC_BASE + PORTC_PCR0_OFFSET)
.equ PORTC_PCR1, (PORTC_BASE + PORTC_PCR1_OFFSET)
.equ PORTC_PCR2, (PORTC_BASE + PORTC_PCR2_OFFSET)
.equ PORTC_PCR3, (PORTC_BASE + PORTC_PCR3_OFFSET)
.equ PORTC_PCR4, (PORTC_BASE + PORTC_PCR4_OFFSET)
.equ PORTC_PCR5, (PORTC_BASE + PORTC_PCR5_OFFSET)
.equ PORTC_PCR6, (PORTC_BASE + PORTC_PCR6_OFFSET)
.equ PORTC_PCR7, (PORTC_BASE + PORTC_PCR7_OFFSET)
.equ PORTC_PCR8, (PORTC_BASE + PORTC_PCR8_OFFSET)
.equ PORTC_PCR9, (PORTC_BASE + PORTC_PCR9_OFFSET)
.equ PORTC_PCR10, (PORTC_BASE + PORTC_PCR10_OFFSET)
.equ PORTC_PCR11, (PORTC_BASE + PORTC_PCR11_OFFSET)
.equ PORTC_PCR12, (PORTC_BASE + PORTC_PCR12_OFFSET)
.equ PORTC_PCR13, (PORTC_BASE + PORTC_PCR13_OFFSET)
.equ PORTC_PCR14, (PORTC_BASE + PORTC_PCR14_OFFSET)
.equ PORTC_PCR15, (PORTC_BASE + PORTC_PCR15_OFFSET)
.equ PORTC_PCR16, (PORTC_BASE + PORTC_PCR16_OFFSET)
.equ PORTC_PCR17, (PORTC_BASE + PORTC_PCR17_OFFSET)
.equ PORTC_PCR18, (PORTC_BASE + PORTC_PCR18_OFFSET)
.equ PORTC_PCR19, (PORTC_BASE + PORTC_PCR19_OFFSET)
.equ PORTC_PCR20, (PORTC_BASE + PORTC_PCR20_OFFSET)
.equ PORTC_PCR21, (PORTC_BASE + PORTC_PCR21_OFFSET)
.equ PORTC_PCR22, (PORTC_BASE + PORTC_PCR22_OFFSET)
.equ PORTC_PCR23, (PORTC_BASE + PORTC_PCR23_OFFSET)
.equ PORTC_PCR24, (PORTC_BASE + PORTC_PCR24_OFFSET)
.equ PORTC_PCR25, (PORTC_BASE + PORTC_PCR25_OFFSET)
.equ PORTC_PCR26, (PORTC_BASE + PORTC_PCR26_OFFSET)
.equ PORTC_PCR27, (PORTC_BASE + PORTC_PCR27_OFFSET)
.equ PORTC_PCR28, (PORTC_BASE + PORTC_PCR28_OFFSET)
.equ PORTC_PCR29, (PORTC_BASE + PORTC_PCR29_OFFSET)
.equ PORTC_PCR30, (PORTC_BASE + PORTC_PCR30_OFFSET)
.equ PORTC_PCR31, (PORTC_BASE + PORTC_PCR31_OFFSET)
.equ PORTC_GPCLR, (PORTC_BASE + PORTC_GPCLR_OFFSET)
.equ PORTC_GPCHR, (PORTC_BASE + PORTC_GPCHR_OFFSET)
.equ PORTC_ISFR, (PORTC_BASE + PORTC_ISFR_OFFSET)
//---------------------------------------------------------------
//Port D
.equ PORTD_BASE, 0x4004C000
.equ PORTD_PCR0_OFFSET, 0x00
.equ PORTD_PCR1_OFFSET, 0x04
.equ PORTD_PCR2_OFFSET, 0x08
.equ PORTD_PCR3_OFFSET, 0x0C
.equ PORTD_PCR4_OFFSET, 0x10
.equ PORTD_PCR5_OFFSET, 0x14
.equ PORTD_PCR6_OFFSET, 0x18
.equ PORTD_PCR7_OFFSET, 0x1C
.equ PORTD_PCR8_OFFSET, 0x20
.equ PORTD_PCR9_OFFSET, 0x24
.equ PORTD_PCR10_OFFSET, 0x28
.equ PORTD_PCR11_OFFSET, 0x2C
.equ PORTD_PCR12_OFFSET, 0x30
.equ PORTD_PCR13_OFFSET, 0x34
.equ PORTD_PCR14_OFFSET, 0x38
.equ PORTD_PCR15_OFFSET, 0x3C
.equ PORTD_PCR16_OFFSET, 0x40
.equ PORTD_PCR17_OFFSET, 0x44
.equ PORTD_PCR18_OFFSET, 0x48
.equ PORTD_PCR19_OFFSET, 0x4C
.equ PORTD_PCR20_OFFSET, 0x50
.equ PORTD_PCR21_OFFSET, 0x54
.equ PORTD_PCR22_OFFSET, 0x58
.equ PORTD_PCR23_OFFSET, 0x5C
.equ PORTD_PCR24_OFFSET, 0x60
.equ PORTD_PCR25_OFFSET, 0x64
.equ PORTD_PCR26_OFFSET, 0x68
.equ PORTD_PCR27_OFFSET, 0x6C
.equ PORTD_PCR28_OFFSET, 0x70
.equ PORTD_PCR29_OFFSET, 0x74
.equ PORTD_PCR30_OFFSET, 0x78
.equ PORTD_PCR31_OFFSET, 0x7C
.equ PORTD_GPCLR_OFFSET, 0x80
.equ PORTD_GPCHR_OFFSET, 0x84
.equ PORTD_ISFR_OFFSET, 0xA0
.equ PORTD_PCR0, (PORTD_BASE + PORTD_PCR0_OFFSET)
.equ PORTD_PCR1, (PORTD_BASE + PORTD_PCR1_OFFSET)
.equ PORTD_PCR2, (PORTD_BASE + PORTD_PCR2_OFFSET)
.equ PORTD_PCR3, (PORTD_BASE + PORTD_PCR3_OFFSET)
.equ PORTD_PCR4, (PORTD_BASE + PORTD_PCR4_OFFSET)
.equ PORTD_PCR5, (PORTD_BASE + PORTD_PCR5_OFFSET)
.equ PORTD_PCR6, (PORTD_BASE + PORTD_PCR6_OFFSET)
.equ PORTD_PCR7, (PORTD_BASE + PORTD_PCR7_OFFSET)
.equ PORTD_PCR8, (PORTD_BASE + PORTD_PCR8_OFFSET)
.equ PORTD_PCR9, (PORTD_BASE + PORTD_PCR9_OFFSET)
.equ PORTD_PCR10, (PORTD_BASE + PORTD_PCR10_OFFSET)
.equ PORTD_PCR11, (PORTD_BASE + PORTD_PCR11_OFFSET)
.equ PORTD_PCR12, (PORTD_BASE + PORTD_PCR12_OFFSET)
.equ PORTD_PCR13, (PORTD_BASE + PORTD_PCR13_OFFSET)
.equ PORTD_PCR14, (PORTD_BASE + PORTD_PCR14_OFFSET)
.equ PORTD_PCR15, (PORTD_BASE + PORTD_PCR15_OFFSET)
.equ PORTD_PCR16, (PORTD_BASE + PORTD_PCR16_OFFSET)
.equ PORTD_PCR17, (PORTD_BASE + PORTD_PCR17_OFFSET)
.equ PORTD_PCR18, (PORTD_BASE + PORTD_PCR18_OFFSET)
.equ PORTD_PCR19, (PORTD_BASE + PORTD_PCR19_OFFSET)
.equ PORTD_PCR20, (PORTD_BASE + PORTD_PCR20_OFFSET)
.equ PORTD_PCR21, (PORTD_BASE + PORTD_PCR21_OFFSET)
.equ PORTD_PCR22, (PORTD_BASE + PORTD_PCR22_OFFSET)
.equ PORTD_PCR23, (PORTD_BASE + PORTD_PCR23_OFFSET)
.equ PORTD_PCR24, (PORTD_BASE + PORTD_PCR24_OFFSET)
.equ PORTD_PCR25, (PORTD_BASE + PORTD_PCR25_OFFSET)
.equ PORTD_PCR26, (PORTD_BASE + PORTD_PCR26_OFFSET)
.equ PORTD_PCR27, (PORTD_BASE + PORTD_PCR27_OFFSET)
.equ PORTD_PCR28, (PORTD_BASE + PORTD_PCR28_OFFSET)
.equ PORTD_PCR29, (PORTD_BASE + PORTD_PCR29_OFFSET)
.equ PORTD_PCR30, (PORTD_BASE + PORTD_PCR30_OFFSET)
.equ PORTD_PCR31, (PORTD_BASE + PORTD_PCR31_OFFSET)
.equ PORTD_GPCLR, (PORTD_BASE + PORTD_GPCLR_OFFSET)
.equ PORTD_GPCHR, (PORTD_BASE + PORTD_GPCHR_OFFSET)
.equ PORTD_ISFR, (PORTD_BASE + PORTD_ISFR_OFFSET)
//---------------------------------------------------------------
//Port E
.equ PORTE_BASE, 0x4004D000
.equ PORTE_PCR0_OFFSET, 0x00
.equ PORTE_PCR1_OFFSET, 0x04
.equ PORTE_PCR2_OFFSET, 0x08
.equ PORTE_PCR3_OFFSET, 0x0C
.equ PORTE_PCR4_OFFSET, 0x10
.equ PORTE_PCR5_OFFSET, 0x14
.equ PORTE_PCR6_OFFSET, 0x18
.equ PORTE_PCR7_OFFSET, 0x1C
.equ PORTE_PCR8_OFFSET, 0x20
.equ PORTE_PCR9_OFFSET, 0x24
.equ PORTE_PCR10_OFFSET, 0x28
.equ PORTE_PCR11_OFFSET, 0x2C
.equ PORTE_PCR12_OFFSET, 0x30
.equ PORTE_PCR13_OFFSET, 0x34
.equ PORTE_PCR14_OFFSET, 0x38
.equ PORTE_PCR15_OFFSET, 0x3C
.equ PORTE_PCR16_OFFSET, 0x40
.equ PORTE_PCR17_OFFSET, 0x44
.equ PORTE_PCR18_OFFSET, 0x48
.equ PORTE_PCR19_OFFSET, 0x4C
.equ PORTE_PCR20_OFFSET, 0x50
.equ PORTE_PCR21_OFFSET, 0x54
.equ PORTE_PCR22_OFFSET, 0x58
.equ PORTE_PCR23_OFFSET, 0x5C
.equ PORTE_PCR24_OFFSET, 0x60
.equ PORTE_PCR25_OFFSET, 0x64
.equ PORTE_PCR26_OFFSET, 0x68
.equ PORTE_PCR27_OFFSET, 0x6C
.equ PORTE_PCR28_OFFSET, 0x70
.equ PORTE_PCR29_OFFSET, 0x74
.equ PORTE_PCR30_OFFSET, 0x78
.equ PORTE_PCR31_OFFSET, 0x7C
.equ PORTE_GPCLR_OFFSET, 0x80
.equ PORTE_GPCHR_OFFSET, 0x84
.equ PORTE_ISFR_OFFSET, 0xA0
.equ PORTE_PCR0, (PORTE_BASE + PORTE_PCR0_OFFSET)
.equ PORTE_PCR1, (PORTE_BASE + PORTE_PCR1_OFFSET)
.equ PORTE_PCR2, (PORTE_BASE + PORTE_PCR2_OFFSET)
.equ PORTE_PCR3, (PORTE_BASE + PORTE_PCR3_OFFSET)
.equ PORTE_PCR4, (PORTE_BASE + PORTE_PCR4_OFFSET)
.equ PORTE_PCR5, (PORTE_BASE + PORTE_PCR5_OFFSET)
.equ PORTE_PCR6, (PORTE_BASE + PORTE_PCR6_OFFSET)
.equ PORTE_PCR7, (PORTE_BASE + PORTE_PCR7_OFFSET)
.equ PORTE_PCR8, (PORTE_BASE + PORTE_PCR8_OFFSET)
.equ PORTE_PCR9, (PORTE_BASE + PORTE_PCR9_OFFSET)
.equ PORTE_PCR10, (PORTE_BASE + PORTE_PCR10_OFFSET)
.equ PORTE_PCR11, (PORTE_BASE + PORTE_PCR11_OFFSET)
.equ PORTE_PCR12, (PORTE_BASE + PORTE_PCR12_OFFSET)
.equ PORTE_PCR13, (PORTE_BASE + PORTE_PCR13_OFFSET)
.equ PORTE_PCR14, (PORTE_BASE + PORTE_PCR14_OFFSET)
.equ PORTE_PCR15, (PORTE_BASE + PORTE_PCR15_OFFSET)
.equ PORTE_PCR16, (PORTE_BASE + PORTE_PCR16_OFFSET)
.equ PORTE_PCR17, (PORTE_BASE + PORTE_PCR17_OFFSET)
.equ PORTE_PCR18, (PORTE_BASE + PORTE_PCR18_OFFSET)
.equ PORTE_PCR19, (PORTE_BASE + PORTE_PCR19_OFFSET)
.equ PORTE_PCR20, (PORTE_BASE + PORTE_PCR20_OFFSET)
.equ PORTE_PCR21, (PORTE_BASE + PORTE_PCR21_OFFSET)
.equ PORTE_PCR22, (PORTE_BASE + PORTE_PCR22_OFFSET)
.equ PORTE_PCR23, (PORTE_BASE + PORTE_PCR23_OFFSET)
.equ PORTE_PCR24, (PORTE_BASE + PORTE_PCR24_OFFSET)
.equ PORTE_PCR25, (PORTE_BASE + PORTE_PCR25_OFFSET)
.equ PORTE_PCR26, (PORTE_BASE + PORTE_PCR26_OFFSET)
.equ PORTE_PCR27, (PORTE_BASE + PORTE_PCR27_OFFSET)
.equ PORTE_PCR28, (PORTE_BASE + PORTE_PCR28_OFFSET)
.equ PORTE_PCR29, (PORTE_BASE + PORTE_PCR29_OFFSET)
.equ PORTE_PCR30, (PORTE_BASE + PORTE_PCR30_OFFSET)
.equ PORTE_PCR31, (PORTE_BASE + PORTE_PCR31_OFFSET)
.equ PORTE_GPCLR, (PORTE_BASE + PORTE_GPCLR_OFFSET)
.equ PORTE_GPCHR, (PORTE_BASE + PORTE_GPCHR_OFFSET)
.equ PORTE_ISFR, (PORTE_BASE + PORTE_ISFR_OFFSET)
//---------------------------------------------------------------
//System integration module (SIM)
.equ SIM_BASE, 0x40047000
.equ SIM_SOPT1_OFFSET, 0x00
.equ SIM_SOPT1CFG_OFFSET, 0x04
.equ SIM_SOPT2_OFFSET, 0x1004
.equ SIM_SOPT4_OFFSET, 0x100C
.equ SIM_SOPT5_OFFSET, 0x1010
.equ SIM_SOPT7_OFFSET, 0x1018
.equ SIM_SDID_OFFSET, 0x1024
.equ SIM_SCGC4_OFFSET, 0x1034
.equ SIM_SCGC5_OFFSET, 0x1038
.equ SIM_SCGC6_OFFSET, 0x103C
.equ SIM_SCGC7_OFFSET, 0x1040
.equ SIM_CLKDIV1_OFFSET, 0x1044
.equ SIM_FCFG1_OFFSET, 0x104C
.equ SIM_FCFG2_OFFSET, 0x1050
.equ SIM_UIDMH_OFFSET, 0x1058
.equ SIM_UIDML_OFFSET, 0x105C
.equ SIM_UIDL_OFFSET, 0x1060
.equ SIM_COPC_OFFSET, 0x1100
.equ SIM_SRVCOP_OFFSET, 0x1104
.equ SIM_CLKDIV1, (SIM_BASE + SIM_CLKDIV1_OFFSET)
.equ SIM_COPC, (SIM_BASE + SIM_COPC_OFFSET)
.equ SIM_FCFG1, (SIM_BASE + SIM_FCFG1_OFFSET)
.equ SIM_FCFG2, (SIM_BASE + SIM_FCFG2_OFFSET)
.equ SIM_SCGC4, (SIM_BASE + SIM_SCGC4_OFFSET) 
.equ SIM_SCGC5, (SIM_BASE + SIM_SCGC5_OFFSET)
.equ SIM_SCGC6, (SIM_BASE + SIM_SCGC6_OFFSET)
.equ SIM_SCGC7, (SIM_BASE + SIM_SCGC7_OFFSET)
.equ SIM_SDID, (SIM_BASE + SIM_SDID_OFFSET)
.equ SIM_SOPT1, (SIM_BASE + SIM_SOPT1_OFFSET)
.equ SIM_SOPT1CFG, (SIM_BASE + SIM_SOPT1CFG_OFFSET)
.equ SIM_SOPT2, (SIM_BASE + SIM_SOPT2_OFFSET)
.equ SIM_SOPT4, (SIM_BASE + SIM_SOPT4_OFFSET)
.equ SIM_SOPT5, (SIM_BASE + SIM_SOPT5_OFFSET)
.equ SIM_SOPT7, (SIM_BASE + SIM_SOPT7_OFFSET)
.equ SIM_SRVCOP, (SIM_BASE + SIM_SRVCOP_OFFSET)
.equ SIM_UIDL, (SIM_BASE + SIM_UIDL_OFFSET)
.equ SIM_UIDMH, (SIM_BASE + SIM_UIDMH_OFFSET)
.equ SIM_UIDML, (SIM_BASE + SIM_UIDML_OFFSET)
//---------------------------------------------------------------
//SIM_CLKDIV1
//31-28:OUTDIV1=clock 1 output divider value
//             :set divider for core/system clock,
//             :from which bus/flash clocks are derived
//             :divide by OUTDIV1 + 1
//27-19:Reserved// read-only// always 0
//18-16:OUTDIV4=clock 4 output divider value
//             :sets divider for bus and flash clocks,
//             :relative to core/system clock
//             :divide by OUTDIV4 + 1
//15-00:Reserved// read-only// always 0
.equ SIM_CLKDIV1_OUTDIV1_MASK, 0xF0000000
.equ SIM_CLKDIV1_OUTDIV1_SHIFT, 28
.equ SIM_CLKDIV1_OUTDIV4_MASK, 0x00070000
.equ SIM_CLKDIV1_OUTDIV4_SHIFT, 16
//---------------------------------------------------------------
//SIM_COPC
//31-04:Reserved// read-only// always 0
//03-02:COPT=COP watchdog timeout
//          :00=disabled
//          :01=timeout after 2^5 LPO cycles or 2^13 bus cycles
//          :10=timeout after 2^8 LPO cycles or 2^16 bus cycles
//          :11=timeout after 2^10 LPO cycles or 2^18 bus cycles
//   01:COPCLKS=COP clock select
//             :0=internal 1 kHz
//             :1=bus clock
//   00:COPW=COP windowed mode
.equ COP_COPT_MASK, 0x0000000C
.equ COP_COPT_SHIFT, 2
.equ COP_COPCLKS_MASK, 0x00000002
.equ COP_COPCLKS_SHIFT, 1
.equ COP_COPW_MASK, 0x00000001
.equ COP_COPW_SHIFT, 1
//---------------------------------------------------------------
//SIM_SCGC4
//1->31-28:Reserved// read-only// always 1
//0->27-24:Reserved// read-only// always 0
//0->   23:SPI1=SPI1 clock gate control (disabled)
//0->   22:SPI0=SPI0 clock gate control (disabled)
//0->21-20:Reserved// read-only// always 0
//0->   19:CMP=comparator clock gate control (disabled)
//0->   18:USBOTG=USB clock gate control (disabled)
//0->17-14:Reserved// read-only// always 0
//0->   13:Reserved// read-only// always 0
//0->   12:UART2=UART2 clock gate control (disabled)
//1->   11:UART1=UART1 clock gate control (disabled)
//0->   10:UART0=UART0 clock gate control (disabled)
//0->09-08:Reserved// read-only// always 0
//0->   07:I2C1=I2C1 clock gate control (disabled)
//0->   06:I2C0=I2C0 clock gate control (disabled)
//1->05-04:Reserved// read-only// always 1
//0->03-00:Reserved// read-only// always 0
.equ SIM_SCGC4_SPI1_MASK, 0x00800000
.equ SIM_SCGC4_SPI1_SHIFT, 23
.equ SIM_SCGC4_SPI0_MASK, 0x00400000
.equ SIM_SCGC4_SPI0_SHIFT, 22
.equ SIM_SCGC4_CMP_MASK, 0x00080000
.equ SIM_SCGC4_CMP_SHIFT, 19
.equ SIM_SCGC4_USBOTG_MASK, 0x00040000
.equ SIM_SCGC4_USBOTG_SHIFT, 18
.equ SIM_SCGC4_UART2_MASK, 0x00001000
.equ SIM_SCGC4_UART2_SHIFT, 12
.equ SIM_SCGC4_UART1_MASK, 0x00000800
.equ SIM_SCGC4_UART1_SHIFT, 11
.equ SIM_SCGC4_UART0_MASK, 0x00000400
.equ SIM_SCGC4_UART0_SHIFT, 10
.equ SIM_SCGC4_I2C1_MASK, 0x00000080
.equ SIM_SCGC4_I2C1_SHIFT, 7
.equ SIM_SCGC4_I2C0_MASK, 0x00000040
.equ SIM_SCGC4_I2C0_SHIFT, 6
//---------------------------------------------------------------
//SIM_SCGC5
//31-20:Reserved// read-only// always 0
//   19:SLCD=segment LCD clock gate control
//18-14:Reserved// read-only// always 0
//   13:PORTE=Port E clock gate control
//   12:PORTD=Port D clock gate control
//   11:PORTC=Port C clock gate control
//   10:PORTB=Port B clock gate control
//    9:PORTA=Port A clock gate control
//08-07:Reserved// read-only// always 1
//    6:Reserved// read-only// always 0
//    5:TSI=TSI access control
//04-02:Reserved// read-only// always 0
//    1:Reserved// read-only// always 0
//    0:LPTMR=Low power timer access control
.equ SIM_SCGC5_SLCD_MASK, 0x00080000
.equ SIM_SCGC5_SLCD_SHIFT, 19
.equ SIM_SCGC5_PORTE_MASK, 0x00002000
.equ SIM_SCGC5_PORTE_SHIFT, 13
.equ SIM_SCGC5_PORTD_MASK, 0x00001000
.equ SIM_SCGC5_PORTD_SHIFT, 12
.equ SIM_SCGC5_PORTC_MASK, 0x00000800
.equ SIM_SCGC5_PORTC_SHIFT, 11
.equ SIM_SCGC5_PORTB_MASK, 0x00000400
.equ SIM_SCGC5_PORTB_SHIFT, 10
.equ SIM_SCGC5_PORTA_MASK, 0x00000200
.equ SIM_SCGC5_PORTA_SHIFT, 9
.equ SIM_SCGC5_TSI_MASK, 0x00000020
.equ SIM_SCGC5_TSI_SHIFT, 6
.equ SIM_SCGC5_LPTMR_MASK, 0x00000001
.equ SIM_SCGC5_LPTMR_SHIFT, 0
//---------------------------------------------------------------
//SIM_SCGC6
//   31:DAC0=DAC0 clock gate control
//   30:(reserved):read-only:0
//   29:RTC=RTC access control
//   28:(reserved):read-only:0
//   27:ADC0=ADC0 clock gate control
//   26:TPM2=TPM2 clock gate control
//   25:TPM1=TMP1 clock gate control
//   24:TPM0=TMP0 clock gate control
//   23:PIT=PIT clock gate control
//22-16:(reserved)
//   15:(reserved)
//14-02:(reserved)
//    1:DMAMUX=DMA mux clock gate control
//    0:FTF=flash memory clock gate control
.equ SIM_SCGC6_DAC0_MASK, 0x80000000
.equ SIM_SCGC6_DAC0_SHIFT, 31
.equ SIM_SCGC6_RTC_MASK, 0x20000000
.equ SIM_SCGC6_RTC_SHIFT, 29
.equ SIM_SCGC6_ADC0_MASK, 0x08000000
.equ SIM_SCGC6_ADC0_SHIFT, 27
.equ SIM_SCGC6_TPM2_MASK, 0x04000000
.equ SIM_SCGC6_TPM2_SHIFT, 26
.equ SIM_SCGC6_TPM1_MASK, 0x02000000
.equ SIM_SCGC6_TPM1_SHIFT, 25
.equ SIM_SCGC6_TPM0_MASK, 0x01000000
.equ SIM_SCGC6_TPM0_SHIFT, 24
.equ SIM_SCGC6_PIT_MASK, 0x00800000
.equ SIM_SCGC6_PIT_SHIFT, 23
.equ SIM_SCGC6_DMAMUX_MASK, 0x00000002
.equ SIM_SCGC6_DMAMUX_SHIFT, 1
.equ SIM_SCGC6_FTF_MASK, 0x00000001
.equ SIM_SCGC6_FTF_SHIFT, 0
//---------------------------------------------------------------
//SIM_SOPT1 (POR or LVD:  0x80000000)
//   31:USBREGEN=USB voltage regulator enable (1)
//   30:USBSSTBY=USB voltage regulator standby during stop, VLPS, LLS, and VLLS (0)
//   29:USBVSTBY=USB voltage regulator standby during VLPS and VLLS (0)
//28-20:(reserved):read-only:000000000
//19-18:OSC32KSEL=32K oscillator clock select (00)
//      (ERCLK32K for sLCD, RTC, and LPTMR)
//                00:System oscillator (OSC32KCLK)
//                01:(reserved)
//                10:RTC_CLKIN
//                11:LPO 1kHz
// 17-6:(reserved):read-only:000000000000
//  5-0:(reserved)
.equ SIM_SOPT1_OSC32KSEL_MASK, 0xC0000
.equ SIM_SOPT1_OSC32KSEL_SHIFT, 18
.equ SIM_SOPT1_USBVSTBY_MASK, 0x20000000
.equ SIM_SOPT1_USBVSTBY_SHIFT, 29
.equ SIM_SOPT1_USBSSTBY_MASK, 0x40000000
.equ SIM_SOPT1_USBSSTBY_SHIFT, 30
.equ SIM_SOPT1_USBREGEN_MASK, 0x80000000
.equ SIM_SOPT1_USBREGEN_SHIFT, 31
//---------------------------------------------------------------
//SIM_SOPT2
//31-28:(reserved):read-only:0
//27-26:UART0SRC=UART0 clock source select
//               00:clock disabled
//               01:MCGFLLCLK or MCGPLLCLK/2
//               10:OSCERCLK
//               11:MCGIRCLK
//25-24:TPMSRC=TPM clock source select
//             00:clock disabled
//             01:MCGFLLCLK or MCGPLLCLK/2
//             10:OSCERCLK
//             11:MCGIRCLK
//23-19:(reserved):read-only:0
//   18:USBSRC=USB clock source select
//             0:USB_CLKIN
//             1:MCGFLLCLK or MCGPLLCLK/2
//   17:(reserved):read-only:0
//   16:PLLFLLSEL=PLL/FLL clock select
//             0:MCGFLLCLK
//             1:MCGPLLCLK/2
//15- 8:(reserved):read-only:0
// 7- 5:CLKOUTSEL=CLKOUT select
//                000:(reserved)
//                001:(reserved)
//                010:bus clock
//                011:LPO clock (1 KHz)
//                100:MCGIRCLK
//                101:(reserved)
//                110:OSCERCLK
//                110:(reserved)
//    4:RTCCLKOUTSEL=RTC clock out select
//                   0:RTC (1 Hz)
//                   1:OSCERCLK
// 3- 0:(reserved):read-only:0
.equ SIM_SOPT2_UART0SRC_MASK, 0x0C000000
.equ SIM_SOPT2_UART0SRC_SHIFT, 26
.equ SIM_SOPT2_TPMSRC_MASK, 0x03000000
.equ SIM_SOPT2_TPMSRC_SHIFT, 24
.equ SIM_SOPT2_USBSRC_MASK, 0x00040000
.equ SIM_SOPT2_USBSRC_SHIFT, 18
.equ SIM_SOPT2_PLLFLLSEL_MASK, 0x00010000
.equ SIM_SOPT2_PLLFLLSEL_SHIFT, 16
.equ SIM_SOPT2_CLKOUTSEL_MASK, 0xE0
.equ SIM_SOPT2_CLKOUTSEL_SHIFT, 5
.equ SIM_SOPT2_RTCCLKOUTSEL_MASK, 0x10
.equ SIM_SOPT2_RTCCLKOUTSEL_SHIFT, 4
//---------------------------------------------------------------
//SIM_SOPT5
//31-20:Reserved// read-only// always 0
//   19:Reserved// read-only// always 0
//   18:UART2ODE=UART2 open drain enable
//   17:UART1ODE=UART1 open drain enable
//   16:UART0ODE=UART0 open drain enable
//15-07:Reserved// read-only// always 0
//   06:UART1TXSRC=UART1 receive data select
//                :0=UART1_RX pin
//                :1=CMP0 output
//05-04:UART1TXSRC=UART1 transmit data select source
//                :00=UART1_TX pin
//                :01=UART1_TX pin modulated with TPM1 channel 0 output
//                :10=UART1_TX pin modulated with TPM2 channel 0 output
//                :11=(reserved)
//   03:Reserved// read-only// always 0
//   02:UART0RXSRC=UART0 receive data select
//                :0=UART0_RX pin
//                :1=CMP0 output
//01-00:UART0TXSRC=UART0 transmit data select source
//                :00=UART0_TX pin
//                :01=UART0_TX pin modulated with TPM1 channel 0 output
//                :10=UART0_TX pin modulated with TPM2 channel 0 output
//                :11=(reserved)
.equ SIM_SOPT5_UART2ODE_MASK, 0x00040000
.equ SIM_SOPT5_UART2ODE_SHIFT, 18
.equ SIM_SOPT5_UART1ODE_MASK, 0x00020000
.equ SIM_SOPT5_UART1ODE_SHIFT, 17
.equ SIM_SOPT5_UART0ODE_MASK, 0x00010000
.equ SIM_SOPT5_UART0ODE_SHIFT, 16
.equ SIM_SOPT5_UART1RXSRC_MASK, 0x00000040
.equ SIM_SOPT5_UART1RXSRC_SHIFT, 6
.equ SIM_SOPT5_UART1TXSRC_MASK, 0x00000030
.equ SIM_SOPT5_UART1TXSRC_SHIFT, 4
.equ SIM_SOPT5_UART0RXSRC_MASK, 0x00000004
.equ SIM_SOPT5_UART0RXSRC_SHIFT, 2
.equ SIM_SOPT5_UART0TXSRC_MASK, 0x00000003
.equ SIM_SOPT5_UART0TXSRC_SHIFT, 0
//---------------------------------------------------------------
//Timer/PWM modules (TPMx)
.equ TPM_SC_OFFSET, 0x00
.equ TPM_CNT_OFFSET, 0x04
.equ TPM_MOD_OFFSET, 0x08
.equ TPM_C0SC_OFFSET, 0x0C
.equ TPM_C0V_OFFSET, 0x10
.equ TPM_C1SC_OFFSET, 0x14
.equ TPM_C1V_OFFSET, 0x18
.equ TPM_C2SC_OFFSET, 0x1C
.equ TPM_C2V_OFFSET, 0x20
.equ TPM_C3SC_OFFSET, 0x24
.equ TPM_C3V_OFFSET, 0x28
.equ TPM_C4SC_OFFSET, 0x2C
.equ TPM_C4V_OFFSET, 0x30
.equ TPM_C5SC_OFFSET, 0x34
.equ TPM_C5V_OFFSET, 0x38
.equ TPM_STATUS_OFFSET, 0x50
.equ TPM_CONF_OFFSET, 0x84
//---------------------------------------------------------------
//TPMx_CnSC:  Channel n Status and Control
//31-8:(reserved):read-only:0
//   7:CHF=channel flag
//         set on channel event
//         write 1 to clear
//   6:CHIE=channel interrupt enable
//   5:MSB=channel mode select B (see selection table below)
//   4:MSA=channel mode select A (see selection table below)
//   3:ELSB=edge or level select B (see selection table below)
//   2:ELSA=edge or level select A (see selection table below)
//   1:(reserved):read-only:0
//   0:DMA=DMA enable
//Mode, Edge, and Level Selection
//MSB:MSA | ELSB:ELSA | Mode           | Configuration
//  0 0   |    0 0    | (none)         | channel disabled
//  0 1   |    0 0    | SW compare     | pin not used
//  1 X   |    0 0    | SW compare     | pin not used
//  0 0   |    0 1    | input capture  | rising edge
//  0 0   |    1 0    | input capture  | falling edge
//  0 0   |    1 1    | input capture  | either edge
//  0 1   |    0 1    | output compare | toggle on match
//  0 1   |    1 0    | output compare | clear on match
//  0 1   |    1 1    | output compare | set on match
//  1 0   |    X 1    | PWM            | low pulse
//  1 0   |    1 0    | PWM            | high pulse
//  1 1   |    X 1    | output compare | pulse high on match
//  1 1   |    1 0    | output compare | pulse low on match
.equ TPM_CnSC_CHF_MASK, 0x80
.equ TPM_CnSC_CHF_SHIFT, 7
.equ TPM_CnSC_CHIE_MASK, 0x40
.equ TPM_CnSC_CHIE_SHIFT, 6
.equ TPM_CnSC_MSB_MASK, 0x20
.equ TPM_CnSC_MSB_SHIFT, 5
.equ TPM_CnSC_MSA_MASK, 0x10
.equ TPM_CnSC_MSA_SHIFT, 4
.equ TPM_CnSC_ELSB_MASK, 0x08
.equ TPM_CnSC_ELSB_SHIFT, 3
.equ TPM_CnSC_ELSA_MASK, 0x04
.equ TPM_CnSC_ELSA_SHIFT, 2
.equ TPM_CnSC_CDMA_MASK, 0x01
.equ TPM_CnSC_CDMA_SHIFT, 0
//---------------------------------------------------------------
//TPMx_CnV:  Channel n Value
//31-16:(reserved):read-only:0
//16- 0:MOD (all bytes must be written at the same time)
.equ TPM_CnV_VAL_MASK, 0xFFFF
.equ TPM_CnV_VAL_SHIFT, 0
//---------------------------------------------------------------
//TPMx_CONF:  Configuration
//31-28:(reserved):read-only:0
//27-24:TRGSEL=trigger select
//             should be changed only when counter disabled
//             0000:EXTRG_IN (external trigger pin input)
//             0001:CMP0 output
//             0010:(reserved)
//             0011:(reserved)
//             0100:PIT trigger 0
//             0101:PIT trigger 1
//             0110:(reserved)
//             0111:(reserved)
//             1000:TPM0 overflow
//             1001:TPM1 overflow
//             1010:TPM2 overflow
//             1011:(reserved)
//             1100:RTC alarm
//             1101:RTC seconds
//             1110:LPTMR trigger
//             1111:(reserved)
//23-19:(reserved):read-only:0
//   18:CROT=counter reload on trigger
//           should be changed only when counter disabled
//   17:CSOO=counter stop on overflow
//           should be changed only when counter disabled
//   16:CSOT=counter start on trigger
//           should be changed only when counter disabled
//15-10:(reserved):read-only:0
//    9:GTBEEN=global time base enable
//    8:(reserved):read-only:0
// 7- 6:DBGMODE=debug mode
//              00:paused during debug, and during debug
//                 trigger and input caputure events ignored
//              01:(reserved)
//              10:(reserved)
//              11:counter continues during debug
//    5:DOZEEN=doze enable
//             0:counter continues during debug
//             1:paused during debug, and during debug
//               trigger and input caputure events ignored
// 4- 0:(reserved):read-only:0
.equ TPM_CONF_TRGSEL_MASK, 0x0F000000
.equ TPM_CONF_TRGSEL_SHIFT, 24
.equ TPM_CONF_CROT_MASK, 0x00040000
.equ TPM_CONF_CROT_SHIFT, 18
.equ TPM_CONF_CSOO_MASK, 0x00020000
.equ TPM_CONF_CSOO_SHIFT, 17
.equ TPM_CONF_CSOT_MASK, 0x00010000
.equ TPM_CONF_CSOT_SHIFT, 16
.equ TPM_CONF_GTBEEN_MASK, 0x200
.equ TPM_CONF_GTBEEN_SHIFT, 9
.equ TPM_CONF_DBGMODE_MASK, 0xC0
.equ TPM_CONF_DBGMODE_SHIFT, 6
.equ TPM_CONF_DOZEEN_MASK, 0x20
.equ TPM_CONF_DOZEEN_SHIFT, 5
//---------------------------------------------------------------
//TPMx_MOD:  Modulo
//31-16:(reserved):read-only:0
//16- 0:MOD (all bytes must be written at the same time)
.equ TPM_MOD_MOD_MASK, 0xFFFF
.equ TPM_MOD_MOD_SHIFT, 0xFFFF
//---------------------------------------------------------------
//TPMx_SC:  Status and Control
//31-9:(reserved):read-only:0
//   8:DMA=DMA enable
//   7:TOF=timer overflow flag
//         set on count after TPMx_CNT = TPMx_MOD
//         write 1 to clear
//   6:TOIE=timer overflow interrupt enable
//   5:CPWMS=center-aligned PWM select
//           0:edge align (up count)
//           1:center align (up-down count)
// 4-3:CMOD=clock mode selection
//          00:counter disabled
//          01:TPMx_CNT increments on every TPMx clock
//          10:TPMx_CNT increments on rising edge of TPMx_EXTCLK
//          11:(reserved)
// 2-0:PS=prescale factor selection
//        can be written only when counter is disabled
//        count clock = CMOD selected clock / 2^PS
.equ TPM_SC_DMA_MASK, 0x100
.equ TPM_SC_DMA_SHIFT, 8
.equ TPM_SC_TOF_MASK, 0x80
.equ TPM_SC_TOF_SHIFT, 7
.equ TPM_SC_TOIE_MASK, 0x40
.equ TPM_SC_TOIE_SHIFT, 6
.equ TPM_SC_CPWMS_MASK, 0x20
.equ TPM_SC_CPWMS_SHIFT, 5
.equ TPM_SC_CMOD_MASK, 0x18
.equ TPM_SC_CMOD_SHIFT, 3
.equ TPM_SC_PS_MASK, 0x07
.equ TPM_SC_PS_SHIFT, 0
//---------------------------------------------------------------
//TPMx_STATUS:  Capture and Compare Status
//31-9:(reserved):read-only:0
//   8:TOF=timer overflow flag=TPMx_SC.TOF
// 7-6:(reserved):read-only:0
//   5:CH5F=channel 5 flag=TPMx_C5SC.CHF
//   4:CH4F=channel 4 flag=TPMx_C4SC.CHF
//   3:CH3F=channel 3 flag=TPMx_C3SC.CHF
//   2:CH2F=channel 2 flag=TPMx_C2SC.CHF
//   1:CH1F=channel 1 flag=TPMx_C1SC.CHF
//   0:CH0F=channel 0 flag=TPMx_C0SC.CHF
.equ TPM_STATUS_TOF_MASK, 0x100
.equ TPM_STATUS_TOF_SHIFT, 8
.equ TPM_STATUS_CH5F_MASK, 0x20
.equ TPM_STATUS_CH5F_SHIFT, 5
.equ TPM_STATUS_CH4F_MASK, 0x10
.equ TPM_STATUS_CH4F_SHIFT, 4
.equ TPM_STATUS_CH3F_MASK, 0x08
.equ TPM_STATUS_CH3F_SHIFT, 3
.equ TPM_STATUS_CH2F_MASK, 0x04
.equ TPM_STATUS_CH2F_SHIFT, 2
.equ TPM_STATUS_CH1F_MASK, 0x02
.equ TPM_STATUS_CH1F_SHIFT, 1
.equ TPM_STATUS_CH0F_MASK, 0x01
.equ TPM_STATUS_CH0F_SHIFT, 0
//---------------------------------------------------------------
//Timer/PWM module 0 (TPM0)
.equ TPM0_BASE, 0x40038000
.equ TPM0_SC, (TPM0_BASE + TPM_SC_OFFSET    )
.equ TPM0_CNT, (TPM0_BASE + TPM_CNT_OFFSET   )
.equ TPM0_MOD, (TPM0_BASE + TPM_MOD_OFFSET   )
.equ TPM0_C0SC, (TPM0_BASE + TPM_C0SC_OFFSET  )
.equ TPM0_C0V, (TPM0_BASE + TPM_C0V_OFFSET   )
.equ TPM0_C1SC, (TPM0_BASE + TPM_C1SC_OFFSET  )
.equ TPM0_C1V, (TPM0_BASE + TPM_C1V_OFFSET   )
.equ TPM0_C2SC, (TPM0_BASE + TPM_C2SC_OFFSET  )
.equ TPM0_C2V, (TPM0_BASE + TPM_C2V_OFFSET   )
.equ TPM0_C3SC, (TPM0_BASE + TPM_C3SC_OFFSET  )
.equ TPM0_C3V, (TPM0_BASE + TPM_C3V_OFFSET   )
.equ TPM0_C4SC, (TPM0_BASE + TPM_C4SC_OFFSET  )
.equ TPM0_C4V, (TPM0_BASE + TPM_C4V_OFFSET   )
.equ TPM0_C5SC, (TPM0_BASE + TPM_C5SC_OFFSET  )
.equ TPM0_C5V, (TPM0_BASE + TPM_C5V_OFFSET   )
.equ TPM0_STATUS, (TPM0_BASE + TPM_STATUS_OFFSET)
.equ TPM0_CONF, (TPM0_BASE + TPM_CONF_OFFSET  )
//---------------------------------------------------------------
//Timer/PWM module 1 (TPM1)
.equ TPM1_BASE, 0x40039000
.equ TPM1_SC, (TPM1_BASE + TPM_SC_OFFSET    )
.equ TPM1_CNT, (TPM1_BASE + TPM_CNT_OFFSET   )
.equ TPM1_MOD, (TPM1_BASE + TPM_MOD_OFFSET   )
.equ TPM1_C0SC, (TPM1_BASE + TPM_C0SC_OFFSET  )
.equ TPM1_C0V, (TPM1_BASE + TPM_C0V_OFFSET   )
.equ TPM1_C1SC, (TPM1_BASE + TPM_C1SC_OFFSET  )
.equ TPM1_C1V, (TPM1_BASE + TPM_C1V_OFFSET   )
.equ TPM1_C2SC, (TPM1_BASE + TPM_C2SC_OFFSET  )
.equ TPM1_C2V, (TPM1_BASE + TPM_C2V_OFFSET   )
.equ TPM1_C3SC, (TPM1_BASE + TPM_C3SC_OFFSET  )
.equ TPM1_C3V, (TPM1_BASE + TPM_C3V_OFFSET   )
.equ TPM1_C4SC, (TPM1_BASE + TPM_C4SC_OFFSET  )
.equ TPM1_C4V, (TPM1_BASE + TPM_C4V_OFFSET   )
.equ TPM1_C5SC, (TPM1_BASE + TPM_C5SC_OFFSET  )
.equ TPM1_C5V, (TPM1_BASE + TPM_C5V_OFFSET   )
.equ TPM1_STATUS, (TPM1_BASE + TPM_STATUS_OFFSET)
.equ TPM1_CONF, (TPM1_BASE + TPM_CONF_OFFSET  )
//---------------------------------------------------------------
//Timer/PWM module 2 (TPM2)
.equ TPM2_BASE, 0x4003A000
.equ TPM2_SC, (TPM2_BASE + TPM_SC_OFFSET    )
.equ TPM2_CNT, (TPM2_BASE + TPM_CNT_OFFSET   )
.equ TPM2_MOD, (TPM2_BASE + TPM_MOD_OFFSET   )
.equ TPM2_C0SC, (TPM2_BASE + TPM_C0SC_OFFSET  )
.equ TPM2_C0V, (TPM2_BASE + TPM_C0V_OFFSET   )
.equ TPM2_C1SC, (TPM2_BASE + TPM_C1SC_OFFSET  )
.equ TPM2_C1V, (TPM2_BASE + TPM_C1V_OFFSET   )
.equ TPM2_C2SC, (TPM2_BASE + TPM_C2SC_OFFSET  )
.equ TPM2_C2V, (TPM2_BASE + TPM_C2V_OFFSET   )
.equ TPM2_C3SC, (TPM2_BASE + TPM_C3SC_OFFSET  )
.equ TPM2_C3V, (TPM2_BASE + TPM_C3V_OFFSET   )
.equ TPM2_C4SC, (TPM2_BASE + TPM_C4SC_OFFSET  )
.equ TPM2_C4V, (TPM2_BASE + TPM_C4V_OFFSET   )
.equ TPM2_C5SC, (TPM2_BASE + TPM_C5SC_OFFSET  )
.equ TPM2_C5V, (TPM2_BASE + TPM_C5V_OFFSET   )
.equ TPM2_STATUS, (TPM2_BASE + TPM_STATUS_OFFSET)
.equ TPM2_CONF, (TPM2_BASE + TPM_CONF_OFFSET  )
//---------------------------------------------------------------
//UART 0
.equ UART0_BASE, 0x4006A000
.equ UART0_BDH_OFFSET, 0x00
.equ UART0_BDL_OFFSET, 0x01
.equ UART0_C1_OFFSET, 0x02
.equ UART0_C2_OFFSET, 0x03
.equ UART0_S1_OFFSET, 0x04
.equ UART0_S2_OFFSET, 0x05
.equ UART0_C3_OFFSET, 0x06
.equ UART0_D_OFFSET, 0x07
.equ UART0_MA1_OFFSET, 0x08
.equ UART0_MA2_OFFSET, 0x09
.equ UART0_C4_OFFSET, 0x0A
.equ UART0_C5_OFFSET, 0x0B
.equ UART0_BDH, (UART0_BASE + UART0_BDH_OFFSET)
.equ UART0_BDL, (UART0_BASE + UART0_BDL_OFFSET)
.equ UART0_C1, (UART0_BASE + UART0_C1_OFFSET)
.equ UART0_C2, (UART0_BASE + UART0_C2_OFFSET)
.equ UART0_S1, (UART0_BASE + UART0_S1_OFFSET)
.equ UART0_S2, (UART0_BASE + UART0_S2_OFFSET)
.equ UART0_C3, (UART0_BASE + UART0_C3_OFFSET)
.equ UART0_D, (UART0_BASE + UART0_D_OFFSET)
.equ UART0_MA1, (UART0_BASE + UART0_MA1_OFFSET)
.equ UART0_MA2, (UART0_BASE + UART0_MA2_OFFSET)
.equ UART0_C4, (UART0_BASE + UART0_C4_OFFSET)
.equ UART0_C5, (UART0_BASE + UART0_C5_OFFSET)
//---------------------------------------------------------------
//UART0_BDH
//  7:LBKDIE=LIN break detect IE
//  6:RXEDGIE=RxD input active edge IE
//  5:SBNS=Stop bit number select
//4-0:SBR[12:0] (BUSCLK / (16 x 9600))
.equ UART0_BDH_LBKDIE_MASK, 0x80
.equ UART0_BDH_LBKDIE_SHIFT, 7
.equ UART0_BDH_RXEDGIE_MASK, 0x40
.equ UART0_BDH_RXEDGIE_SHIFT, 6
.equ UART0_BDH_SBNS_MASK, 0x20
.equ UART0_BDH_SBNS_SHIFT, 5
.equ UART0_BDH_SBR_MASK, 0x1F
.equ UART0_BDH_SBR_SHIFT, 0
//---------------------------------------------------------------
//UART0_BDL
//7-0:SBR[7:0] (BUSCLK / 16 x 9600))
.equ UART0_BDL_SBR_MASK, 0xFF
.equ UART0_BDL_SBR_SHIFT, 0
//---------------------------------------------------------------
//UART0_C1
//7:LOOPS=loop mode select (normal)
//6:DOZEEN=UART disabled in wait mode (enabled)
//5:RSRC=receiver source select (internal--no effect LOOPS=0)
//4:M=9- or 8-bit mode select (1 start, 8 data [lsb first], 1 stop)
//3:WAKE=receiver wakeup method select (idle)
//2:IDLE=idle line type select (idle begins after start bit)
//1:PE=parity enable (disabled)
//0:PT=parity type (even parity--no effect PE=0)
.equ UART0_C1_LOOPS_MASK, 0x80
.equ UART0_C1_LOOPS_SHIFT, 7
.equ UART0_C1_DOZEEN_MASK, 0x40
.equ UART0_C1_DOZEEN_SHIFT, 6
.equ UART0_C1_RSRC_MASK, 0x20
.equ UART0_C1_RSRC_SHIFT, 5
.equ UART0_C1_M_MASK, 0x10
.equ UART0_C1_M_SHIFT, 4
.equ UART0_C1_WAKE_MASK, 0x08
.equ UART0_C1_WAKE_SHIFT, 3
.equ UART0_C1_ILT_MASK, 0x04
.equ UART0_C1_ILT_SHIFT, 2
.equ UART0_C1_PE_MASK, 0x02
.equ UART0_C1_PE_SHIFT, 1
.equ UART0_C1_PT_MASK, 0x01
.equ UART0_C1_PT_SHIFT, 0
//---------------------------------------------------------------
//UART0_C2
//7:TIE=transmitter IE for TDRE (disabled)
//6:TCIE=trasmission complete IE for TC (disabled)
//5:RIE=receiver IE for RDRF (disabled)
//4:ILIE=idle line IE for IDLE (disabled)
//3:TE=transmitter enable (disabled)
//2:RE=receiver enable (disabled)
//1:RWU=receiver wakeup control (normal)
//0:SBK=send break (disabled, normal)
.equ UART0_C2_TIE_MASK, 0x80
.equ UART0_C2_TIE_SHIFT, 7
.equ UART0_C2_TCIE_MASK, 0x40
.equ UART0_C2_TCIE_SHIFT, 6
.equ UART0_C2_RIE_MASK, 0x20
.equ UART0_C2_RIE_SHIFT, 5
.equ UART0_C2_ILIE_MASK, 0x10
.equ UART0_C2_ILIE_SHIFT, 4
.equ UART0_C2_TE_MASK, 0x08
.equ UART0_C2_TE_SHIFT, 3
.equ UART0_C2_RE_MASK, 0x04
.equ UART0_C2_RE_SHIFT, 2
.equ UART0_C2_RWU_MASK, 0x02
.equ UART0_C2_RWU_SHIFT, 1
.equ UART0_C2_SBK_MASK, 0x01
.equ UART0_C2_SBK_SHIFT, 0
//---------------------------------------------------------------
//UART0_C3
//7:R8T9=Receive bit 8// transmit bit 9 (not used M=0)
//6:R9T8=Receive bit 9// transmit bit 8 (not used M=0)
//5:TXDIR=TxD pin direction in single-wire mode 
//                        (input--no effect LOOPS=0)
//4:TXINV=transmit data inversion (not invereted)
//3:ORIE=overrun IE for OR (disabled)
//2:NEIE=noise error IE for NF (disabled)
//1:FEIE=framing error IE for FE (disabled)
//0:PEIE=parity error IE for PF (disabled)
.equ UART0_C3_R8T9_MASK, 0x80
.equ UART0_C3_R8T9_SHIFT, 7
.equ UART0_C3_R9T8_MASK, 0x40
.equ UART0_C3_R9T8_SHIFT, 6
.equ UART0_C3_TXDIR_MASK, 0x20
.equ UART0_C3_TXDIR_SHIFT, 5
.equ UART0_C3_TXINV_MASK, 0x10
.equ UART0_C3_TXINV_SHIFT, 4
.equ UART0_C3_ORIE_MASK, 0x08
.equ UART0_C3_ORIE_SHIFT, 3
.equ UART0_C3_NEIE_MASK, 0x04
.equ UART0_C3_NEIE_SHIFT, 2
.equ UART0_C3_FEIE_MASK, 0x02
.equ UART0_C3_FEIE_SHIFT, 1
.equ UART0_C3_PEIE_MASK, 0x01
.equ UART0_C3_PEIE_SHIFT, 0
//---------------------------------------------------------------
//UART0_C4
//  7:MAEN1=Match address mode enable 1 (disabled)
//  6:MAEN2=Match address mode enable 2 (disabled)
//  5:M10=10-bit mode select (not selected)
//4-0:OSR=Over sampling ratio (01111)
//        00000 <= OSR <= 00010:  (invalid// defaults to ratio = 16)        
//        00011 <= OSR <= 11111:  ratio = OSR + 1
.equ UART0_C4_MAEN1_MASK, 0x80
.equ UART0_C4_MAEN1_SHIFT, 7
.equ UART0_C4_MAEN2_MASK, 0x40
.equ UART0_C4_MAEN2_SHIFT, 6
.equ UART0_C4_M10_MASK, 0x20
.equ UART0_C4_M10_SHIFT, 5
.equ UART0_C4_OSR_MASK, 0x1F
.equ UART0_C4_OSR_SHIFT, 0
//---------------------------------------------------------------
//UART0_C5
//  7:TDMAE=Transmitter DMA enable (disabled)
//  6:(reserved):  read-only:  0
//  5:RDMAE=Receiver full DMA enable (disabled)
//4-2:(reserved):  read-only:  000
//  1:BOTHEDGE=Both edge sampling (only rising edge)
//  0:RESYNCDIS=Resynchronization disable (enabled)
.equ UART0_C5_TDMAE_MASK, 0x80
.equ UART0_C5_TDMAE_SHIFT, 7
.equ UART0_C5_RDMAE_MASK, 0x20
.equ UART0_C5_RDMAE_SHIFT, 5
.equ UART0_C5_BOTHEDGE_MASK, 0x02
.equ UART0_C5_BOTHEDGE_SHIFT, 1
.equ UART0_C5_RESYNCDIS_MASK, 0x01
.equ UART0_C5_RESYNCDIS_SHIFT, 0
//---------------------------------------------------------------
//UART0_D
//7:R7T7=Receive data buffer bit 7// 
//       transmit data buffer bit 7
//6:R6T6=Receive data buffer bit 6// 
//       transmit data buffer bit 6
//5:R5T5=Receive data buffer bit 5// 
//       transmit data buffer bit 5
//4:R4T4=Receive data buffer bit 4// 
//       transmit data buffer bit 4
//3:R3T3=Receive data buffer bit 3// 
//       transmit data buffer bit 3
//2:R2T2=Receive data buffer bit 2// 
//       transmit data buffer bit 2
//1:R1T1=Receive data buffer bit 1// 
//       transmit data buffer bit 1
//0:R0T0=Receive data buffer bit 0// 
//       transmit data buffer bit 0
.equ UART0_D_R7T7_MASK, 0x80
.equ UART0_D_R7T7_SHIFT, 7
.equ UART0_D_R6T6_MASK, 0x40
.equ UART0_D_R6T6_SHIFT, 6
.equ UART0_D_R5T5_MASK, 0x20
.equ UART0_D_R5T5_SHIFT, 5
.equ UART0_D_R4T4_MASK, 0x10
.equ UART0_D_R4T4_SHIFT, 4
.equ UART0_D_R3T3_MASK, 0x08
.equ UART0_D_R3T3_SHIFT, 3
.equ UART0_D_R2T2_MASK, 0x04
.equ UART0_D_R2T2_SHIFT, 2
.equ UART0_D_R1T1_MASK, 0x02
.equ UART0_D_R1T1_SHIFT, 1
.equ UART0_D_R0T0_MASK, 0x01
.equ UART0_D_R0T0_SHIFT, 0
//---------------------------------------------------------------
//UART0_MA1
//7-0:MA=Match address
.equ UART0_MA1_MA_MASK, 0xFF
.equ UART0_MA1_MA_SHIFT, 0
//---------------------------------------------------------------
//UART0_MA2
//7-0:MA=Match address
.equ UART0_MA2_MA_MASK, 0xFF
.equ UART0_MA2_MA_SHIFT, 0
//---------------------------------------------------------------
//UART0_S1
//7:TDRE=transmit data register empty flag
//6:TC=transmission complete flag
//5:RDRF=receive data register full flag
//4:IDLE=idle line flag
//3:OR=receiver overrun flag
//2:NF=noise flag
//1:FE=framing error flag
//0:PF=parity error flag
.equ UART0_S1_TDRE_MASK, 0x80
.equ UART0_S1_TDRE_SHIFT, 7
.equ UART0_S1_TC_MASK, 0x40
.equ UART0_S1_TC_SHIFT, 6
.equ UART0_S1_RDRF_MASK, 0x20
.equ UART0_S1_RDRF_SHIFT, 5
.equ UART0_S1_IDLE_MASK, 0x10
.equ UART0_S1_IDLE_SHIFT, 4
.equ UART0_S1_OR_MASK, 0x08
.equ UART0_S1_OR_SHIFT, 3
.equ UART0_S1_NF_MASK, 0x04
.equ UART0_S1_NF_SHIFT, 2
.equ UART0_S1_FE_MASK, 0x02
.equ UART0_S1_FE_SHIFT, 1
.equ UART0_S1_PF_MASK, 0x01
.equ UART0_S1_PF_SHIFT, 0
//---------------------------------------------------------------
//UART0_S2
//7:LBKDIF=LIN break detect interrupt flag
//6:RXEDGIF=RxD pin active edge interrupt flag
//5:MSBF=MSB first (LSB first)
//4:RXINV=receive data inversion (not inverted)
//3:RWUID=receive wake-up idle detect (not detected)
//2:BRK13=break character generation length (10 bit times)
//1:LBKDE=LIN break detect enable (10 bit times)
//0:RAF=receiver active flag
.equ UART0_S2_LBKDIF_MASK, 0x80
.equ UART0_S2_LBKDIF_SHIFT, 7
.equ UART0_S2_RXEDGIF_MASK, 0x40
.equ UART0_S2_RXEDGIF_SHIFT, 6
.equ UART0_S2_MSBF_MASK, 0x20
.equ UART0_S2_MSBF_SHIFT, 5
.equ UART0_S2_RXINV_MASK, 0x10
.equ UART0_S2_RXINV_SHIFT, 4
.equ UART0_S2_RWUID_MASK, 0x08
.equ UART0_S2_RWUID_SHIFT, 3
.equ UART0_S2_BRK13_MASK, 0x04
.equ UART0_S2_BRK13_SHIFT, 2
.equ UART0_S2_LBKDE_MASK, 0x02
.equ UART0_S2_LBKDE_SHIFT, 1
.equ UART0_S2_RAF_MASK, 0x01
.equ UART0_S2_RAF_SHIFT, 0
//---------------------------------------------------------------
//UART 1 and UART2
.equ UART_BDH_OFFSET, 0x00
.equ UART_BDL_OFFSET, 0x01
.equ UART_C1_OFFSET, 0x02
.equ UART_C2_OFFSET, 0x03
.equ UART_S1_OFFSET, 0x04
.equ UART_S2_OFFSET, 0x05
.equ UART_C3_OFFSET, 0x06
.equ UART_D_OFFSET, 0x07
.equ UART_C4_OFFSET, 0x08
//---------------------------------------------------------------
//UART 1
.equ UART1_BASE, 0x4006B000
.equ UART1_BDH, (UART1_BASE + UART_BDH_OFFSET)
.equ UART1_BDL, (UART1_BASE + UART_BDL_OFFSET)
.equ UART1_C1, (UART1_BASE + UART_C1_OFFSET)
.equ UART1_C2, (UART1_BASE + UART_C2_OFFSET)
.equ UART1_S1, (UART1_BASE + UART_S1_OFFSET)
.equ UART1_S2, (UART1_BASE + UART_S2_OFFSET)
.equ UART1_C3, (UART1_BASE + UART_C3_OFFSET)
.equ UART1_D, (UART1_BASE + UART_D_OFFSET)
.equ UART1_C4, (UART1_BASE + UART_C4_OFFSET)
//---------------------------------------------------------------
//UART 2
.equ UART2_BASE, 0x4006C000
.equ UART2_BDH, (UART2_BASE + UART_BDH_OFFSET)
.equ UART2_BDL, (UART2_BASE + UART_BDL_OFFSET)
.equ UART2_C1, (UART2_BASE + UART_C1_OFFSET)
.equ UART2_C2, (UART2_BASE + UART_C2_OFFSET)
.equ UART2_S1, (UART2_BASE + UART_S1_OFFSET)
.equ UART2_S2, (UART2_BASE + UART_S2_OFFSET)
.equ UART2_C3, (UART2_BASE + UART_C3_OFFSET)
.equ UART2_D, (UART2_BASE + UART_D_OFFSET)
.equ UART2_C4, (UART2_BASE + UART_C4_OFFSET)
//---------------------------------------------------------------
//UARTx_BDH
//  7:LBKDIE=LIN break detect IE
//  6:RXEDGIE=RxD input active edge IE
//  5:SBNS=Stop bit number select
//4-0:SBR[12:0] (BUSCLK / (16 x 9600))
.equ UART_BDH_LBKDIE_MASK, 0x80
.equ UART_BDH_LBKDIE_SHIFT, 7
.equ UART_BDH_RXEDGIE_MASK, 0x40
.equ UART_BDH_RXEDGIE_SHIFT, 6
.equ UART_BDH_SBNS_MASK, 0x20
.equ UART_BDH_SBNS_SHIFT, 5
.equ UART_BDH_SBR_MASK, 0x1F
.equ UART_BDH_SBR_SHIFT, 0
//---------------------------------------------------------------
//UARTx_BDL
//7-0:SBR[7:0] (BUSCLK / 16 x 9600))
.equ UART_BDL_SBR_MASK, 0xFF
.equ UART_BDL_SBR_SHIFT, 0
//---------------------------------------------------------------
//UARTx_C1
//7:LOOPS=loops select (normal)
//6:UARTSWAI=UART stop in wait mode (disabled)
//5:RSRC=receiver source select (internal--no effect LOOPS=0)
//4:M=9- or 8-bit mode select (1 start, 8 data [lsb first], 1 stop)
//3:WAKE=receiver wakeup method select (idle)
//2:IDLE=idle line type select (idle begins after start bit)
//1:PE=parity enable (disabled)
//0:PT=parity type (even parity--no effect PE=0)
.equ UART_C1_LOOPS_MASK, 0x80
.equ UART_C1_LOOPS_SHIFT, 7
.equ UART_C1_UARTSWAI_MASK, 0x40
.equ UART_C1_UARTSWAI_SHIFT, 6
.equ UART_C1_RSRC_MASK, 0x20
.equ UART_C1_RSRC_SHIFT, 5
.equ UART_C1_M_MASK, 0x10
.equ UART_C1_M_SHIFT, 4
.equ UART_C1_WAKE_MASK, 0x08
.equ UART_C1_WAKE_SHIFT, 3
.equ UART_C1_ILT_MASK, 0x04
.equ UART_C1_ILT_SHIFT, 2
.equ UART_C1_PE_MASK, 0x02
.equ UART_C1_PE_SHIFT, 1
.equ UART_C1_PT_MASK, 0x01
.equ UART_C1_PT_SHIFT, 0
//---------------------------------------------------------------
//UARTx_C2
//7:TIE=transmit IE for TDRE (disabled)
//6:TCIE=trasmission complete IE for TC (disabled)
//5:RIE=receiver IE for RDRF (disabled)
//4:ILIE=idle line IE for IDLE (disabled)
//3:TE=transmitter enable (enabled)
//2:RE=receiver enable (enabled)
//1:RWU=receiver wakeup control (normal)
//0:SBK=send break (disabled, normal)
.equ UART_C2_TIE_MASK, 0x80
.equ UART_C2_TIE_SHIFT, 7
.equ UART_C2_TCIE_MASK, 0x40
.equ UART_C2_TCIE_SHIFT, 6
.equ UART_C2_RIE_MASK, 0x20
.equ UART_C2_RIE_SHIFT, 5
.equ UART_C2_ILIE_MASK, 0x10
.equ UART_C2_ILIE_SHIFT, 4
.equ UART_C2_TE_MASK, 0x08
.equ UART_C2_TE_SHIFT, 3
.equ UART_C2_RE_MASK, 0x04
.equ UART_C2_RE_SHIFT, 2
.equ UART_C2_RWU_MASK, 0x02
.equ UART_C2_RWU_SHIFT, 1
.equ UART_C2_SBK_MASK, 0x01
.equ UART_C2_SBK_SHIFT, 0
//---------------------------------------------------------------
//UARTx_C3
//7:R8=9th data bit for receiver (not used M=0)
//6:T8=9th data bit for transmitter (not used M=0)
//5:TXDIR=TxD pin direction in single-wire mode (no effect LOOPS=0)
//4:TXINV=transmit data inversion (not invereted)
//3:ORIE=overrun IE for OR (disabled)
//2:NEIE=noise error IE for NF (disabled)
//1:FEIE=framing error IE for FE (disabled)
//0:PEIE=parity error IE for PF (disabled)
.equ UART_C3_R8_MASK, 0x80
.equ UART_C3_R8_SHIFT, 7
.equ UART_C3_T8_MASK, 0x40
.equ UART_C3_T8_SHIFT, 6
.equ UART_C3_TXDIR_MASK, 0x20
.equ UART_C3_TXDIR_SHIFT, 5
.equ UART_C3_TXINV_MASK, 0x10
.equ UART_C3_TXINV_SHIFT, 4
.equ UART_C3_ORIE_MASK, 0x08
.equ UART_C3_ORIE_SHIFT, 3
.equ UART_C3_NEIE_MASK, 0x04
.equ UART_C3_NEIE_SHIFT, 2
.equ UART_C3_FEIE_MASK, 0x02
.equ UART_C3_FEIE_SHIFT, 1
.equ UART_C3_PEIE_MASK, 0x01
.equ UART_C3_PEIE_SHIFT, 0
//---------------------------------------------------------------
//UARTx_C4
//  7:TDMAS=transmitter DMA select (disabled)
//  6:(reserved)// read-only// always 0
//  5:RDMAS=receiver full DMA select (disabled)
//  4:(reserved)// read-only// always 0
//  3:(reserved)// read-only// always 0
//2-0:(reserved)// read-only// always 0
.equ UART_C4_TDMAS_MASK, 0x80
.equ UART_C4_TDMAS_SHIFT, 7
.equ UART_C4_RDMAS_MASK, 0x20
.equ UART_C4_RDMAS_SHIFT, 5
//---------------------------------------------------------------
//UARTx_S1
//7:TDRE=transmit data register empty flag
//6:TC=transmission complete flag
//5:RDRF=receive data register full flag
//4:IDLE=idle line flag
//3:OR=receiver overrun flag
//2:NF=noise flag
//1:FE=framing error flag
//0:PF=parity error flag
.equ UART_S1_TDRE_MASK, 0x80
.equ UART_S1_TDRE_SHIFT, 7
.equ UART_S1_TC_MASK, 0x40
.equ UART_S1_TC_SHIFT, 6
.equ UART_S1_RDRF_MASK, 0x20
.equ UART_S1_RDRF_SHIFT, 5
.equ UART_S1_IDLE_MASK, 0x10
.equ UART_S1_IDLE_SHIFT, 4
.equ UART_S1_OR_MASK, 0x08
.equ UART_S1_OR_SHIFT, 3
.equ UART_S1_NF_MASK, 0x04
.equ UART_S1_NF_SHIFT, 2
.equ UART_S1_FE_MASK, 0x02
.equ UART_S1_FE_SHIFT, 1
.equ UART_S1_PF_MASK, 0x01
.equ UART_S1_PF_SHIFT, 0
//---------------------------------------------------------------
//UARTx_S2
//7:LBKDIF=LIN break detect interrupt flag
//6:RXEDGIF=RxD pin active edge interrupt flag
//5:(reserved)//read-only// always 0
//4:RXINV=receive data inversion
//3:RWUID=receive wake-up idle detect
//2:BRK13=break character generation length
//1:LBKDE=LIN break detect enable
//0:RAF=receiver active flag
.equ UART_S2_LBKDIF_MASK, 0x80
.equ UART_S2_LBKDIF_SHIFT, 7
.equ UART_S2_RXEDGIF_MASK, 0x40
.equ UART_S2_RXEDGIF_SHIFT, 6
.equ UART_S2_RXINV_MASK, 0x10
.equ UART_S2_RXINV_SHIFT, 4
.equ UART_S2_RWUID_MASK, 0x08
.equ UART_S2_RWUID_SHIFT, 3
.equ UART_S2_BRK13_MASK, 0x04
.equ UART_S2_BRK13_SHIFT, 2
.equ UART_S2_LBKDE_MASK, 0x02
.equ UART_S2_LBKDE_SHIFT, 1
.equ UART_S2_RAF_MASK, 0x01
.equ UART_S2_RAF_SHIFT, 0
//public int getRed()           {return red//}
