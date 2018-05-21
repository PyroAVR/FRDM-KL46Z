//****************************************************************
//Descriptive comment header goes here.
//(What does the program do?)
//Name:  <Your name here>
//Date:  <Date completed here>
//Class:  CMPE-250
//Section:  <Your lab section, day, and time here>
//---------------------------------------------------------------
//Keil Template for KL46 Assembly with Keil C startup
//R. W. Melton
//November 13, 2017
//****************************************************************
//Assembler directives
.syntax unified
.equ  MIXED_ASM_C, true
//****************************************************************
//Include files
.include "MKL46Z4.s"     //Included by start.s
//****************************************************************
//EQUates
.equ TPM_CnV_PWM_DUTY_2ms, 6000
.equ TPM_CnV_PWM_DUTY_1ms, 2200
.equ pwm_2ms, TPM_CnV_PWM_DUTY_2ms
.equ pwm_1ms, TPM_CnV_PWM_DUTY_1ms
.equ dac0_steps, 4096    //guessing!
.equ servo_positions, 5
//****************************************************************
//MACROs
//****************************************************************
//Program
//C source will contain main ()
//Only subroutines and ISRs in this assembly source
.text
//>>>>> begin subroutine code <<<<<
.global init_rxtx
init_rxtx://   proc    {r0-r14}, {}
            push {r0-r2, lr}
            ldr  r2, =500
            ldr  r1, =TxQueue
            ldr  r0, =txq
            bl   InitQueue
            ldr  r1, =RxQueue
            ldr  r0, =rxq 
            bl   InitQueue
            bl   init_uart0
            pop  {r0-r2, pc}
            //endp

tpm0_handler:
            bx lr

//>>>>>   end subroutine code <<<<<
.align
//**********************************************************************
//Constants
.data
//>>>>> begin constants here <<<<<
.global dac0_table_0
dac0_table_0:
dac0_table:
            .word (((dac0_steps - 1) * 1) / (servo_positions * 2))
            .word (((dac0_steps - 1) * 3) / (servo_positions * 2))
            .word (((dac0_steps - 1) * 5) / (servo_positions * 2))
            .word (((dac0_steps - 1) * 7) / (servo_positions * 2))
            .word (((dac0_steps - 1) * 9) / (servo_positions * 2))

.global pwm_duty_table_0
pwm_duty_table_0:
pwm_duty_table:
            .word pwm_2ms
            .word ((3*(pwm_2ms-pwm_1ms)/4) + pwm_1ms)
            .word (((pwm_2ms-pwm_1ms)/2) + pwm_1ms)
            .word (((pwm_2ms-pwm_1ms)/4) + pwm_1ms)
            .word pwm_1ms

//>>>>>   end constants here <<<<<
//**********************************************************************
//Variables
//>>>>> begin variables here <<<<<
.bss
.global TxQueue
TxQueue: .space 18
.align
.global RxQueue
RxQueue: .space 18
.align
txq: .space 500
rxq: .space 500
.align
//>>>>>   end variables here <<<<<
.section "vtab", "a"
//ARM core vectors
 .long stack_top          //00:end of stack
 .long Startup            //01:reset vector
 .long Dummy_Handler      //02:NMI
 .long HardFault_LCD      //03:hard fault
 .long Dummy_Handler      //04:(reserved)
 .long Dummy_Handler      //05:(reserved)
 .long Dummy_Handler      //06:(reserved)
 .long Dummy_Handler      //07:(reserved)
 .long Dummy_Handler      //08:(reserved)
 .long Dummy_Handler      //09:(reserved)
 .long Dummy_Handler      //10:(reserved)
 .long Dummy_Handler      //11:SVCall (supervisor call)
 .long Dummy_Handler      //12:(reserved)
 .long Dummy_Handler      //13:(reserved)
 .long Dummy_Handler      //14:PendableSrvReq (pendable request 
                                      //   for system service)
 .long Dummy_Handler      //15:SysTick (system tick timer)
 .long Dummy_Handler      //16:DMA channel 0 xfer complete/error
 .long Dummy_Handler      //17:DMA channel 1 xfer complete/error
 .long Dummy_Handler      //18:DMA channel 2 xfer complete/error
 .long Dummy_Handler      //19:DMA channel 3 xfer complete/error
 .long Dummy_Handler      //20:(reserved)
 .long Dummy_Handler      //21:command complete// read collision
 .long Dummy_Handler      //22:low-voltage detect//
                                      //   low-voltage warning
 .long Dummy_Handler      //23:low leakage wakeup
 .long Dummy_Handler      //24:I2C0
 .long Dummy_Handler      //25:I2C1
 .long Dummy_Handler      //26:SPI0 (all IRQ sources)
 .long Dummy_Handler      //27:SPI1 (all IRQ sources)
 .long uart0_isr          //28:UART0 (status// error)
 .long Dummy_Handler      //29:UART1 (status// error)
 .long Dummy_Handler      //30:UART2 (status// error)
 .long Dummy_Handler      //31:ADC0
 .long Dummy_Handler      //32:CMP0
 .long tpm0_handler       //33:TPM0
 .long Dummy_Handler      //34:TPM1
 .long Dummy_Handler      //35:TPM2
 .long Dummy_Handler      //36:RTC (alarm)
 .long Dummy_Handler      //37:RTC (seconds)
 .long Dummy_Handler      //38:PIT (all IRQ sources)
 .long Dummy_Handler      //39:I2S0
 .long Dummy_Handler      //40:USB0
 .long Dummy_Handler      //41:DAC0
 .long Dummy_Handler      //42:TSI0
 .long Dummy_Handler      //43:MCG
 .long Dummy_Handler      //44:LPTMR0
 .long Dummy_Handler      //45:Segment LCD
 .long Dummy_Handler      //46:PORTA pin detect
 .long Dummy_Handler      //47:PORTC and PORTD pin detect

.end
