/*********************************************************************/
/* <Your program description here>                                   */
/* Name:  <Your name here>                                           */
/* Date:  <Date completed>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <Your section here>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 3, 2017                                       */
/*********************************************************************/
#include "Exercise11.h"
#define print(x) PutStringSB(x, sizeof(x))
#define scan (x, s) GetStringSB(x, s)
#define isnum(x)    ((x >= '0') && (x <= '9'))
int main (void) {
  uint8_t scan_buf[80];
  __asm(".syntax unified\ncpsid i");
  /* Perform all device initialization here */
  /* Before unmasking interrupts            */
  init_rxtx();
  init_dac0();
  init_and_cal_adc0();
  /*init_tpm0();*/
  __asm("cpsie i");
  

  for (;;) { 
    //User input
    print("\xA\xDType a number from 1 to 5:");
    uint8_t servo_pos = hex2nib(getchar());
    PutNumUB(servo_pos);
    if(servo_pos < 1 || servo_pos > 5) continue;
    
    //DAC
    uint8_t dac_l = (uint8_t)(dac0_table [servo_pos - 1] & 0xff);
    uint8_t dac_h = (uint8_t)(dac0_table [servo_pos - 1] >> 8);
    DAC0->DAT[0].DATL = dac_l;
    DAC0->DAT[0].DATH = dac_h;
    //ADC
    ADC0->SC1[0] = ADC0_SC1_SGL_DAC0;
    while(!(ADC0->SC1[0] & ADC_COCO_MASK));
    uint16_t newval = ADC0->R[0];


    print("\xA\xD\tOriginal digital value:\t0x");
    PutNumHex(dac0_table[servo_pos - 1]);
    print("\xA\xD        New digital value:\t0x");
    PutNumHex(newval);
    print("\xA\xD");
  //  TPM0->CONTROLS[4].CnV = pwm_duty_table[((newval*5)/1024)];
  } 

} /* main */

void init_dac0()    {
    SIM->SCGC6           |= SIM_SCGC6_DAC0_MASK;
    SIM->SCGC5           |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[30]        = SET_PTE30_DAC0_OUT;     //ALT1
    DAC0->C1              = DAC_C1_BUFFER_DISABLED;
    DAC0->C0              = DAC_C0_ENABLE;
    DAC0->DAT[0].DATL     = DAC_DATL_0V;
    DAC0->DAT[0].DATH     = DAC_DATH_0V;

}


void init_and_cal_adc0()    {
    SIM->SCGC6  |= SIM_SCGC6_ADC0_MASK;
    ADC0->CFG1   = ADC0_CFG1_LP_LONG_SGL10_3MHZ;
    ADC0->CFG2   = ADC0_CFG2_CHAN_A_NORMAL_LONG;
    ADC0->SC2    = ADC0_SC2_SWTRIG_VDDA;

    do  {
        ADC0->SC3 = ADC0_SC3_CAL;
        while(ADC0->SC3 & ADC_CAL_MASK);    //no no no no no
    } while(ADC0->SC3 & ADC_CALF_MASK);  //that's a pretty nasty conditional
    
    ADC0->PG = (((uint16_t)ADC0->CLPS + (uint16_t)ADC0->CLP4
               + (uint16_t)ADC0->CLP3 + (uint16_t)ADC0->CLP2
               + (uint16_t)ADC0->CLP1 + (uint16_t)ADC0->CLP0)
               >> 1) | (uint16_t) 0x8000;
    ADC0->SC3 = ADC0_SC3_SINGLE; //just like me :^(
}

void init_tpm0()    {
    SIM->SCGC6             |= (1 << 24);//SIM_SCGC6_TPM0_MASK;
    SIM->SCGC5             |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[31]          = SET_PTE31_TPM0_CH4_OUT;
    SIM->SOPT2             &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2             |= SIM_SOPT2_TPM_MCGPLLCLK_DIV2;
    TPM0->CONF              = TPM_CONF_DEFAULT;
    TPM0->CNT               = TPM_CNT_INIT;
    TPM0->MOD               = TPM_MOD_PWM_PERIOD_20ms;
    TPM0->CONTROLS[4].CnSC  = TPM_CnSC_PWMH;
    TPM0->CONTROLS[4].CnV   = TPM_CnV_PWM_DUTY_2ms;
    TPM0->SC                = TPM_SC_CLK_DIV16;
}

uint32_t atoi(char *s)  {
  uint32_t res = 0;
  char *ptr = s;
  int  count;
  for(count = 0; isnum(*(ptr++)); count++);   //a Melton-level loop
  //for(int mul = 1; 
  return res;
}
