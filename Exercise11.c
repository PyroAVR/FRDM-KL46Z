#include "Exercise11.h"
#define print(x) PutStringSB(x, sizeof(x))
#define scan (x, s) GetStringSB(x, s)
#define isnum(x)    ((x >= '0') && (x <= '9'))
/*
 *Read a number from the console, then pass it through the DAC and ADC, reading
 *back what value the user initially typed via mapping onto the range [1,5].
 *The value is then used to select one duty value out of an array and move an RC
 *servo motor to positions evenly spaced along its range using the TPM module
 *to provide PWM signals routed through PORTE.
 *Repeat indefinitely.
 */
const uint32_t tpm_conf_default = TPM_CONF_DEFAULT;
const uint32_t set_pte31_tpm0_ch4_out = SET_PTE31_TPM0_CH4_OUT;
int main (void) {
    /*
     *Device initialization.  .syntax unified is necessary to tell GNU as how
     *to interpret the assembly syntax.  We choose to prefer the new Unified ARM
     *syntax, which is common to ARM and Thumb mode assembly.  Interrupts are
     *re-enabled after all initialization is complete.
     */
    __asm(".syntax unified\ncpsid i");
    init_rxtx();
    init_dac0();
    init_and_cal_adc0();
    init_tpm0();
    __asm("cpsie i");

    /*
     *Loop indefinitely, first prompting the console for a value on [1,5]
     */
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

        //Report back to the user and update the servo position
        print("\xA\xD\tOriginal digital value:\t0x");
        PutNumHex(dac0_table[servo_pos - 1]);
        print("\xA\xD        New digital value:\t0x");
        PutNumHex(newval);
        print("\xA\xD");
        TPM0->CONTROLS[4].CnV = pwm_duty_table[((newval*5)/1024)];
    } 

}

void init_dac0()    {
    SIM->SCGC6           |= SIM_SCGC6_DAC0_MASK;    //Enable DAC0 clock
    SIM->SCGC5           |= SIM_SCGC5_PORTE_MASK;   //Enable PORTE clock
    PORTE->PCR[30]        = SET_PTE30_DAC0_OUT;     //ALT1
    DAC0->C1              = DAC_C1_BUFFER_DISABLED; //Software-driven updates
    DAC0->C0              = DAC_C0_ENABLE;          //
    DAC0->DAT[0].DATL     = DAC_DATL_0V;            //Set output to 0V
    DAC0->DAT[0].DATH     = DAC_DATH_0V;            //

}


void init_and_cal_adc0()    {
    SIM->SCGC6  |= SIM_SCGC6_ADC0_MASK;             //Enable ADC0 clock
    ADC0->CFG1   = ADC0_CFG1_LP_LONG_SGL10_3MHZ;    //Set sample frequency
    ADC0->CFG2   = ADC0_CFG2_CHAN_A_NORMAL_LONG;    //
    ADC0->SC2    = ADC0_SC2_SWTRIG_VDDA;            //Poll for values

    do  {                                           //Calibrate
        ADC0->SC3 = ADC0_SC3_CAL;
        while(ADC0->SC3 & ADC_CAL_MASK);
    } while(ADC0->SC3 & ADC_CALF_MASK);             //Wait on calibration
    
    ADC0->PG = (((uint16_t)ADC0->CLPS + (uint16_t)ADC0->CLP4
               + (uint16_t)ADC0->CLP3 + (uint16_t)ADC0->CLP2
               + (uint16_t)ADC0->CLP1 + (uint16_t)ADC0->CLP0)
               >> 1) | (uint16_t) 0x8000;           //no idea fam
    ADC0->SC3 = ADC0_SC3_SINGLE;                    //Single value on trigger
}

void init_tpm0()    {
    SIM->SCGC6             |= SIM_SCGC6_TPM0_MASK;          //Enable TPM0 clock
    SIM->SCGC5             |= SIM_SCGC5_PORTE_MASK;         //Enable PORTE clock
    //PORTE->PCR[31]          = SET_PTE31_TPM0_CH4_OUT;       //Set pin mux
    SIM->SOPT2             &= ~SIM_SOPT2_TPMSRC_MASK;       //Set TPM clksrc
    SIM->SOPT2             |= SIM_SOPT2_TPM_MCGPLLCLK_DIV2; //
//    TPM0->CONF              = TPM_CONF_DEFAULT;
    TPM0->CNT               = TPM_CNT_INIT;                 //Set initial val.
    TPM0->MOD               = TPM_MOD_PWM_PERIOD_20ms;      //Set PWM per
    TPM0->CONTROLS[4].CnSC  = TPM_CnSC_PWMH;                //Set high time
    TPM0->CONTROLS[4].CnV   = TPM_CnV_PWM_DUTY_2ms;         //Set trigger start
    TPM0->SC                = TPM_SC_CLK_DIV16;             //Set TPM clkdiv
}
