#ifndef __MKL46Z4_H__
#define __MKL46Z4_H__
#include "MKL46Z4_Constants.h"
//structures and unions for register map definitions
typedef int uint32_t;
typedef char uint8_t;
typedef short int uint16_t;
typedef union {
    char DAT;
    char DATL:4;
    char DATH:4;
} DAC_DAT;

typedef struct {
    DAC_DAT DAT[2];
    uint8_t SR, C0,C1,C2;
} DAC_S;


typedef struct    {
    uint32_t SC1[2];
    uint32_t CFG1, CFG2;
    uint32_t R[2];
    uint32_t CV1, CV2;
    uint32_t SC2, SC3;
    uint32_t OFS;
    uint32_t PG, MG;
    uint32_t CLPD, CLPS, CLP4, CLP3, CLP2, CLP1, CLP0;
    uint32_t CLMD, CLMS, CLM4, CLM3, CLM2, CLM1, CLM0;
} ADC_S;


typedef struct  {
    uint32_t SOPT1;
    uint32_t SOPT1CFG;
    uint32_t SOPT2, SOPT4, SOPT5, SOPT7;
    uint32_t SDID;
    uint32_t SCGC4, SCGC5, SCGC6, SCGC7;
    uint32_t CLKDIV1;
    uint32_t FCFG1, FCFG2;
    uint32_t UIDMH;
    uint32_t UIDML;
    uint32_t COPC;
    uint32_t SRVCOP;
} SIM_S;

typedef struct  {
    uint32_t CnSC;
    uint32_t CnV;
} TPM_CHAN;

typedef struct  {
    uint32_t SC;
    uint32_t CNT;
    uint32_t MOD;
    TPM_CHAN CONTROLS[5];
    uint32_t STATUS;
    uint32_t CONF;
} TPM_S;


typedef struct  {
    uint32_t PCR[32];
    uint32_t GPCLR;
    uint32_t GPCHR;
    uint32_t ISFR;
} PORT;

DAC_S *DAC0 = 0x4003F000;

ADC_S *ADC0 = 0x4003B000;

SIM_S *SIM = 0x40047000;

TPM_S *TPM0 = 0x40038000;
TPM_S *TPM1 = 0x40039000;
TPM_S *TPM2 = 0x4003A000;

PORT *PORTE = 0x4004D000;

#endif
