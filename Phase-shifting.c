//###########################################################################
// Multi pulse test firmware
// Yunlei Jiang
// Buck-mode: high-side switch is activated
// Boost-mode: low-side switchi is activated
#include "F28x_Project.h"

//
// Defines
//
#define EPWM1_MAX_DB   10
#define EPWM2_MAX_DB   10
#define EPWM3_MAX_DB   10
#define EPWM1_MIN_DB   10
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   10
#define DB_UP          1
#define DB_DOWN        0

#define PRD_No 1000 // 200 MHz/PRD_No/2
#define PRD_No_ePWM4 5000 // 200 MHz/PRD_No/2, this value should change vs PRD_No,
//e.g., PRD_No = 200, it equals 1600, PRD_No = 100, then it equals 800
#define DB_LHa   13 // before S1H on, 1--5 ns,30--40--32// NTB: 24
#define DB_HLa   8 // before S1L on, 1--5 ns---32 // NTH: 32

#define DB_LHb   13 // before S1H on, 1--5 ns, 40 for asynchronous mode
#define DB_HLb   8 // before S1L on, 1--5 ns, 18 tested in syn on 101219


#define CompA_b 490 // CompA_b = PRD_No - (Phi_Hoff - Phi_Loff)/5ns; e.g., CompA_b = 500 - (100ns-75ns)/5ns = 495;

#define Pulse_No 50 // number of cycles in one cycle; 1s // NTB:34
#define OneCyclePulse_No 50
//

// Globals
//
Uint32 EPwm1TimerIntCount;
Uint32 EPwm2TimerIntCount;
Uint32 EPwm3TimerIntCount;
Uint32 EPwm4TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint16 EPwm2_DB_Direction;
Uint16 EPwm3_DB_Direction;
Uint16 EPwm4_DB_Direction;
Uint16 PS = 16;   // 85ns/5 = 17 - 1

//
// Function Prototypes
//
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);

__interrupt void epwm1_isr(void);

//
// Main
//
void main(void)
{
//
     InitSysCtrl();
//
//    InitGpio();

//
// enable PWM1, PWM2 and PWM3
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR2.bit.EPWM4=1;

//
// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2837xD_EPwm.c file
//
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;// added by me: 1--divided by 2
    EDIS; // very important, otherwise the code above may not work

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();
//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;
//
//
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr;
    //PieVectTable.EPWM3_INT = &epwm3_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

//
// Step 4. Initialize the Device Peripherals:
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

//
// Step 5. User specific code, enable interrupts:
// Initialize counters:
//
    EPwm1TimerIntCount = 0;
    EPwm2TimerIntCount = 0;
    EPwm3TimerIntCount = 0;
    EPwm4TimerIntCount = 0;

//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
    IER |= M_INT3;

//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//
    for(;;)
    {
        asm ("          NOP");
    }
}



__interrupt void epwm1_isr(void)
{
    EPwm1TimerIntCount++;


    if (EPwm1TimerIntCount < Pulse_No)
    {
        // Active Low PWMs - Setup Deadband
            //
            EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
            EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
            EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
            EPwm1Regs.DBRED.bit.DBRED = DB_LHa;
            EPwm1Regs.DBFED.bit.DBFED = DB_HLa;
            EPwm1_DB_Direction = DB_UP;


            // Active Low complementary PWMs - setup the deadband
            //
            EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
            EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
            EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
            EPwm2Regs.DBRED.bit.DBRED = DB_LHb;
            EPwm2Regs.DBFED.bit.DBFED = DB_HLb;
            EPwm2_DB_Direction = DB_UP;

    }
    else
    {
        // Active Low PWMs - Setup Deadband
            //
            EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
            EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
            EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
            EPwm1Regs.DBRED.bit.DBRED = DB_LHa;
            EPwm1Regs.DBFED.bit.DBFED = DB_HLa;
            EPwm1_DB_Direction = DB_UP;

        // Active Low complementary PWMs - setup the deadband
        //
        EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
        EPwm2Regs.DBRED.bit.DBRED = DB_LHb;
        EPwm2Regs.DBFED.bit.DBFED = DB_HLb;
        EPwm2_DB_Direction = DB_UP;
    }



    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = PRD_No;                     // Set timer period, 200MHz/100/2=1MHz, 1 us
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;   // Sync down-stream module

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //
    EPwm1Regs.CMPA.bit.CMPA = CompA_b;
    EPwm1Regs.CMPB.bit.CMPB = CompA_b;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CBD = AQ_CLEAR;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED.bit.DBRED = DB_LHa;
    EPwm1Regs.DBFED.bit.DBFED = DB_HLa;
    EPwm1_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = PRD_No;                       // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = PS;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow just to observe on
                                                   // the scope

    //
    // Setup compare
    //
    EPwm2Regs.CMPA.bit.CMPA = CompA_b;
    EPwm2Regs.CMPB.bit.CMPB = CompA_b+1;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR;

    //EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
    //EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = DB_LHb;
    EPwm2Regs.DBFED.bit.DBFED = DB_HLb;
    EPwm2_DB_Direction = DB_UP;

    //
    // Interrupt where we will modify the deadband
    //
    //EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    //EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
}

