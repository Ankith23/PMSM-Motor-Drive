/* TRANSITION IS HAPPENNING SMOOTHLY AFTER 10Hz*/
#include "F28x_Project.h"
#include "math.h"
void InitTimer0(void);//Timer0 Initialization Function
void gpioInit(void);//GPIO Initialization Function
void SysControl(void);//System Clock Initialization Function
void pwm1(void);//EPWM1 Initialization Function
void pwm2(void);//EPWM2 Initialization Function
void pwm3(void);//EPWM3 Initialization Function
void pwm4(void);//EPWM4 Initialization Function
void PWM5(void);//EPWM5 Initialization Function
void PWM6(void);//EPWM6 Initialization Function
void equal(void);//Frequency Determination Function
void pwgen(void);//PWM Generation for EQEP1 Module Function
void QEPInit(void);//EQEP Initialization Function
void BLDCMOTOR();//BLDC Motor Run Logic
void NEWBLDCMOTOR();//PMSM Motor Run Logic
void Adca_init();//ADC Initialization Function

__interrupt void Timer0ISR(void);
__interrupt void pwm1ISR(void);
__interrupt void pwm2ISR(void);
__interrupt void pwm3ISR(void);
__interrupt void pwm4ISR(void);
__interrupt void pwm5ISR(void);
__interrupt void pwm6ISR(void);
__interrupt void UTOISR(void);
__interrupt void adcaISR(void);

long Pre_Value,Curr_Value,Timer_Value, Period_I, Period_A, Period_B,Period_S, Phase_B,Phase_S,latchvalue[400],H1,H2,H3,prd,a;
long int pos, pos2, pos3;
double fr, Timer_Period,D4,D5,D6;
long DC,New,Fre_Avg,Period4;
unsigned long New_Prd,CMP4,CMP5,CMP6;
long Ia[100],Ib[100],Ic[100];
int i,j,k,pmsm;
int PMSM, BLDC;
float Fre_Sum,FreArray[100];
int Fre_Average;
int y,fre,u,freq,frequ,frequency;

void main(void)
{
   SysControl();
       gpioInit();
   InitTimer0();
      pwm1();
      pwm2();
      pwm3();
      pwm4();
      PWM5();
      PWM6();
      Adca_init();
    QEPInit();

       EALLOW;
    //disable the global interrupt PG 94
    DINT;
    //enable the PIE module
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    //copy the address PIE vector table

    //Clear interrupt flag
    IFR = 0;
    //enable the interrupt in the CPU
    IER = 0xffff;
    //enable global interrupt
    EINT;
    EDIS;
    while(1){
          equal(); // FREQUENCY CALCULATION FUNCTION
          pwgen(); // EPWM PULSE GENERATION FOR THE EMULATION OF ENCODER GIVEN TO EQEP1 MODULE OF TMS320F28379D
{
          if((j+=1)==400) j=0;
              latchvalue[j] = EQep1Regs.QPOSSLAT; // Read the position/counter value into to a variable
             pos=((latchvalue[j]));//Phase B
             pos2 = pos + 120;     //Phase Y
             pos3 = pos + 240;     //Phase 
             }
            D6 = 0.5*(1+sin((pos3*3.14)/180)); // STORING THEE VALUES OF SINE WITH 240 DEGREE PHASE SHIFT IN D6
            D5 = 0.5*(1+sin((pos2*3.14)/180)); // STORING THEE VALUES OF SINE WITH 120 DEGREE PHASE SHIFT IN D5
            D4 = 0.5*(1+sin((pos*3.14)/180));  // STORING THEE VALUES OF SINE WITH 0 DEGREE PHASE SHIFT IN D4

            if((frequency < 1000.0))           // For Frequency less than 10Hz Run as BLDC Motor
             {
                PMSM = 1;
                BLDC=1;
                BLDCMOTOR();
             }
             else                              // For Frequency more than 10Hz Run as PMSM Motor
             {
                BLDC=0;
                PMSM = 0;
                NEWBLDCMOTOR();                //PMSM Motor Logic
             }
    }
}
void BLDCMOTOR(void)
{
    /**************************************************** EPWM PRESCALAR CONFIGURATION ****************************************************/
     EPwm4Regs.TBCTL.bit.CLKDIV=4;
     EPwm4Regs.TBCTL.bit.HSPCLKDIV=1;
     EPwm5Regs.TBCTL.bit.CLKDIV=4;
     EPwm5Regs.TBCTL.bit.HSPCLKDIV=1;
     EPwm6Regs.TBCTL.bit.CLKDIV=4;
     EPwm6Regs.TBCTL.bit.HSPCLKDIV=1;
     /**************************************************** EPWM PRESCALAR CONFIGURATION ****************************************************/

       New_Prd = 4000;
       EPwm4Regs.TBPRD= New_Prd;
       EPwm5Regs.TBPRD= New_Prd;
       EPwm6Regs.TBPRD= New_Prd;

       H3=GpioDataRegs.GPDDAT.bit.GPIO111;
       H2=GpioDataRegs.GPCDAT.bit.GPIO94;
       H1=GpioDataRegs.GPBDAT.bit.GPIO52;

   if((GpioDataRegs.GPBDAT.bit.GPIO52==1)&&(GpioDataRegs.GPCDAT.bit.GPIO94==0)&&(GpioDataRegs.GPDDAT.bit.GPIO111==1))
   {
       EPwm4Regs.CMPA.bit.CMPA=4000;
       EPwm5Regs.CMPB.bit.CMPB=0;//on
       EPwm4Regs.CMPB.bit.CMPB=4010;
       EPwm5Regs.CMPA.bit.CMPA=0;
       EPwm6Regs.CMPA.bit.CMPA=0;
       EPwm6Regs.CMPB.bit.CMPB=4010;

   }
   else if((GpioDataRegs.GPBDAT.bit.GPIO52==1)&&(GpioDataRegs.GPCDAT.bit.GPIO94==0)&&(GpioDataRegs.GPDDAT.bit.GPIO111==0))
   {
       EPwm4Regs.CMPA.bit.CMPA=4000;
       EPwm5Regs.CMPB.bit.CMPB=4010;
       EPwm4Regs.CMPB.bit.CMPB=4010;
       EPwm5Regs.CMPA.bit.CMPA=0;
       EPwm6Regs.CMPA.bit.CMPA=0;
       EPwm6Regs.CMPB.bit.CMPB=0;//on
   }
   else if((GpioDataRegs.GPBDAT.bit.GPIO52==1)&&(GpioDataRegs.GPCDAT.bit.GPIO94==1)&&(GpioDataRegs.GPDDAT.bit.GPIO111==0))
   {
       EPwm4Regs.CMPA.bit.CMPA=0;
       EPwm5Regs.CMPB.bit.CMPB=4010;
       EPwm4Regs.CMPB.bit.CMPB=4010;
       EPwm5Regs.CMPA.bit.CMPA=4010;
       EPwm6Regs.CMPA.bit.CMPA=0;
       EPwm6Regs.CMPB.bit.CMPB=0;//on
   }
   else if((GpioDataRegs.GPBDAT.bit.GPIO52==0)&&(GpioDataRegs.GPCDAT.bit.GPIO94==1)&&(GpioDataRegs.GPDDAT.bit.GPIO111==0))
   {
       EPwm4Regs.CMPA.bit.CMPA=0;
       EPwm5Regs.CMPB.bit.CMPB=4010;
       EPwm4Regs.CMPB.bit.CMPB=0;//on
          EPwm5Regs.CMPA.bit.CMPA=4000;
          EPwm6Regs.CMPA.bit.CMPA=0;
          EPwm6Regs.CMPB.bit.CMPB=4010;
      }
      else if((GpioDataRegs.GPBDAT.bit.GPIO52==0)&&(GpioDataRegs.GPCDAT.bit.GPIO94==1)&&(GpioDataRegs.GPDDAT.bit.GPIO111==1))
      {
          EPwm4Regs.CMPA.bit.CMPA=0;
          EPwm5Regs.CMPB.bit.CMPB=4010;
          EPwm4Regs.CMPB.bit.CMPB=0;//on
          EPwm5Regs.CMPA.bit.CMPA=0;
          EPwm6Regs.CMPA.bit.CMPA=4000;
          EPwm6Regs.CMPB.bit.CMPB=4010;
      }
      else if((GpioDataRegs.GPBDAT.bit.GPIO52==0)&&(GpioDataRegs.GPCDAT.bit.GPIO94==0)&&(GpioDataRegs.GPDDAT.bit.GPIO111==1))
      {
          EPwm4Regs.CMPA.bit.CMPA=0;
          EPwm5Regs.CMPB.bit.CMPB=0;//on
          EPwm4Regs.CMPB.bit.CMPB=4010;
          EPwm5Regs.CMPA.bit.CMPA=0;
          EPwm6Regs.CMPA.bit.CMPA=4000;
          EPwm6Regs.CMPB.bit.CMPB=4010;
      }
     }

void NEWBLDCMOTOR(void){

    /**************************************************** EPWM PRESCALAR CONFIGURATION ****************************************************/
     EPwm4Regs.TBCTL.bit.CLKDIV=4;
     EPwm4Regs.TBCTL.bit.HSPCLKDIV=1;
     EPwm5Regs.TBCTL.bit.CLKDIV=4;
     EPwm5Regs.TBCTL.bit.HSPCLKDIV=1;
     EPwm6Regs.TBCTL.bit.CLKDIV=4;
     EPwm6Regs.TBCTL.bit.HSPCLKDIV=1;
     /**************************************************** EPWM PRESCALAR CONFIGURATION ****************************************************/
     Period4 = Period_A;       // SWITCHING FREQUENCY IS EQUAL TO FREQUENCY OF EPWM2A (EQEP1A)
     New_Prd = Period4;
     EPwm4Regs.TBPRD=New_Prd;
     EPwm5Regs.TBPRD=New_Prd;
     EPwm6Regs.TBPRD=New_Prd;

     EPwm6Regs.CMPA.bit.CMPA=(0.5*New_Prd)*(1+sin((pos3*3.14)/180));
     EPwm5Regs.CMPA.bit.CMPA=((0.5*New_Prd)*(1+sin(((pos2*3.14)/180))));
     EPwm4Regs.CMPA.bit.CMPA=((0.5*New_Prd)*(1+sin(((pos*3.14)/180))));

     EPwm6Regs.CMPB.bit.CMPB=(0.5*New_Prd)*(1+sin((pos3*3.14)/180));
     EPwm5Regs.CMPB.bit.CMPB=((0.5*New_Prd)*(1+sin(((pos2*3.14)/180))));
     EPwm4Regs.CMPB.bit.CMPB=((0.5*New_Prd)*(1+sin(((pos*3.14)/180))));
}


void InitTimer0(void)
{
       EALLOW;
       CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
       CpuTimer0Regs.TCR.bit.TSS = 1;
       CpuTimer0Regs.PRD.all = 3000000;
       CpuTimer0Regs.TPR.all= 128;//162 set the prescale to system clock clock frequency to get the output of 1HHz
       CpuTimer0Regs.TCR.bit.FREE = 0 ;
       CpuTimer0Regs.TCR.bit.TIE = 1;
       CpuTimer0Regs.TCR.bit.TRB = 1;
       CpuTimer0Regs.TCR.bit.TIF = 1;
       CpuTimer0Regs.TCR.bit.TSS = 0;
       EDIS;
}


void gpioInit(void)
{
        EALLOW;
        GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 00;
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 01;

        GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 00;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 01;
        GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 00;
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 01;

        GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 00;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 01;
        GpioCtrlRegs.GPADIR.bit.GPIO4=01;//direction as output

        GpioCtrlRegs.GPCGMUX2.bit.GPIO95 = 00;
        GpioCtrlRegs.GPCMUX2.bit.GPIO95 = 00;
        GpioCtrlRegs.GPCDIR.bit.GPIO95 = 0;

        GpioCtrlRegs.GPAGMUX1.bit.GPIO6=00;//0 for gmux & 1 for mux
        GpioCtrlRegs.GPAMUX1.bit.GPIO6=01;// by muxed pins we have to select pwm value
        GpioCtrlRegs.GPAQSEL1.bit.GPIO6=02;

         GpioCtrlRegs.GPAGMUX1.bit.GPIO8=00;//0 for gmux & 1 for mux
         GpioCtrlRegs.GPAMUX1.bit.GPIO8=01;// by muxed pins we have to select pwm value
         GpioCtrlRegs.GPAQSEL1.bit.GPIO8=02;
         GpioCtrlRegs.GPADIR.bit.GPIO8=01;//direction as output
         GpioCtrlRegs.GPAPUD.bit.GPIO8=01;

            GpioCtrlRegs.GPAGMUX1.bit.GPIO10=00;//0 for gmux & 1 for mux
            GpioCtrlRegs.GPAMUX1.bit.GPIO10=01;// by muxed pins we have to select pwm value
            GpioCtrlRegs.GPAQSEL1.bit.GPIO10=02;
           GpioCtrlRegs.GPADIR.bit.GPIO10=01;//direction as output
           GpioCtrlRegs.GPAPUD.bit.GPIO10=01;

           GpioCtrlRegs.GPAGMUX1.bit.GPIO7=00;//0 for gmux & 1 for mux
           GpioCtrlRegs.GPAMUX1.bit.GPIO7=01;// by muxed pins we have to select pwm value
           GpioCtrlRegs.GPAQSEL1.bit.GPIO7=02;

           GpioCtrlRegs.GPAGMUX1.bit.GPIO9=00;//0 for gmux & 1 for mux
            GpioCtrlRegs.GPAMUX1.bit.GPIO9=01;// by muxed pins we have to select pwm value
            GpioCtrlRegs.GPAQSEL1.bit.GPIO9=02;
            GpioCtrlRegs.GPADIR.bit.GPIO9=01;//direction as output
            GpioCtrlRegs.GPAPUD.bit.GPIO9=01;

       GpioCtrlRegs.GPAGMUX1.bit.GPIO11=00;//0 for gmux & 1 for mux
       GpioCtrlRegs.GPAMUX1.bit.GPIO11=01;// by muxed pins we have to select pwm value
       GpioCtrlRegs.GPAQSEL1.bit.GPIO11=02;
        GpioCtrlRegs.GPADIR.bit.GPIO11=01;//direction as output
        GpioCtrlRegs.GPAPUD.bit.GPIO11=01;

/************* H1, H2, H3 **************/

        GpioCtrlRegs.GPDGMUX1.bit.GPIO111=00;//0 for gmux & 1 for mux
        GpioCtrlRegs.GPDMUX1.bit.GPIO111=00;// by muxed pins we have to select pwm value
         GpioCtrlRegs.GPDDIR.bit.GPIO111=00;//direction as output
         GpioCtrlRegs.GPDPUD.bit.GPIO111=01;
         GpioDataRegs.GPDCLEAR.bit.GPIO111=01;

         GpioCtrlRegs.GPBGMUX2.bit.GPIO52=00;//0 for gmux & 1 for mux
         GpioCtrlRegs.GPBMUX2.bit.GPIO52=00;// by muxed pins we have to select pwm value
          GpioCtrlRegs.GPBDIR.bit.GPIO52=00;//direction as output
          GpioCtrlRegs.GPBPUD.bit.GPIO52=01;
          GpioCtrlRegs.GPBQSEL2.bit.GPIO52=2;
          GpioDataRegs.GPBCLEAR.bit.GPIO52=01;

          GpioCtrlRegs.GPCGMUX2.bit.GPIO94=00;//0 for gmux & 1 for mux
          GpioCtrlRegs.GPCMUX2.bit.GPIO94=00;// by muxed pins we have to select pwm value
           GpioCtrlRegs.GPCDIR.bit.GPIO94=00;//direction as output
           GpioCtrlRegs.GPCPUD.bit.GPIO94=01;
           GpioDataRegs.GPCCLEAR.bit.GPIO94=01;


       EDIS;

}
void SysControl(void)
{
    EALLOW;
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL=00;//PRIMARY OSCILLATOR SELECT
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN=0;//BYPASS PLL
    for(i=0;i<120;i++);//wait for 120 oscialltor cycle
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV=0;//binary num
    for(i=0;i<5;i++)
    {
     ClkCfgRegs.SYSPLLCTL1.bit.PLLEN=0;//lock PLL
     ClkCfgRegs.SYSPLLMULT.bit.IMULT=16;//162Mhx PLL raw clock
     ClkCfgRegs.SYSPLLMULT.bit.FMULT=0;//0.25 fractional mult
     while(ClkCfgRegs.SYSPLLSTS.bit.LOCKS !=1);
    }
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV=1;//desired+1
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN=1;//switch to PLL
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV=0;//divides by 1, 162Mhz clock
   // ClkCfgRegs.LOSPCP.bit.LSPCLKDIV=1;//80Mhz LSP clock
    EDIS;
}

void pwm1(void){
    EALLOW;
          ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;   //EPWMCLK=PLLSYSCLK/2
          CpuSysRegs.PCLKCR2.bit.EPWM1=1;             // CLK ENABLE FOR EPWM1
          CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
          EPwm1Regs.TBCTL.bit.CLKDIV = 4;          //Divides the EPWMCLK by 8 (3 bits - binary)
           EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;       //Divides the EPWMCLK by 2 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

           EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // 0 for up counter
           EPwm1Regs.TBCTR = 0x0000;              // Clear counter

           EPwm1Regs.CMPB.bit.CMPB = 500;          // EPWM compare value

           EPwm1Regs.AQCTLA.bit.CAU = 1;           // Action qualifier
           EPwm1Regs.AQCTLA.bit.ZRO = 2;           // Action qualifier

           EPwm1Regs.ETSEL.bit.INTEN = 1;          // EPWM 1 interrupt enable
           EPwm1Regs.ETSEL.bit.INTSEL = 1;             // Interrupt when TBCTR = zero
           EPwm1Regs.ETPS.bit.INTPRD = 1;          // Interrupt at period
           EPwm1Regs.ETCLR.bit.INT = 1;            // Clear interrupt
   // Phase is 0
           EPwm1Regs.TBCTL.bit.PRDLD = 0;
           EDIS;
}

void pwm2(void){
    EALLOW;
      ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;   //EPWMCLK=PLLSYSCLK/2
      CpuSysRegs.PCLKCR2.bit.EPWM2=1;             // CLK ENABLE FOR EPWM2
      CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
      EPwm2Regs.TBCTL.bit.CLKDIV = 4;          //Divides the EPWMCLK by 1 (3 bits - binary)
       EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;       //Divides the EPWMCLK by 1 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

       EPwm2Regs.TBCTL.bit.CTRMODE = 0;        // 0 for up counter
       EPwm2Regs.TBCTR = 0x0000;              // Clear counter

       EPwm2Regs.AQCTLA.bit.CAU = 1;           // Action qualifier
       EPwm2Regs.AQCTLA.bit.ZRO = 2;           // Action qualifierm
       EPwm2Regs.AQCTLB.bit.CBU = 1;           // Action qualifier
       EPwm2Regs.AQCTLB.bit.ZRO = 2;           // Action qualifier

       EPwm2Regs.ETSEL.bit.INTEN = 1;          // EPWM 1 interrupt enable
       EPwm2Regs.ETSEL.bit.INTSEL = 1;             // Interrupt when TBCTR = zero
       EPwm2Regs.ETPS.bit.INTPRD = 1;          // Interrupt at period
       EPwm2Regs.ETCLR.bit.INT = 1;            // Clear interrupt

       EPwm2Regs.TBCTL.bit.PRDLD = 0;
       EDIS;
}

void pwm3(void){
    EALLOW;
      ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;   //EPWMCLK=PLLSYSCLK/2
      CpuSysRegs.PCLKCR2.bit.EPWM3=1;               // CLK ENABLE FOR EPWM3
      CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
      EPwm3Regs.TBCTL.bit.CLKDIV = 4;               //Divides the EPWMCLK by 1 (3 bits - binary)
       EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;           //Divides the EPWMCLK by 1 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

       EPwm3Regs.TBCTL.bit.CTRMODE = 0;             // 0 for up counter
       EPwm3Regs.TBCTR = 0x0000;                    // Clear counter

       EPwm3Regs.AQCTLA.bit.CAU = 1;                // Action qualifier set to clear at when TBCTR reaches CMPA
       EPwm3Regs.AQCTLA.bit.ZRO = 2;                // Action qualifier set when TBCTR reaches ZERO


       EPwm3Regs.ETSEL.bit.INTEN = 1;               // EPWM 1 interrupt enable
       EPwm3Regs.ETSEL.bit.INTSEL = 1;              // Interrupt when TBCTR = zero
       EPwm3Regs.ETPS.bit.INTPRD = 1;               // Interrupt at period
       EPwm3Regs.ETCLR.bit.INT = 1;                 // Clear interrupt
       EPwm3Regs.TBCTL.bit.PRDLD = 0;
       EDIS;
}
void pwm4(void)
{
    EALLOW;
         //EPWM2
         ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;   //EPWMCLK=PLLSYSCLK/2
         CpuSysRegs.PCLKCR2.bit.EPWM4=1;             // CLK ENABLE FOR EPWM4
         CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;
         EPwm4Regs.TBCTL.bit.CLKDIV=4;               //Divides the EPWMCLK by 8 (3 bits - binary)
         EPwm4Regs.TBCTL.bit.HSPCLKDIV=1;            //Divides the EPWMCLK by 1 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
         EPwm4Regs.TBCTR=0x0000;                     // Clear counter
         EPwm4Regs.TBCTL.bit.CTRMODE=0;              // 0 for up counter

         EPwm4Regs.AQCTLA.bit.CAU=1;                 // Action qualifier set to clear at when TBCTR reaches CMPA
         EPwm4Regs.AQCTLA.bit.ZRO=2;                 // Action qualifier set when TBCTR reaches ZERO
         EPwm4Regs.AQCTLB.bit.CBU=2;                 // Action qualifier set when TBCTR reaches CMPB
         EPwm4Regs.AQCTLB.bit.ZRO=1;                 // Action qualifier set to clear at when TBCTR reaches ZERO

         EPwm4Regs.ETSEL.bit.INTSEL= 2;              // Enable event time-base counter equal to period
         EPwm4Regs.ETSEL.bit.INTEN=1;                // EPWM 1 interrupt enable
         EPwm4Regs.ETCLR.bit.INT=1;                  // Clear interrupt
         EPwm4Regs.ETPS.bit.INTPRD=1;                // Interrupt at period
         EDIS;

    }
void PWM5(void)
{
    EALLOW;
         //EPWM2
         ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;   //EPWMCLK=PLLSYSCLK/2
         CpuSysRegs.PCLKCR2.bit.EPWM5=1;             // CLK ENABLE FOR EPWM5
         CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;
         EPwm5Regs.TBCTL.bit.CLKDIV=4;          //Divides the EPWMCLK by 8 (3 bits - binary)
         EPwm5Regs.TBCTL.bit.HSPCLKDIV=1;       //Divides the EPWMCLK by 1 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
         EPwm5Regs.TBCTR=0x0000;                // Clear counter
         EPwm5Regs.TBCTL.bit.CTRMODE=0;         // 0 for up counter

         EPwm5Regs.AQCTLA.bit.CAU=1;           // Action qualifier set to clear at when TBCTR reaches CMPA
         EPwm5Regs.AQCTLA.bit.ZRO=2;           // Action qualifier set when TBCTR reaches ZERO
         EPwm5Regs.AQCTLB.bit.CBU=2;           // Action qualifier set when TBCTR reaches CMPB
         EPwm5Regs.AQCTLB.bit.ZRO=1;           // Action qualifier set to clear at when TBCTR reaches ZERO

         EPwm5Regs.ETSEL.bit.INTSEL= 2;        // Enable event time-base counter equal to period
         EPwm5Regs.ETSEL.bit.INTEN=1;          // EPWM 1 interrupt enable
         EPwm5Regs.ETCLR.bit.INT=1;            // Clear interrupt
         EPwm5Regs.ETPS.bit.INTPRD=1;          // Interrupt at period
         EDIS;
}
void PWM6(void)
{
         EALLOW; //EPWM3
         ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;   //EPWMCLK=PLLSYSCLK/2
         CpuSysRegs.PCLKCR2.bit.EPWM6=1;             // CLK ENABLE FOR EPWM6
         CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;
         EPwm6Regs.TBCTL.bit.CLKDIV=4;          //Divides the EPWMCLK by 8 (3 bits - binary)
         EPwm6Regs.TBCTL.bit.HSPCLKDIV=1;       //Divides the EPWMCLK by 1 : TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
         EPwm6Regs.TBCTR=0x0000;                // Clear counter
         EPwm6Regs.TBCTL.bit.CTRMODE=0;         // 0 for up counter

         EPwm6Regs.AQCTLA.bit.CAU=1;           // Action qualifier set to clear at when TBCTR reaches CMPA
         EPwm6Regs.AQCTLA.bit.ZRO=2;           // Action qualifier set when TBCTR reaches ZERO
         EPwm6Regs.AQCTLB.bit.CBU=2;           // Action qualifier set when TBCTR reaches CMPB
         EPwm6Regs.AQCTLB.bit.ZRO=1;           // Action qualifier set to clear at when TBCTR reaches ZERO

         EPwm6Regs.ETSEL.bit.INTSEL= 2;        // Enable event time-base counter equal to period
         EPwm6Regs.ETSEL.bit.INTEN=1;          // EPWM 1 interrupt enable
         EPwm6Regs.ETCLR.bit.INT=1;
         EPwm6Regs.ETPS.bit.INTPRD=1;          // Interrupt at period

         EDIS;
}

/**************************************************** FREQUENCY DETERMINATION FUNCTION ****************************************************/

void equal(void)
{
    {
     Curr_Value = GpioDataRegs.GPBDAT.bit.GPIO52;                        // Value of H1 is stored in Curr_Value
     if((Curr_Value == 1)&(Pre_Value == 0))                              // When rising edge is detected start timer
     {
         CpuTimer0Regs.TCR.bit.TSS = 0;                                  // Timer0 Start
         if((Curr_Value == 1)&(Pre_Value == 0))                          // When another rising edge is detected read the timer value
         {
             Timer_Value = CpuTimer0Regs.TIM.all;                        // Timer value is read in Timer_Value Variable
             fre = (125637931/(((CpuTimer0Regs.PRD.all)-Timer_Value)));  // Calculation of frequency
             CpuTimer0Regs.TCR.bit.TRB = 1;                              // Timer is Reloaded after the latched TIM register value is read
         }

     }
     Pre_Value = Curr_Value;
         }

      /**************************************************** AVERAGE FREQUENCY ****************************************************/
      
      FreArray[y] = fre;
      
      y++;
      
      if(y >= 100)
      {
          y = 0;
          Fre_Sum = 0;
      }
      
      /**************************************************** AVERAGE CALCULATION ****************************************************/
      
      for(i = 0; i < 100; i++)
      {
          Fre_Sum += FreArray[i];
      }
      
      Fre_Average = Fre_Sum / 100.0;
      
      frequency = Fre_Average;
}

void pwgen(void)
{
    Period_I = (500000000/frequency);                       //(80000000/(4*4)) = 5000000*100 = 500000000
     EPwm1Regs.TBPRD = (Period_I/30);                       // using as strobe for 12 degree resolution
     EPwm1Regs.CMPA.bit.CMPA = (Period_I/60);              // EPWM compare value


     Period_A = Period_I/90;                               // Generation of 180 EQEP1A Pulses
     EPwm2Regs.TBPRD = Period_A;                            // EPWM 2
     EPwm2Regs.CMPA.bit.CMPA = (Period_A*0.5);              // EPWM compare value
     EPwm2Regs.TBCTL.bit.PHSEN = 1;                         // enable phase loading
     EPwm2Regs.TBPHS.bit.TBPHS = 0;                         // Phase is 0

     Period_B = Period_I/90;                               // Generation of 180 EQEP1B Pulses
     EPwm3Regs.TBPRD = Period_B;                            // EPWM 3
     EPwm3Regs.CMPA.bit.CMPA = ((Period_B*0.5));            // EPWM compare value

     Phase_B = (Period_B/4);                                // For 90 degree Phase shift between EQEP1A and EQEP1B
     EPwm3Regs.TBCTL.bit.PHSEN = 1;                         // enable phase loading
     EPwm3Regs.TBPHS.bit.TBPHS = Phase_B;                   // Phase is 0
}

void QEPInit(void)
{
        EALLOW;
        GpioCtrlRegs.GPAGMUX2.bit.GPIO20=0;                 //EQEP1A pin Mux for 1A
        GpioCtrlRegs.GPAGMUX2.bit.GPIO21=0;                 //EQEP1B pin Mux for1B
        GpioCtrlRegs.GPAGMUX2.bit.GPIO22=0;                 //1S-Strobe
        GpioCtrlRegs.GPDGMUX1.bit.GPIO99=1;                 //1I-Index
        GpioCtrlRegs.GPAMUX2.bit.GPIO20=1;
        GpioCtrlRegs.GPAMUX2.bit.GPIO21=1;
        GpioCtrlRegs.GPAMUX2.bit.GPIO22=1;
        GpioCtrlRegs.GPDMUX1.bit.GPIO99=1;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO20=2;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO21=2;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO22=2;
        GpioCtrlRegs.GPDQSEL1.bit.GPIO99=2;


        CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;                   // Enable clock to the QEP module

        EQep1Regs.QEPCTL.bit.FREE_SOFT=0;
        EQep1Regs.QEPCTL.bit.PCRM = 0;                      //03;Position counter reset on Unit time count event; 00:position counter reset on an index event1
        EQep1Regs.QEPCTL.bit.SEI=0;                         // Does nothing (Strobe event initialization of position counter

        EQep1Regs.QPOSINIT=0x0000000;                       // position counter initialization
        EQep1Regs.QEPCTL.bit.SWI=1;                         //Initialize the position counter with the value stored in QPOSINIT
        EQep1Regs.QEPCTL.bit.SEL=2;                         //Position counter is latched on the rising event of Strobe signal QEPS
        EQep1Regs.QEPCTL.bit.IEL= 0;                        //Disable Index latch
        EQep1Regs.QEPCTL.bit.QCLM=1;                        //Latch on unit time out; Positon couter, capture timer, capture period register

        EQep1Regs.QUPRD=80000;
        EQep1Regs.QPOSMAX=1440;                             // Maximum Value of position Counter #4000

        EQep1Regs.QEPCTL.bit.WDE=0;                         //Disable unit timer

        EQep1Regs.QDECCTL.bit.QSRC=00;                      //Quadrature count mode
        EQep1Regs.QDECCTL.bit.XCR=1;                        // Count the raising edges only
        EQep1Regs.QDECCTL.bit.QSP=0;                        // Dont invert the Strobe signal
        EQep1Regs.QCAPCTL.bit.CEN=1;                        //Enable the Capture moduele
        EQep1Regs.QCAPCTL.bit.UPPS= 0;                      //Unit position event pre-scalar
        EQep1Regs.QCAPCTL.bit.CCPS=7;                       //Capture timer clock pre-scalar
        EQep1Regs.QEPCTL.bit.QPEN=1;                        //Enable the eQEP position counter
        EQep1Regs.QEINT.bit.UTO = 0;                        //Unit time out interrupt disable
        EQep1Regs.QEINT.bit.PCO=1;
        EQep1Regs.QEINT.bit.PCU=1;
        EQep1Regs.QEINT.bit.PCR=1;
        EQep1Regs.QEINT.bit.PCE=1;
        EQep1Regs.QEINT.bit.QDC=1;
        EQep1Regs.QEINT.bit.QPE=1;
        EQep1Regs.QEINT.bit.SEL=1;
        EQep1Regs.QCLR.bit.UTO=1;                           //Clear UTO Flag
        EQep1Regs.QEPCTL.bit.UTE=0;                         // disable unit timer

        EQep1Regs.QCLR.bit.PCO=0;
        EQep1Regs.QCLR.bit.PCU=0;
        EDIS;
    }
void Adca_init(void)
{
    EALLOW;


               CpuSysRegs.PCLKCR13.bit.ADC_A=1;
               // set the ADC  clock frequency
               AdcaRegs.ADCCTL2.bit.PRESCALE=10;

                // select the resolution ,signal  modes
               AdcaRegs.ADCCTL2.bit.RESOLUTION=0;
               AdcaRegs.ADCCTL2.bit.SIGNALMODE=0;

                //VREFHI  and VREFLOW (reference is set based on the voltage available in the launch pad)
               //interrupts pulse position
               AdcaRegs.ADCCTL1.bit.INTPULSEPOS=1;

                //Enable the power to ADC
               AdcaRegs.ADCCTL1.bit.ADCPWDNZ=1;

                // SOC initialization
               AdcaRegs.ADCSOC2CTL.bit.CHSEL=2;      //ADCINA2 = pin 29
               AdcaRegs.ADCSOC2CTL.bit.ACQPS= 511;
               AdcaRegs.ADCSOC2CTL.bit.TRIGSEL=1;


               AdcaRegs.ADCSOC4CTL.bit.CHSEL=4;      //ADCINA4 = 69;
               AdcaRegs.ADCSOC4CTL.bit.ACQPS=511;
               AdcaRegs.ADCSOC4CTL.bit.TRIGSEL=1;


               AdcaRegs.ADCSOC3CTL.bit.CHSEL=3;      // ADCINA3 = pin 26
               AdcaRegs.ADCSOC3CTL.bit.ACQPS=511;
               AdcaRegs.ADCSOC3CTL.bit.TRIGSEL=1;



               AdcaRegs.ADCINTSOCSEL1.bit.SOC3=01;
               AdcaRegs.ADCINTSOCSEL1.bit.SOC4=01;
                AdcaRegs.ADCINTSOCSEL1.bit.SOC2=01;

               AdcaRegs.ADCINTSEL1N2.bit.INT1SEL=2; //SOC 2
               AdcaRegs.ADCINTSEL1N2.bit.INT1E=1;
               AdcaRegs.ADCINTSEL1N2.bit.INT2SEL=3; //SOC 2

                 AdcaRegs.ADCINTSEL1N2.bit.INT2E=1;
                 AdcaRegs.ADCINTSEL3N4.bit.INT3SEL=4; //SOC 2

                 AdcaRegs.ADCINTSEL3N4.bit.INT4E=1;
                // AdcaRegs.ADCINTSEL3N4.bit.INT3SEL=4; //SOC 2
                 AdcaRegs.ADCINTSEL3N4.bit.INT3E=1;


                AdcaRegs.ADCINTSEL1N2.bit.INT1CONT=1;


                //clear the interrupt flag
            //    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;
                EDIS;
}

__interrupt void Timer0ISR(void){

    CpuTimer0Regs.TCR.bit.TIF=1;
     PieCtrlRegs.PIEACK.bit.ACK1=1;

}
__interrupt void pwm1ISR(void){
    EPwm1Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void pwm2ISR(void){
    EPwm2Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void pwm3ISR(void){

    EPwm3Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void pwm4ISR(void){
    EPwm4Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void pwm5ISR(void){
    EPwm5Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}
__interrupt void pwm6ISR(void){
    EPwm6Regs.ETCLR.bit.INT = 1;            // Clear interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
__interrupt void adcaISR(void)
{
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=01;// clear the interrupt flag
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT2=01;// clear the interrupt flag
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT3=01;// clear the interrupt flag
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT4=01;// clear the interrupt flag

          PieCtrlRegs.PIEACK.bit.ACK1 =1;//clear the acknowledge bit
          CpuTimer0Regs.TCR.bit.TIF=1;  //as we not selected the timer0 for trigger
}
__interrupt void UTOISR(void)
{
    EQep1Regs.QCLR.bit.SEL=1;
    EQep1Regs.QCLR.bit.UTO=1;//Clear UTO Flag
    EQep1Regs.QCLR.bit.PCO=1;//Clear Overflow Flag
    EQep1Regs.QCLR.bit.PCU=1;//Clear Underflow Flag
    EQep1Regs.QCLR.bit.INT=1; // clear global interrupt bit
    EQep1Regs.QCLR.bit.QDC=1;
    EQep1Regs.QCLR.bit.PCE =1;
    EQep1Regs.QCLR.bit.PHE =1;
    EQep1Regs.QEPSTS.bit.UPEVNT=01;
    PieCtrlRegs.PIEACK.bit.ACK1=01;
}
