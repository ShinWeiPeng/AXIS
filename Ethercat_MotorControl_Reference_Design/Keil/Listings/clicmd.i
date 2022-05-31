#line 1 "..\\Source\\Debug\\clicmd.c"




























 

 
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\NuMicro.h"
 





 



#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
 







 






































 







 
 
 



 



 
typedef enum IRQn
{
     
    NonMaskableInt_IRQn           = -14,       
    MemoryManagement_IRQn         = -12,       
    BusFault_IRQn                 = -11,       
    UsageFault_IRQn               = -10,       
    SVCall_IRQn                   = -5,        
    DebugMonitor_IRQn             = -4,        
    PendSV_IRQn                   = -2,        
    SysTick_IRQn                  = -1,        

     

    BOD_IRQn                      = 0,         
    IRC_IRQn                      = 1,         
    PWRWU_IRQn                    = 2,         
    RAMPE_IRQn                    = 3,         
    CKFAIL_IRQn                   = 4,         
    RTC_IRQn                      = 6,         
    TAMPER_IRQn                   = 7,         
    WDT_IRQn                      = 8,         
    WWDT_IRQn                     = 9,         
    EINT0_IRQn                    = 10,        
    EINT1_IRQn                    = 11,        
    EINT2_IRQn                    = 12,        
    EINT3_IRQn                    = 13,        
    EINT4_IRQn                    = 14,        
    EINT5_IRQn                    = 15,        
    GPA_IRQn                      = 16,        
    GPB_IRQn                      = 17,        
    GPC_IRQn                      = 18,        
    GPD_IRQn                      = 19,        
    GPE_IRQn                      = 20,        
    GPF_IRQn                      = 21,        
    QSPI0_IRQn                    = 22,        
    SPI0_IRQn                     = 23,        
    BRAKE0_IRQn                   = 24,        
    EPWM0P0_IRQn                  = 25,        
    EPWM0P1_IRQn                  = 26,        
    EPWM0P2_IRQn                  = 27,        
    BRAKE1_IRQn                   = 28,        
    EPWM1P0_IRQn                  = 29,        
    EPWM1P1_IRQn                  = 30,        
    EPWM1P2_IRQn                  = 31,        
    TMR0_IRQn                     = 32,        
    TMR1_IRQn                     = 33,        
    TMR2_IRQn                     = 34,        
    TMR3_IRQn                     = 35,        
    UART0_IRQn                    = 36,        
    UART1_IRQn                    = 37,        
    I2C0_IRQn                     = 38,        
    I2C1_IRQn                     = 39,        
    PDMA_IRQn                     = 40,        
    DAC_IRQn                      = 41,        
    EADC00_IRQn                   = 42,        
    EADC01_IRQn                   = 43,        
    ACMP01_IRQn                   = 44,        
    EADC02_IRQn                   = 46,        
    EADC03_IRQn                   = 47,        
    UART2_IRQn                    = 48,        
    UART3_IRQn                    = 49,        
    QSPI1_IRQn                    = 50,        
    SPI1_IRQn                     = 51,        
    SPI2_IRQn                     = 52,        
    USBD_IRQn                     = 53,        
    USBH_IRQn                     = 54,        
    USBOTG_IRQn                   = 55,        
    CAN0_IRQn                     = 56,        
    CAN1_IRQn                     = 57,        
    SC0_IRQn                      = 58,        
    SC1_IRQn                      = 59,        
    SC2_IRQn                      = 60,        
    SPI3_IRQn                     = 62,        
    EMAC_TX_IRQn                  = 66,        
    EMAC_RX_IRQn                  = 67,        
    SDH0_IRQn                     = 64,        
    USBD20_IRQn                   = 65,        
    I2S0_IRQn                     = 68,        
    OPA_IRQn                      = 70,        
    CRPT_IRQn                     = 71,        
    GPG_IRQn                      = 72,        
    EINT6_IRQn                    = 73,        
    UART4_IRQn                    = 74,        
    UART5_IRQn                    = 75,        
    USCI0_IRQn                    = 76,        
    USCI1_IRQn                    = 77,        
    BPWM0_IRQn                    = 78,        
    BPWM1_IRQn                    = 79,        
    SPIM_IRQn                     = 80,        
    CCAP_IRQn                     = 81,        
    I2C2_IRQn                     = 82,        
    QEI0_IRQn                     = 84,        
    QEI1_IRQn                     = 85,        
    ECAP0_IRQn                    = 86,        
    ECAP1_IRQn                    = 87,        
    GPH_IRQn                      = 88,        
    EINT7_IRQn                    = 89,        
    SDH1_IRQn                     = 90,        
    HSUSBH_IRQn                   = 92,        
    USBOTG20_IRQn                 = 93,        
    TRNG_IRQn                     = 101,       
    UART6_IRQn                    = 102,       
    UART7_IRQn                    = 103,       
    EADC10_IRQn                   = 104,       
    EADC11_IRQn                   = 105,       
    EADC12_IRQn                   = 106,       
    EADC13_IRQn                   = 107,       
    CAN2_IRQn                     = 108,       
}
IRQn_Type;






 

 
#line 197 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   


#line 1 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

















 




 



 

 













#line 120 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"



 
#line 135 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 209 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 1 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"











 


#line 54 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

#line 211 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

#line 212 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmSimd.h"

 
#line 88 "..\\..\\..\\Library\\CMSIS\\Include\\core_cmSimd.h"

 






#line 213 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"
















 
#line 256 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

 






 
#line 272 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 1550 "..\\..\\..\\Library\\CMSIS\\Include\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4UL)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4UL)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4UL)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4UL)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4UL)) ? (uint32_t)(4UL) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4UL)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4UL));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4UL)) ? (uint32_t)(4UL) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4UL)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4UL));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4UL) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 202 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"
 





 








#line 17 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"




 

#line 29 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"




extern uint32_t SystemCoreClock;      
extern uint32_t CyclesPerUs;          
extern uint32_t PllClock;             










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);







 
#line 203 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 204 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"




#pragma anon_unions


 
 
 

#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\sys_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile const  uint32_t PDID;                   
    volatile uint32_t RSTSTS;                 
    volatile uint32_t IPRST0;                 
    volatile uint32_t IPRST1;                 
    volatile uint32_t IPRST2;                 
     
    volatile const  uint32_t RESERVE0[1];
     
    volatile uint32_t BODCTL;                 
    volatile uint32_t IVSCTL;                 
     
    volatile const  uint32_t RESERVE1[1];
     
    volatile uint32_t PORCTL;                 
    volatile uint32_t VREFCTL;                
    volatile uint32_t USBPHY;                 
    volatile uint32_t GPA_MFPL;               
    volatile uint32_t GPA_MFPH;               
    volatile uint32_t GPB_MFPL;               
    volatile uint32_t GPB_MFPH;               
    volatile uint32_t GPC_MFPL;               
    volatile uint32_t GPC_MFPH;               
    volatile uint32_t GPD_MFPL;               
    volatile uint32_t GPD_MFPH;               
    volatile uint32_t GPE_MFPL;               
    volatile uint32_t GPE_MFPH;               
    volatile uint32_t GPF_MFPL;               
    volatile uint32_t GPF_MFPH;               
    volatile uint32_t GPG_MFPL;               
    volatile uint32_t GPG_MFPH;               
    volatile uint32_t GPH_MFPL;               
    volatile uint32_t GPH_MFPH;               
     
    volatile const  uint32_t RESERVE2[4];
     
    volatile uint32_t GPA_MFOS;               
    volatile uint32_t GPB_MFOS;               
    volatile uint32_t GPC_MFOS;               
    volatile uint32_t GPD_MFOS;               
    volatile uint32_t GPE_MFOS;               
    volatile uint32_t GPF_MFOS;               
    volatile uint32_t GPG_MFOS;               
    volatile uint32_t GPH_MFOS;               
     
    volatile const  uint32_t RESERVE3[8];
     
    volatile uint32_t SRAM_INTCTL;            
    volatile uint32_t SRAM_STATUS;            
    volatile const  uint32_t SRAM_ERRADDR;           
     
    volatile const  uint32_t RESERVE4[1];
     
    volatile uint32_t SRAM_BISTCTL;           
    volatile const  uint32_t SRAM_BISTSTS;           
     
    volatile const  uint32_t RESERVE5[3];
     
    volatile uint32_t HIRCTCTL;               
    volatile uint32_t HIRCTIEN;               
    volatile uint32_t HIRCTISTS;              
    volatile uint32_t IRCTCTL;                
    volatile uint32_t IRCTIEN;                
    volatile uint32_t IRCTISTS;               
     
    volatile const  uint32_t RESERVE6[1];
     
    volatile uint32_t REGLCTL;                
     
    volatile const  uint32_t RESERVE7[58];
     
    volatile uint32_t PORDISAN;               
     
    volatile const  uint32_t RESERVE8;
     
    volatile const  uint32_t CSERVER;                
    volatile uint32_t PLCTL;                  
    volatile const  uint32_t PLSTS;                  
     
    volatile const  uint32_t RESERVE9[128];
     
    volatile uint32_t AHBMCTL;                

} SYS_T;




 












































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   




 

typedef struct
{


    



















































































































 
    volatile uint32_t NMIEN;                  
    volatile const  uint32_t NMISTS;                 

} NMI_T;




 



























































































   
   
   


#pragma no_anon_unions


#line 216 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\clk_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t PWRCTL;                 
    volatile uint32_t AHBCLK;                 
    volatile uint32_t APBCLK0;                
    volatile uint32_t APBCLK1;                
    volatile uint32_t CLKSEL0;                
    volatile uint32_t CLKSEL1;                
    volatile uint32_t CLKSEL2;                
    volatile uint32_t CLKSEL3;                
    volatile uint32_t CLKDIV0;                
    volatile uint32_t CLKDIV1;                
    volatile uint32_t CLKDIV2;                
    volatile uint32_t CLKDIV3;                
    volatile uint32_t CLKDIV4;                
    volatile uint32_t PCLKDIV;                
     
    volatile const  uint32_t RESERVE1[2];
     
    volatile uint32_t PLLCTL;                 
     
    volatile const  uint32_t RESERVE2[3];
     
    volatile const  uint32_t STATUS;                 
     
    volatile const  uint32_t RESERVE3[3];
     
    volatile uint32_t CLKOCTL;                
     
    volatile const  uint32_t RESERVE4[3];
     
    volatile uint32_t CLKDCTL;                
    volatile uint32_t CLKDSTS;                
    volatile uint32_t CDUPB;                  
    volatile uint32_t CDLOWB;                 
     
    volatile const  uint32_t RESERVE5[4];
     
    volatile uint32_t PMUCTL;                 
    volatile uint32_t PMUSTS;                 
    volatile uint32_t LDOCTL;                 
    volatile uint32_t SWKDBCTL;               
    volatile uint32_t PASWKCTL;               
    volatile uint32_t PBSWKCTL;               
    volatile uint32_t PCSWKCTL;               
    volatile uint32_t PDSWKCTL;               
    volatile uint32_t IOPDCTL;                

} CLK_T;




 






































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 217 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\fmc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    

























































































































































































































































































































































































































 
    volatile uint32_t ISPCTL;                 
    volatile uint32_t ISPADDR;                
    volatile uint32_t ISPDAT;                 
    volatile uint32_t ISPCMD;                 
    volatile uint32_t ISPTRG;                 
    volatile const  uint32_t DFBA;                   
    
    volatile const  uint32_t RESERVE0[10];
    
    volatile uint32_t ISPSTS;                 
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CYCCTL;                 
    volatile  uint32_t KPKEY0;                 
    volatile  uint32_t KPKEY1;                 
    volatile  uint32_t KPKEY2;                 
    volatile uint32_t KPKEYTRG;               
    volatile uint32_t KPKEYSTS;               
    volatile const  uint32_t KPKEYCNT;               
    volatile const  uint32_t KPCNT;                  
    
    volatile const  uint32_t RESERVE2[5];
    
    volatile uint32_t MPDAT0;                 
    volatile uint32_t MPDAT1;                 
    volatile uint32_t MPDAT2;                 
    volatile uint32_t MPDAT3;                 
    
    volatile const  uint32_t RESERVE3[12];
    
    volatile const  uint32_t MPSTS;                  
    volatile const  uint32_t MPADDR;                 
    
    volatile const  uint32_t RESERVE4[2];
    
    volatile const  uint32_t XOMR0STS;               
    volatile const  uint32_t XOMR1STS;               
    volatile const  uint32_t XOMR2STS;               
    volatile const  uint32_t XOMR3STS;               
    volatile const  uint32_t XOMSTS;                 

} FMC_T;




 

































































































































































































   
   
   


#pragma no_anon_unions


#line 218 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\gpio_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    








































































































































































































 

    volatile uint32_t MODE;           
    volatile uint32_t DINOFF;         
    volatile uint32_t DOUT;           
    volatile uint32_t DATMSK;         
    volatile const  uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t INTTYPE;        
    volatile uint32_t INTEN;          
    volatile uint32_t INTSRC;         
    volatile uint32_t SMTEN;          
    volatile uint32_t SLEWCTL;        
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t PUSEL;          

} GPIO_T;

typedef struct
{

    





























 

    volatile uint32_t DBCTL;          

} GPIO_DBCTL_T;




 


























































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 219 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\pdma_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    





















































































 
    volatile uint32_t CTL;              
    volatile uint32_t SA;               
    volatile uint32_t DA;               
    volatile uint32_t NEXT;             
} DSCT_T;


typedef struct
{
    
















 
    volatile uint32_t STCR;            
    volatile uint32_t ASOCR;           
} STRIDE_T;

typedef struct
{
    
















 
    volatile uint32_t AICTL;          
    volatile uint32_t RCNT;           
} REPEAT_T;

typedef struct
{


    



























































































































































































































































































































































































 
    DSCT_T DSCT[16];
    volatile const  uint32_t CURSCAT[16];               
    
    volatile const  uint32_t RESERVE1[176];
    
    volatile uint32_t CHCTL;                  
    volatile  uint32_t PAUSE;                  
    volatile  uint32_t SWREQ;                  
    volatile const  uint32_t TRGSTS;                 
    volatile uint32_t PRISET;                 
    volatile  uint32_t PRICLR;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t ABTSTS;                 
    volatile uint32_t TDSTS;                  
    volatile uint32_t ALIGN;                  
    volatile const  uint32_t TACTSTS;                
    volatile uint32_t TOUTPSC;                
    volatile uint32_t TOUTEN;                 
    volatile uint32_t TOUTIEN;                
    volatile uint32_t SCATBA;                 
    volatile uint32_t TOC0_1;                 
    
    volatile const  uint32_t RESERVE2[7];
    
    volatile uint32_t CHRST;                  
    
    volatile const  uint32_t RESERVE3[7];
    
    volatile uint32_t REQSEL0_3;              
    volatile uint32_t REQSEL4_7;              
    volatile uint32_t REQSEL8_11;             
    volatile uint32_t REQSEL12_15;            
    
    volatile const  uint32_t RESERVE4[28];
    
    STRIDE_T     STRIDE[6];
    
    volatile uint32_t RESERVE5[52];
    
    REPEAT_T    REPEAT[2];
} PDMA_T;




 


















































































































































































































































































   
   
   


#pragma no_anon_unions


#line 220 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\timer_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    









































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CMP;                    
    volatile uint32_t INTSTS;                 
    volatile uint32_t CNT;                    
    volatile const  uint32_t CAP;                    
    volatile uint32_t EXTCTL;                 
    volatile uint32_t EINTSTS;                
    volatile uint32_t TRGCTL;                 
    volatile uint32_t ALTCTL;                 
     
    volatile const  uint32_t RESERVE0[7];
     
    volatile uint32_t PWMCTL;                 
    volatile uint32_t PWMCLKSRC;              
    volatile uint32_t PWMCLKPSC;              
    volatile uint32_t PWMCNTCLR;              
    volatile uint32_t PWMPERIOD;              
    volatile uint32_t PWMCMPDAT;              
    volatile uint32_t PWMDTCTL;               
    volatile const  uint32_t PWMCNT;                 
    volatile uint32_t PWMMSKEN;               
    volatile uint32_t PWMMSK;                 
    volatile uint32_t PWMBNF;                 
    volatile uint32_t PWMFAILBRK;             
    volatile uint32_t PWMBRKCTL;              
    volatile uint32_t PWMPOLCTL;              
    volatile uint32_t PWMPOEN;                
    volatile  uint32_t PWMSWBRK;               
    volatile uint32_t PWMINTEN0;              
    volatile uint32_t PWMINTEN1;              
    volatile uint32_t PWMINTSTS0;             
    volatile uint32_t PWMINTSTS1;             
    volatile uint32_t PWMEADCTS;              
    volatile uint32_t PWMSCTL;                
    volatile  uint32_t PWMSTRG;                
    volatile uint32_t PWMSTATUS;              
    volatile const  uint32_t PWMPBUF;                
    volatile const  uint32_t PWMCMPBUF;              

} TIMER_T;




 








































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 221 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\wdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





























































































 
    volatile uint32_t CTL;                    
    volatile uint32_t ALTCTL;                 
    volatile  uint32_t RSTCNT;                 

} WDT_T;




 








































   
   
   


#pragma no_anon_unions


#line 222 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\wwdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






































































 
    volatile  uint32_t RLDCNT;                 
    volatile uint32_t CTL;                    
    volatile uint32_t STATUS;                 
    volatile const  uint32_t CNT;                    

} WWDT_T;




 




























   
   
   


#pragma no_anon_unions


#line 223 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\rtc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t INIT;                   
    volatile uint32_t RWEN;                   
    volatile uint32_t FREQADJ;                
    volatile uint32_t TIME;                   
    volatile uint32_t CAL;                    
    volatile uint32_t CLKFMT;                 
    volatile uint32_t WEEKDAY;                
    volatile uint32_t TALM;                   
    volatile uint32_t CALM;                   
    volatile const  uint32_t LEAPYEAR;               
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t TICK;                   
    volatile uint32_t TAMSK;                  
    volatile uint32_t CAMSK;                  
    volatile uint32_t SPRCTL;                 
    volatile uint32_t SPR[20];                
    
    volatile const  uint32_t RESERVE0[28];
    
    volatile uint32_t LXTCTL;                 
    volatile uint32_t GPIOCTL0;               
    volatile uint32_t GPIOCTL1;               
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t DSTCTL;                 
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t TAMPCTL;                
    
    volatile const  uint32_t RESERVE3[1];
    
    volatile uint32_t TAMPSEED;               
    
    volatile const  uint32_t RESERVE4[1];
    
    volatile const  uint32_t TAMPTIME;               
    volatile const  uint32_t TAMPCAL;                

} RTC_T;




 





































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 224 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\epwm_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    














 
    volatile uint32_t RCAPDAT;  
    volatile uint32_t FCAPDAT;  
} ECAPDAT_T;

typedef struct
{


    














































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile uint32_t SYNC;                   
    volatile uint32_t SWSYNC;                 
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC[3];              
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    volatile uint32_t LOAD;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t PERIOD[6];              
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CMPDAT[6];              
    
    volatile const  uint32_t RESERVE2[2];
    
    volatile uint32_t DTCTL[3];               
    
    volatile const  uint32_t RESERVE3[1];
    
    volatile uint32_t PHS[3];                 
    
    volatile const  uint32_t RESERVE4[1];
    
    volatile const  uint32_t CNT[6];                 
    
    volatile const  uint32_t RESERVE5[2];
    
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    volatile uint32_t BNF;                    
    volatile uint32_t FAILBRK;                
    volatile uint32_t BRKCTL[3];              
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    volatile  uint32_t SWBRK;                  
    volatile uint32_t INTEN0;                 
    volatile uint32_t INTEN1;                 
    volatile uint32_t INTSTS0;                
    volatile uint32_t INTSTS1;                
    
    volatile const  uint32_t RESERVE6[1];
    
    volatile uint32_t DACTRGEN;               
    volatile uint32_t EADCTS0;                
    volatile uint32_t EADCTS1;                
    volatile uint32_t FTCMPDAT[3];            
    
    volatile const  uint32_t RESERVE7[1];
    
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    volatile uint32_t LEBCTL;                 
    volatile uint32_t LEBCNT;                 
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE8[3];
    
    volatile uint32_t IFA[6];                 
    
    volatile const  uint32_t RESERVE9[2];
    
    volatile uint32_t AINTSTS;                
    volatile uint32_t AINTEN;                 
    volatile uint32_t APDMACTL;               
    
    volatile const  uint32_t RESERVE10[41];
    
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    ECAPDAT_T CAPDAT[6];                   
    volatile uint32_t PDMACTL;                
    volatile const  uint32_t PDMACAP[3];             
    
    volatile const  uint32_t RESERVE11[1];
    
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    
    volatile const  uint32_t RESERVE12[43];
    
    volatile const  uint32_t PBUF[6];                
    volatile const  uint32_t CMPBUF[6];              
    volatile const  uint32_t CPSCBUF[3];             
    volatile const  uint32_t FTCBUF[3];              
    volatile uint32_t FTCI;                   

} EPWM_T;




 

















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 225 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\bpwm_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    














 
    volatile uint32_t RCAPDAT;  
    volatile uint32_t FCAPDAT;  
} BCAPDAT_T;

typedef struct
{


    


























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC;                 
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    
    volatile const  uint32_t RESERVE2[2];
    
    volatile uint32_t PERIOD;                 
    
    volatile const  uint32_t RESERVE3[7];
    
    volatile uint32_t CMPDAT[6];              
    
    volatile const  uint32_t RESERVE4[10];
    
    volatile const  uint32_t CNT;                    
    
    volatile const  uint32_t RESERVE5[7];
    
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    
    volatile const  uint32_t RESERVE6[5];
    
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    
    volatile const  uint32_t RESERVE7[1];
    
    volatile uint32_t INTEN;                  
    
    volatile const  uint32_t RESERVE8[1];
    
    volatile uint32_t INTSTS;                 
    
    volatile const  uint32_t RESERVE9[3];
    
    volatile uint32_t EADCTS0;                
    volatile uint32_t EADCTS1;                
    
    volatile const  uint32_t RESERVE10[4];
    
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    
    volatile const  uint32_t RESERVE11[2];
    
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE12[55];
    
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    BCAPDAT_T CAPDAT[6];                   
    
    volatile const  uint32_t RESERVE13[5];
    
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    
    volatile const  uint32_t RESERVE14[43];
    
    volatile const  uint32_t PBUF;                   
    
    volatile const  uint32_t RESERVE15[5];
    
    volatile const  uint32_t CMPBUF[6];              

} BPWM_T;




 






























































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 226 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\qei_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































 
    volatile uint32_t CNT;                    
    volatile uint32_t CNTHOLD;                
    volatile uint32_t CNTLATCH;               
    volatile uint32_t CNTCMP;                 
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t CNTMAX;                 
    volatile uint32_t CTL;                    
    
    volatile const  uint32_t RESERVE1[4];
    
    volatile uint32_t STATUS;                 

} QEI_T;




 

































































































   
   
   


#pragma no_anon_unions


#line 227 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\ecap_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    

















































































































































































































 
    volatile uint32_t CNT;                    
    volatile uint32_t HLD0;                   
    volatile uint32_t HLD1;                   
    volatile uint32_t HLD2;                   
    volatile uint32_t CNTCMP;                 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS;                 

} ECAP_T;




 































































































































   
   
   


#pragma no_anon_unions


#line 228 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\uart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t DAT;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t FIFO;                   
    volatile uint32_t LINE;                   
    volatile uint32_t MODEM;                  
    volatile uint32_t MODEMSTS;               
    volatile uint32_t FIFOSTS;                
    volatile uint32_t INTSTS;                 
    volatile uint32_t TOUT;                   
    volatile uint32_t BAUD;                   
    volatile uint32_t IRDA;                   
    volatile uint32_t ALTCTL;                 
    volatile uint32_t FUNCSEL;                
    volatile uint32_t LINCTL;                 
    volatile uint32_t LINSTS;                 
    volatile uint32_t BRCOMP;                 
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t DWKCOMP;                

} UART_T;




 

























































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 229 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\emac_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    












































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CAMCTL;                 
    volatile uint32_t CAMEN;                  
    volatile uint32_t CAM0M;                  
    volatile uint32_t CAM0L;                  
    volatile uint32_t CAM1M;                  
    volatile uint32_t CAM1L;                  
    volatile uint32_t CAM2M;                  
    volatile uint32_t CAM2L;                  
    volatile uint32_t CAM3M;                  
    volatile uint32_t CAM3L;                  
    volatile uint32_t CAM4M;                  
    volatile uint32_t CAM4L;                  
    volatile uint32_t CAM5M;                  
    volatile uint32_t CAM5L;                  
    volatile uint32_t CAM6M;                  
    volatile uint32_t CAM6L;                  
    volatile uint32_t CAM7M;                  
    volatile uint32_t CAM7L;                  
    volatile uint32_t CAM8M;                  
    volatile uint32_t CAM8L;                  
    volatile uint32_t CAM9M;                  
    volatile uint32_t CAM9L;                  
    volatile uint32_t CAM10M;                 
    volatile uint32_t CAM10L;                 
    volatile uint32_t CAM11M;                 
    volatile uint32_t CAM11L;                 
    volatile uint32_t CAM12M;                 
    volatile uint32_t CAM12L;                 
    volatile uint32_t CAM13M;                 
    volatile uint32_t CAM13L;                 
    volatile uint32_t CAM14M;                 
    volatile uint32_t CAM14L;                 
    volatile uint32_t CAM15MSB;               
    volatile uint32_t CAM15LSB;               
    volatile uint32_t TXDSA;                  
    volatile uint32_t RXDSA;                  
    volatile uint32_t CTL;                    
    volatile uint32_t MIIMDAT;                
    volatile uint32_t MIIMCTL;                
    volatile uint32_t FIFOCTL;                
    volatile  uint32_t TXST;                   
    volatile  uint32_t RXST;                   
    volatile uint32_t MRFL;                   
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t GENSTS;                 
    volatile uint32_t MPCNT;                  
    volatile const  uint32_t RPCNT;                  
     
    volatile const  uint32_t RESERVE0[2];
     
    volatile uint32_t FRSTS;                  
    volatile const  uint32_t CTXDSA;                 
    volatile const  uint32_t CTXBSA;                 
    volatile const  uint32_t CRXDSA;                 
    volatile const  uint32_t CRXBSA;                 
     
    volatile const  uint32_t RESERVE1[9];
     
    volatile uint32_t TSCTL;                  
     
    volatile const  uint32_t RESERVE2[3];
     
    volatile const  uint32_t TSSEC;                  
    volatile const  uint32_t TSSUBSEC;               
    volatile uint32_t TSINC;                  
    volatile uint32_t TSADDEND;               
    volatile uint32_t UPDSEC;                 
    volatile uint32_t UPDSUBSEC;              
    volatile uint32_t ALMSEC;                 
    volatile uint32_t ALMSUBSEC;              

} EMAC_T;




 














































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 230 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\sc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    













































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t DAT;                    
    volatile uint32_t CTL;                    
    volatile uint32_t ALTCTL;                 
    volatile uint32_t EGT;                    
    volatile uint32_t RXTOUT;                 
    volatile uint32_t ETUCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t STATUS;                 
    volatile uint32_t PINCTL;                 
    volatile uint32_t TMRCTL0;                
    volatile uint32_t TMRCTL1;                
    volatile uint32_t TMRCTL2;                
    volatile uint32_t UARTCTL;                
     
    volatile const  uint32_t RESERVE0[5];
     
    volatile uint32_t ACTCTL;                 

} SC_T;




 













































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 231 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\i2s_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    













































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CLKDIV;                 
    volatile uint32_t IEN;                    
    volatile uint32_t STATUS0;                
    volatile  uint32_t TXFIFO;                 
    volatile const  uint32_t RXFIFO;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS1;                

} I2S_T;




 




























































































































































































































   
   
   


#pragma no_anon_unions


#line 232 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\spi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








































































































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile  uint32_t TX;                     
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile const  uint32_t RX;                     
    
    volatile const  uint32_t RESERVE2[11];
    
    volatile uint32_t I2SCTL;                 
    volatile uint32_t I2SCLK;                 
    volatile uint32_t I2SSTS;                 

} SPI_T;




 





















































































































































































































































































   
   
   


#pragma no_anon_unions


#line 233 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\qspi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    











































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile  uint32_t TX;                     
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile const  uint32_t RX;                     

} QSPI_T;




 








































































































































































































   
   
   


#pragma no_anon_unions


#line 234 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\spim_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

























































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t RXCLKDLY;               
    volatile const  uint32_t RX[4];                  
    volatile uint32_t TX[4];                  
    volatile uint32_t SRAMADDR;               
    volatile uint32_t DMACNT;                 
    volatile uint32_t FADDR;                  
    volatile  uint32_t KEY1;                   
    volatile  uint32_t KEY2;                   
    volatile uint32_t DMMCTL;                 
    volatile uint32_t CTL2;                   

} SPIM_T;




 






















































































































   
   
   


#pragma no_anon_unions


#line 235 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\i2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

















































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t ADDR0;                  
    volatile uint32_t DAT;                    
    volatile const  uint32_t STATUS0;                
    volatile uint32_t CLKDIV;                 
    volatile uint32_t TOCTL;                  
    volatile uint32_t ADDR1;                  
    volatile uint32_t ADDR2;                  
    volatile uint32_t ADDR3;                  
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t ADDRMSK2;               
    volatile uint32_t ADDRMSK3;               
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS1;                
    volatile uint32_t TMCTL;                  
    volatile uint32_t BUSCTL;                 
    volatile uint32_t BUSTCTL;                
    volatile uint32_t BUSSTS;                 
    volatile uint32_t PKTSIZE;                
    volatile const  uint32_t PKTCRC;                 
    volatile uint32_t BUSTOUT;                
    volatile uint32_t CLKTOUT;                

} I2C_T;




 

























































































































































































































   
   
   


#pragma no_anon_unions


#line 236 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\uuart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t DATIN0;                 
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile uint32_t CTLIN0;                 
    
    volatile const  uint32_t RESERVE2[1];
    
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile uint32_t TXDAT;                  
    volatile uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    
    volatile const  uint32_t RESERVE3[4];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} UUART_T;




 



















































































































































































































   
   
   


#pragma no_anon_unions


#line 237 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\uspi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    












































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t DATIN0;                 
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile uint32_t CTLIN0;                 
    
    volatile const  uint32_t RESERVE2[1];
    
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    
    volatile const  uint32_t RESERVE3[4];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} USPI_T;




 













































































































































































































   
   
   


#pragma no_anon_unions


#line 238 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\ui2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






















































































































































































































































































































































 
    volatile uint32_t CTL;                    
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE1[8];
    
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t DEVADDR0;               
    volatile uint32_t DEVADDR1;               
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                
    
    volatile const  uint32_t RESERVE3[8];
    
    volatile uint32_t ADMAT;                  
    volatile uint32_t TMCTL;                  

} UI2C_T;




 






































































































































































   
   
   


#pragma no_anon_unions


#line 239 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\can_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    























































































































































































































 
    volatile uint32_t CREQ;          
    volatile uint32_t CMASK;         
    volatile uint32_t MASK1;         
    volatile uint32_t MASK2;         
    volatile uint32_t ARB1;          
    volatile uint32_t ARB2;          
    volatile uint32_t MCON;          
    volatile uint32_t DAT_A1;        
    volatile uint32_t DAT_A2;        
    volatile uint32_t DAT_B1;        
    volatile uint32_t DAT_B2;        
    
    volatile const uint32_t RESERVE0[13];
    
} CAN_IF_T;


typedef struct
{


    


























































































































































































































 
    volatile uint32_t CON;                    
    volatile uint32_t STATUS;                 
    volatile const  uint32_t ERR;                    
    volatile uint32_t BTIME;                  
    volatile const  uint32_t IIDR;                   
    volatile uint32_t TEST;                   
    volatile uint32_t BRPE;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile CAN_IF_T IF[2];
    
    volatile const  uint32_t RESERVE2[8];
    
    volatile const  uint32_t TXREQ1;                 
    volatile const  uint32_t TXREQ2;                 
    
    volatile const  uint32_t RESERVE3[6];
    
    volatile const  uint32_t NDAT1;                  
    volatile const  uint32_t NDAT2;                  
    
    volatile const  uint32_t RESERVE4[6];
    
    volatile const  uint32_t IPND1;                  
    volatile const  uint32_t IPND2;                  
    
    volatile const  uint32_t RESERVE5[6];
    
    volatile const  uint32_t MVLD1;                  
    volatile const  uint32_t MVLD2;                  
    volatile uint32_t WU_EN;                  
    volatile uint32_t WU_STATUS;              

} CAN_T;




 































































































































































































































   
   
   


#pragma no_anon_unions


#line 240 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\sdh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    





















































































































































































































































































































 

    volatile uint32_t FB[32];                 
    
    volatile const  uint32_t RESERVE0[224];
    
    volatile uint32_t DMACTL;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t DMASA;                  
    volatile const  uint32_t DMABCNT;                
    volatile uint32_t DMAINTEN;               
    volatile uint32_t DMAINTSTS;              
    
    volatile const  uint32_t RESERVE2[250];
    
    volatile uint32_t GCTL;                   
    volatile uint32_t GINTEN;                 
    volatile const  uint32_t GINTSTS;                
    
    volatile const  uint32_t RESERVE3[5];
    
    volatile uint32_t CTL;                    
    volatile uint32_t CMDARG;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t RESP0;                  
    volatile const  uint32_t RESP1;                  
    volatile uint32_t BLEN;                   
    volatile uint32_t TOUT;                   

} SDH_T;





 


























































































































































   
   
   


#pragma no_anon_unions




#line 241 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\ebi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    









































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t TCTL0;                  
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CTL1;                   
    volatile uint32_t TCTL1;                  
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CTL2;                   
    volatile uint32_t TCTL2;                  

} EBI_T;




 









































































































































































   
   
   


#pragma no_anon_unions


#line 242 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\usbd_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    































































 
    volatile uint32_t BUFSEG;                
    volatile uint32_t MXPLD;                 
    volatile uint32_t CFG;                   
    volatile uint32_t CFGP;                  

} USBD_EP_T;

typedef struct
{


    













































































































































































































































































































 

    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t FADDR;                  
    volatile const  uint32_t EPSTS;                  
    volatile uint32_t ATTR;                   
    volatile const  uint32_t VBUSDET;                
    volatile uint32_t STBUFSEG;               
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile const  uint32_t EPSTS0;                 
    volatile const  uint32_t EPSTS1;                 
    
    volatile const  uint32_t RESERVE1[24];
    
    volatile const  uint32_t LPMATTR;                
    volatile const  uint32_t FN;                     
    volatile uint32_t SE0;                    
    
    volatile const  uint32_t RESERVE2[283];
    
    USBD_EP_T     EP[12];                 

} USBD_T;





 



























































































































































































   
   
   


#pragma no_anon_unions


#line 243 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\hsusbd_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    























































































































































































































































 

    union
    {
        volatile uint32_t EPDAT;
        volatile uint8_t  EPDAT_BYTE;

    };                                   

    volatile uint32_t EPINTSTS;              
    volatile uint32_t EPINTEN;               
    volatile const  uint32_t EPDATCNT;              
    volatile uint32_t EPRSPCTL;              
    volatile uint32_t EPMPS;                 
    volatile uint32_t EPTXCNT;               
    volatile uint32_t EPCFG;                 
    volatile uint32_t EPBUFST;               
    volatile uint32_t EPBUFEND;              

} HSUSBD_EP_T;

typedef struct
{

    




















































































































































































































































































































































































































































































































































































 

    volatile const  uint32_t GINTSTS;                
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t GINTEN;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t BUSINTSTS;              
    volatile uint32_t BUSINTEN;               
    volatile uint32_t OPER;                   
    volatile const  uint32_t FRAMECNT;               
    volatile uint32_t FADDR;                  
    volatile uint32_t TEST;                   

    union
    {
        volatile uint32_t CEPDAT;
        volatile uint8_t  CEPDAT_BYTE;

    };                                    

    volatile uint32_t CEPCTL;                 
    volatile uint32_t CEPINTEN;               
    volatile uint32_t CEPINTSTS;              
    volatile uint32_t CEPTXCNT;               
    volatile const  uint32_t CEPRXCNT;               
    volatile const  uint32_t CEPDATCNT;              
    volatile const  uint32_t SETUP1_0;               
    volatile const  uint32_t SETUP3_2;               
    volatile const  uint32_t SETUP5_4;               
    volatile const  uint32_t SETUP7_6;               
    volatile uint32_t CEPBUFST;               
    volatile uint32_t CEPBUFEND;              
    volatile uint32_t DMACTL;                 
    volatile uint32_t DMACNT;                 

    HSUSBD_EP_T EP[12];

    
    volatile const  uint32_t RESERVE2[303];
    
    volatile uint32_t DMAADDR;                
    volatile uint32_t PHYCTL;                 

} HSUSBD_T;




 






































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 244 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\usbh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    
























































































































































































































































































































































































































































































































 
    volatile const  uint32_t HcRevision;             
    volatile uint32_t HcControl;              
    volatile uint32_t HcCommandStatus;        
    volatile uint32_t HcInterruptStatus;      
    volatile uint32_t HcInterruptEnable;      
    volatile uint32_t HcInterruptDisable;     
    volatile uint32_t HcHCCA;                 
    volatile uint32_t HcPeriodCurrentED;      
    volatile uint32_t HcControlHeadED;        
    volatile uint32_t HcControlCurrentED;     
    volatile uint32_t HcBulkHeadED;           
    volatile uint32_t HcBulkCurrentED;        
    volatile uint32_t HcDoneHead;             
    volatile uint32_t HcFmInterval;           
    volatile const  uint32_t HcFmRemaining;          
    volatile const  uint32_t HcFmNumber;             
    volatile uint32_t HcPeriodicStart;        
    volatile uint32_t HcLSThreshold;          
    volatile uint32_t HcRhDescriptorA;        
    volatile uint32_t HcRhDescriptorB;        
    volatile uint32_t HcRhStatus;             
    volatile uint32_t HcRhPortStatus[2];      
    
    volatile const  uint32_t RESERVE0[105];
    
    volatile uint32_t HcPhyControl;           
    volatile uint32_t HcMiscControl;          

} USBH_T;




 




























































































































































































































   
   
   


#pragma no_anon_unions


#line 245 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\hsusbh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































 
    volatile const  uint32_t EHCVNR;                 
    volatile const  uint32_t EHCSPR;                 
    volatile const  uint32_t EHCCPR;                 
    
    volatile const  uint32_t RESERVE0[5];
    
    volatile uint32_t UCMDR;                  
    volatile uint32_t USTSR;                  
    volatile uint32_t UIENR;                  
    volatile uint32_t UFINDR;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t UPFLBAR;                
    volatile uint32_t UCALAR;                 
    volatile uint32_t UASSTR;                 
    
    volatile const  uint32_t RESERVE2[8];
    
    volatile uint32_t UCFGR;                  
    volatile uint32_t UPSCR[2];               
    
    volatile const  uint32_t RESERVE3[22];
    
    volatile uint32_t USBPCR0;                
    volatile uint32_t USBPCR1;                

} HSUSBH_T;




 






































































































































































   
   
   


#pragma no_anon_unions


#line 246 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\otg_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
























































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t PHYCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t STATUS;                 

} OTG_T;





 


































































































































   
   
   


#pragma no_anon_unions


#line 247 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\hsotg_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
























































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t PHYCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t STATUS;                 

} HSOTG_T;




 


































































































































   
   
   


#pragma no_anon_unions


#line 248 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\crc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































 
    volatile uint32_t CTL;                    
    volatile uint32_t DAT;                    
    volatile uint32_t SEED;                   
    volatile const  uint32_t CHECKSUM;               

} CRC_T;




 


































   
   
   


#pragma no_anon_unions


#line 249 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\crypto_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    










































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t PRNG_CTL;               
    volatile  uint32_t PRNG_SEED;              
    volatile const  uint32_t PRNG_KEY[8];            
    
    volatile const  uint32_t RESERVE0[8];
    
    volatile const  uint32_t AES_FDBCK[4];           
    volatile const  uint32_t TDES_FDBCKH;            
    volatile const  uint32_t TDES_FDBCKL;            
    
    volatile const  uint32_t RESERVE1[38];
    
    volatile uint32_t AES_CTL;                
    volatile const  uint32_t AES_STS;                
    volatile uint32_t AES_DATIN;              
    volatile const  uint32_t AES_DATOUT;             
    volatile uint32_t AES0_KEY[8];            
    volatile uint32_t AES0_IV[4];             
    volatile uint32_t AES0_SADDR;             
    volatile uint32_t AES0_DADDR;             
    volatile uint32_t AES0_CNT;               
    volatile uint32_t AES1_KEY[8];            
    volatile uint32_t AES1_IV[4];             
    volatile uint32_t AES1_SADDR;             
    volatile uint32_t AES1_DADDR;             
    volatile uint32_t AES1_CNT;               
    volatile uint32_t AES2_KEY[8];            
    volatile uint32_t AES2_IV[4];             
    volatile uint32_t AES2_SADDR;             
    volatile uint32_t AES2_DADDR;             
    volatile uint32_t AES2_CNT;               
    volatile uint32_t AES3_KEY[8];            
    volatile uint32_t AES3_IV[4];             
    volatile uint32_t AES3_SADDR;             
    volatile uint32_t AES3_DADDR;             
    volatile uint32_t AES3_CNT;               
    volatile uint32_t TDES_CTL;               
    volatile const  uint32_t TDES_STS;               
    volatile uint32_t TDES0_KEY1H;            
    volatile uint32_t TDES0_KEY1L;            
    volatile uint32_t TDES0_KEY2H;            
    volatile uint32_t TDES0_KEY2L;            
    volatile uint32_t TDES0_KEY3H;            
    volatile uint32_t TDES0_KEY3L;            
    volatile uint32_t TDES0_IVH;              
    volatile uint32_t TDES0_IVL;              
    volatile uint32_t TDES0_SA;               
    volatile uint32_t TDES0_DA;               
    volatile uint32_t TDES0_CNT;              
    volatile uint32_t TDES_DATIN;             
    volatile const  uint32_t TDES_DATOUT;            
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t TDES1_KEY1H;            
    volatile uint32_t TDES1_KEY1L;            
    volatile uint32_t TDES1_KEY2H;            
    volatile uint32_t TDES1_KEY2L;            
    volatile uint32_t TDES1_KEY3H;            
    volatile uint32_t TDES1_KEY3L;            
    volatile uint32_t TDES1_IVH;              
    volatile uint32_t TDES1_IVL;              
    volatile uint32_t TDES1_SA;               
    volatile uint32_t TDES1_DA;               
    volatile uint32_t TDES1_CNT;              
    
    volatile const  uint32_t RESERVE3[5];
    
    volatile uint32_t TDES2_KEY1H;            
    volatile uint32_t TDES2_KEY1L;            
    volatile uint32_t TDES2_KEY2H;            
    volatile uint32_t TDES2_KEY2L;            
    volatile uint32_t TDES2_KEY3H;            
    volatile uint32_t TDES2_KEY3L;            
    volatile uint32_t TDES2_IVH;              
    volatile uint32_t TDES2_IVL;              
    volatile uint32_t TDES2_SA;               
    volatile uint32_t TDES2_DA;               
    volatile uint32_t TDES2_CNT;              
    
    volatile const  uint32_t RESERVE4[5];
    
    volatile uint32_t TDES3_KEY1H;            
    volatile uint32_t TDES3_KEY1L;            
    volatile uint32_t TDES3_KEY2H;            
    volatile uint32_t TDES3_KEY2L;            
    volatile uint32_t TDES3_KEY3H;            
    volatile uint32_t TDES3_KEY3L;            
    volatile uint32_t TDES3_IVH;              
    volatile uint32_t TDES3_IVL;              
    volatile uint32_t TDES3_SA;               
    volatile uint32_t TDES3_DA;               
    volatile uint32_t TDES3_CNT;              
    
    volatile const  uint32_t RESERVE5[3];
    
    volatile uint32_t HMAC_CTL;               
    volatile const  uint32_t HMAC_STS;               
    volatile const  uint32_t HMAC_DGST[16];          
    volatile uint32_t HMAC_KEYCNT;            
    volatile uint32_t HMAC_SADDR;             
    volatile uint32_t HMAC_DMACNT;            
    volatile uint32_t HMAC_DATIN;             
    
    volatile const  uint32_t RESERVE6[298];
    
    volatile uint32_t ECC_CTL;                
    volatile const  uint32_t ECC_STS;                
    volatile uint32_t ECC_X1[18];             
    volatile uint32_t ECC_Y1[18];             
    volatile uint32_t ECC_X2[18];             
    volatile uint32_t ECC_Y2[18];             
    volatile uint32_t ECC_A[18];              
    volatile uint32_t ECC_B[18];              
    volatile uint32_t ECC_N[18];              
    volatile  uint32_t ECC_K[18];              
    volatile uint32_t ECC_SADDR;              
    volatile uint32_t ECC_DADDR;              
    volatile uint32_t ECC_STARTREG;           
    volatile uint32_t ECC_WORDCNT;            

} CRPT_T;




 

























































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 250 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\trng_reg.h"
 





 







 

 



 

typedef struct
{


    






























































 
    volatile uint32_t CTL;                    
    volatile const  uint32_t DATA;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t ACT;                    

} TRNG_T;




 




























   
   
   


#line 251 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\eadc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
































































































































































































































































































































































































































































































































 
    volatile const  uint32_t DAT[19];                
    volatile const  uint32_t CURDAT;                 
    volatile uint32_t CTL;                    
    volatile  uint32_t SWTRG;                  
    volatile uint32_t PENDSTS;                
    volatile uint32_t OVSTS;                  
    
    volatile const  uint32_t RESERVE0[8];
    
    volatile uint32_t SCTL[19];               
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t INTSRC[4];              
    volatile uint32_t CMP[4];                 
    volatile const  uint32_t STATUS0;                
    volatile const  uint32_t STATUS1;                
    volatile uint32_t STATUS2;                
    volatile const  uint32_t STATUS3;                
    volatile const  uint32_t DDAT[4];                
    volatile uint32_t PWRM;                   
    volatile uint32_t CALCTL;                 
    volatile uint32_t CALDWRD;                
    
    volatile const  uint32_t RESERVE2[5];
    
    volatile uint32_t PDMACTL;                
} EADC_T;




 









































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 252 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\dac_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t SWTRG;                  
    volatile uint32_t DAT;                    
    volatile const  uint32_t DATOUT;                 
    volatile uint32_t STATUS;                 
    volatile uint32_t TCTL;                   

} DAC_T;




 























































   
   
   


#pragma no_anon_unions


#line 253 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\acmp_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





















































































































 
    volatile uint32_t CTL[2];                 
    volatile uint32_t STATUS;                 
    volatile uint32_t VREF;                   

} ACMP_T;




 









































































   
   
   


#pragma no_anon_unions


#line 254 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\opa_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    




















































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t STATUS;                 
    volatile uint32_t CALCTL;                 
    volatile const  uint32_t CALST;                  

} OPA_T;




 




































































































   
   
   


#pragma no_anon_unions




#line 255 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\ccap_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct {


    













































































































































































































































































 
    volatile uint32_t CTL;
    volatile uint32_t PAR;
    volatile uint32_t INT;
    volatile uint32_t POSTERIZE;
    volatile uint32_t MD;
    volatile uint32_t MDADDR;
    volatile uint32_t MDYADDR;
    volatile uint32_t SEPIA;
    volatile uint32_t CWSP;
    volatile uint32_t CWS;
    volatile uint32_t PKTSL;
    volatile uint32_t PLNSL;
    volatile uint32_t FRCTL;
    volatile uint32_t STRIDE;
    
    uint32_t RESERVE0[1];
    
    volatile uint32_t FIFOTH;
    volatile uint32_t CMPADDR;
    volatile uint32_t LUMA_Y1_THD;
    volatile uint32_t PKTSM;
    
    uint32_t RESERVE2[5];
    
    volatile uint32_t PKTBA0;
} CCAP_T;




 




























































































































































   
   
   


#pragma no_anon_unions


#line 256 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"





 
 






 
#line 296 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

 
#line 321 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"


 
#line 349 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 357 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   





 

#line 396 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 404 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 458 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 



   

 
 
 



 
















 
#line 650 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

 










   


 
 
 
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 





 












 



 



 


 
 
 
#line 45 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 75 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 95 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 
#line 109 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 120 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 132 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 




 
 
 






 
 
#line 258 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 358 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 483 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 604 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 712 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 779 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 835 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 887 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 973 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1049 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1108 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1141 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1191 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1246 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1286 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1321 "..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

   




 








 









 









 









 









 










 









 









 









 

















 









 









 









 









 









 









 









 









 









 









 









 
















 



 
 
 
 
static __inline void SYS_UnlockReg(void);
static __inline void SYS_LockReg(void);







 
static __inline void SYS_UnlockReg(void)
{
    do
    {
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x59UL;
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x16UL;
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x88UL;
    }
    while(((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL == 0UL);
}







 
static __inline void SYS_LockReg(void)
{
    ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0UL;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_SetPowerLevel(uint32_t u32PowerLevel);
void SYS_SetVRef(uint32_t u32VRefCTL);

   

   

   








 
#line 669 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
 





 











 



 



 


#line 41 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






#line 57 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"







#line 71 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

















 
 
 




#line 101 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 108 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 115 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 122 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"




















 
 
 






































 
 
 


























































 
 
 
#line 253 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 




 
 
 



 
 
 





 
 
 
#line 284 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 298 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 309 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






















 
 
 

 

#line 351 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 360 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 426 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 436 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 452 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 472 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






























 
 
 












#line 533 "..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

   



 























 




























 


 
 
 
 
static __inline void CLK_SysTickDelay(uint32_t us);
static __inline void CLK_SysTickLongDelay(uint32_t us);









 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = 0x0UL;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL)
    {
    }

     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
}








 
static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;

     
    delay = 349525UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }

        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;

    }
    while(us > 0UL);

}


void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
void CLK_SetPowerDownMode(uint32_t u32PDMode);
void CLK_EnableDPDWKPin(uint32_t u32TriggerType);
uint32_t CLK_GetPMUWKSrc(void);
void CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx);
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx);

   

   

   







 
#line 670 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"
 





 












 




 




 



 
 
 
#line 60 "..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"

 
 
 




   




 

 
 
 









 









 














 








 









 













 










 









 









 









 









 









 









 









 









 









 














 









 









 


















 












 











 













 












 









 
















 









 





 
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysSel);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);



   

   

   








 
#line 672 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\dac.h"
 





 











 



 




 

 
 
 



#line 48 "..\\..\\..\\Library\\StdDriver\\inc\\dac.h"





   




 







 








 








 









 








 









 









 








 








 








 








 











 









 










 










 









 









 








 








 







 


void DAC_Open(DAC_T *dac, uint32_t u32Ch, uint32_t u32TrgSrc);
void DAC_Close(DAC_T *dac, uint32_t u32Ch);
uint32_t DAC_SetDelayTime(DAC_T *dac, uint32_t u32Delay);

   

   

   







 
#line 673 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\emac.h"
 





 











 



 



 











   




 







 








 







 








 







 







 







 







 







 







 









 







 






 






 


void EMAC_Open(uint8_t *pu8MacAddr);
void EMAC_Close(void);
void EMAC_SetMacAddr(uint8_t *pu8MacAddr);
void EMAC_EnableCamEntry(uint32_t u32Entry, uint8_t pu8MacAddr[]);
void EMAC_DisableCamEntry(uint32_t u32Entry);

uint32_t EMAC_RecvPkt(uint8_t *pu8Data, uint32_t *pu32Size);
uint32_t EMAC_RecvPktTS(uint8_t *pu8Data, uint32_t *pu32Size, uint32_t *pu32Sec, uint32_t *pu32Nsec);
void EMAC_RecvPktDone(void);

uint32_t EMAC_SendPkt(uint8_t *pu8Data, uint32_t u32Size);
uint32_t EMAC_SendPktDone(void);
uint32_t EMAC_SendPktDoneTS(uint32_t *pu32Sec, uint32_t *pu32Nsec);

void EMAC_EnableTS(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_DisableTS(void);
void EMAC_GetTime(uint32_t *pu32Sec, uint32_t *pu32Nsec);
void EMAC_SetTime(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_UpdateTime(uint32_t u32Neg, uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_EnableAlarm(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_DisableAlarm(void);

uint32_t EMAC_CheckLinkStatus(void);

   

   

   







 
#line 674 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\uart.h"
 





 












 



 



 

 
 
 

#line 40 "..\\..\\..\\Library\\StdDriver\\inc\\uart.h"

 
 
 











 
 
 
















 
 
 




 
 
 




 
 
 






 
 
 
#line 106 "..\\..\\..\\Library\\StdDriver\\inc\\uart.h"


 
 
 




   




 












 













 













 












 













 













 














 












 













 













 













 













 













 






















 






















 




































 












 













 


 
static __inline void UART_CLEAR_RTS(UART_T* uart);
static __inline void UART_SET_RTS(UART_T* uart);










 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= ~(0x1ul << (1));
}










 
static __inline void UART_SET_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}


void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void UART_SetLineConfig(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);




   

   

   







 
#line 675 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"
 





 











 



 



 













 
#line 52 "..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"

 
#line 60 "..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"

   




 






 







 









 









 









 







 








 








 












 












 







 







 








 
#line 197 "..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"








 









 







 







 
















 







 










 












 












 










 










 












 












 









 








 








 








 


uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);


   

   

   







 
#line 676 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"
 





 











 



 



 





 
 
 






 
 
 







 
 
 



 
 
 




 
 
 





 
 
 






#line 98 "..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"















 
#line 233 "..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"


   




 














 















 














 















 















 















 















 
















 
































 











 












 











 


















 















 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);


   

   

   








 
#line 677 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\ccap.h"
 





 










 



 



 

 
 
 
#line 40 "..\\..\\..\\Library\\StdDriver\\inc\\ccap.h"

 
 
 
























#line 74 "..\\..\\..\\Library\\StdDriver\\inc\\ccap.h"

 
 
 






static uint32_t u32EscapeFrame = 0;
 
 
 





   





 








 















 









 


void CCAP_Open(uint32_t u32InFormat, uint32_t u32OutFormet);
void CCAP_SetCroppingWindow(uint32_t u32VStart,uint32_t u32HStart, uint32_t u32Height, uint32_t u32Width);
void CCAP_SetPacketBuf(uint32_t  u32Address );
void CCAP_Close(void);
void CCAP_EnableInt(uint32_t u32IntMask);
void CCAP_DisableInt(uint32_t u32IntMask);
void CCAP_Start(void);
void CCAP_Stop(uint32_t u32FrameComplete);
void CCAP_SetPacketScaling(uint32_t u32VNumerator, uint32_t u32VDenominator, uint32_t u32HNumerator, uint32_t u32HDenominator);
void CCAP_SetPacketStride(uint32_t u32Stride );
void CCAP_EnableMono(uint32_t u32Interface);
void CCAP_DisableMono(void);
void CCAP_EnableLumaYOne(uint32_t u32th);
void CCAP_DisableLumaYOne(void);

   



   

   







 
#line 678 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\ecap.h"
 





 











 



 



 





 
 
 
#line 42 "..\\..\\..\\Library\\StdDriver\\inc\\ecap.h"







 
 
 




#line 64 "..\\..\\..\\Library\\StdDriver\\inc\\ecap.h"






   



 














 








 















 












 












 















 












 












 








 








 








 








 








 








 















 
#line 260 "..\\..\\..\\Library\\StdDriver\\inc\\ecap.h"







 








 








 








 
















 













 

















 













 








 














 














 









 








 












 









 


void ECAP_Open(ECAP_T* ecap, uint32_t u32FuncMask);
void ECAP_Close(ECAP_T* ecap);
void ECAP_EnableINT(ECAP_T* ecap, uint32_t u32Mask);
void ECAP_DisableINT(ECAP_T* ecap, uint32_t u32Mask);
   

   

   







 
#line 679 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\qei.h"
 





 











 



 



 

 
 
 





 
 
 
#line 46 "..\\..\\..\\Library\\StdDriver\\inc\\qei.h"




   




 







 








 








 








 








 








 












 












 












 












 













 













 








 















 








 









 








 








 









 















 














 









 









 














 














 









 













 



void QEI_Close(QEI_T* qei);
void QEI_DisableInt(QEI_T* qei, uint32_t u32IntSel);
void QEI_EnableInt(QEI_T* qei, uint32_t u32IntSel);
void QEI_Open(QEI_T* qei, uint32_t u32Mode, uint32_t u32Value);
void QEI_Start(QEI_T* qei);
void QEI_Stop(QEI_T* qei);


   

   

   







 
#line 680 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\timer.h"
 





 











 



 



 
 
 
 
#line 37 "..\\..\\..\\Library\\StdDriver\\inc\\timer.h"






#line 49 "..\\..\\..\\Library\\StdDriver\\inc\\timer.h"

#line 56 "..\\..\\..\\Library\\StdDriver\\inc\\timer.h"

   




 














 













 












 














 


 
static __inline void TIMER_Start(TIMER_T *timer);
static __inline void TIMER_Stop(TIMER_T *timer);
static __inline void TIMER_EnableWakeup(TIMER_T *timer);
static __inline void TIMER_DisableWakeup(TIMER_T *timer);
static __inline void TIMER_StartCapture(TIMER_T *timer);
static __inline void TIMER_StopCapture(TIMER_T *timer);
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer);
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer);
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer);
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer);
static __inline void TIMER_EnableInt(TIMER_T *timer);
static __inline void TIMER_DisableInt(TIMER_T *timer);
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer);
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer);
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer);
static __inline void TIMER_ClearIntFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer);
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer);
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer);
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer);
static __inline void TIMER_ResetCounter(TIMER_T *timer);









 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}











 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}









 
static __inline void TIMER_StartCapture(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (3));
}









 
static __inline void TIMER_StopCapture(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (3));
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (6));
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (6));
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return ((timer->INTSTS & (0x1ul << (0))) ? 1UL : 0UL);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1UL : 0UL);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}









 
static __inline void TIMER_ResetCounter(TIMER_T *timer)
{
    timer->CNT = 0UL;
    while((timer->CNT&(0x1ul << (31))) == (0x1ul << (31)))
    {
        ;
    }
}


uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);

   

   

   








#line 681 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\timer_pwm.h"
 





 












 


 



 
 
 
 



 
 
 




 
 
 



 
 
 





 
 
 






 
 
 
#line 74 "..\\..\\..\\Library\\StdDriver\\inc\\timer_pwm.h"


#line 83 "..\\..\\..\\Library\\StdDriver\\inc\\timer_pwm.h"




 
 
 




 
 
 
#line 105 "..\\..\\..\\Library\\StdDriver\\inc\\timer_pwm.h"

 
 
 





 
 
 






 
 
 






   




 











 












 











 











 















 











 











 














 











 













 











 













 











 











 















 
















 
















 


















 




















 













 











 











 












 











 











 











 












 











 











 











 












 











 











 











 












 











 












 











 












 











 












 











 
















 



void TPWM_SetCounterClockSource(TIMER_T *timer, uint32_t u32CntClkSrc);
uint32_t TPWM_ConfigOutputFreqAndDuty(TIMER_T *timer, uint32_t u32Frequency, uint32_t u32DutyCycle);
void TPWM_EnableDeadTime(TIMER_T *timer, uint32_t u32DTCount);
void TPWM_EnableDeadTimeWithPrescale(TIMER_T *timer, uint32_t u32DTCount);
void TPWM_DisableDeadTime(TIMER_T *timer);
void TPWM_EnableCounter(TIMER_T *timer);
void TPWM_DisableCounter(TIMER_T *timer);
void TPWM_EnableTriggerADC(TIMER_T *timer, uint32_t u32Condition);
void TPWM_DisableTriggerADC(TIMER_T *timer);
void TPWM_EnableFaultBrake(TIMER_T *timer, uint32_t u32CH0Level, uint32_t u32CH1Level, uint32_t u32BrakeSource);
void TPWM_EnableFaultBrakeInt(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_DisableFaultBrakeInt(TIMER_T *timer, uint32_t u32IntSource);
uint32_t TPWM_GetFaultBrakeIntFlag(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_ClearFaultBrakeIntFlag(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_SetLoadMode(TIMER_T *timer, uint32_t u32LoadMode);
void TPWM_EnableBrakePinDebounce(TIMER_T *timer, uint32_t u32BrakePinSrc, uint32_t u32DebounceCnt, uint32_t u32ClkSrcSel);
void TPWM_DisableBrakePinDebounce(TIMER_T *timer);
void TPWM_EnableBrakePinInverse(TIMER_T *timer);
void TPWM_DisableBrakePinInverse(TIMER_T *timer);
void TPWM_SetBrakePinSource(TIMER_T *timer, uint32_t u32BrakePinNum);

   

   

   







#line 682 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"
 





 











 



 



 


 
 
 




 
 
 




 
 
 





 
 
 



#line 66 "..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"

 
 
 



 
 
 
#line 145 "..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"
 
 
 





   



 










 











 













 











 













 











 












 












 













 













 













 













 













 













 













 


 
 
 
void PDMA_Open(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_Close(PDMA_T * pdma);
void PDMA_SetTransferCnt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_DisableTimeout(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_SetTimeOut(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(PDMA_T * pdma,uint32_t u32Ch);
void PDMA_EnableInt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Mask);
void PDMA_SetStride(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32DestLen, uint32_t u32SrcLen, uint32_t u32TransCount);
void PDMA_SetRepeat(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32DestInterval, uint32_t u32SrcInterval, uint32_t u32RepeatCount);


   

   

   







 
#line 683 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\crypto.h"
 





 










 



 




 













#line 49 "..\\..\\..\\Library\\StdDriver\\inc\\crypto.h"






#line 65 "..\\..\\..\\Library\\StdDriver\\inc\\crypto.h"

#line 74 "..\\..\\..\\Library\\StdDriver\\inc\\crypto.h"

















typedef enum
{
     
    CURVE_P_192,                         
    CURVE_P_224,                         
    CURVE_P_256,                         
    CURVE_P_384,                         
    CURVE_P_521,                         
    CURVE_K_163,                         
    CURVE_K_233,                         
    CURVE_K_283,                         
    CURVE_K_409,                         
    CURVE_K_571,                         
    CURVE_B_163,                         
    CURVE_B_233,                         
    CURVE_B_283,                         
    CURVE_B_409,                         
    CURVE_B_571,                         
    CURVE_KO_192,                        
    CURVE_KO_224,                        
    CURVE_KO_256,                        
    CURVE_BP_256,                        
    CURVE_BP_384,                        
    CURVE_BP_512,                        
    CURVE_UNDEF,                         
}
E_ECC_CURVE;                             


   




 

 
 
 






 







 







 







 







 







 







 







 







 







 








 







 







 







 







 







 








 







 







 







 







 







 







 







 



   




 


 
 
 

void PRNG_Open(CRPT_T *crpt, uint32_t u32KeySize, uint32_t u32SeedReload, uint32_t u32Seed);
void PRNG_Start(CRPT_T *crpt);
void PRNG_Read(CRPT_T *crpt, uint32_t u32RandKey[]);
void AES_Open(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32EncDec, uint32_t u32OpMode, uint32_t u32KeySize, uint32_t u32SwapType);
void AES_Start(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32DMAMode);
void AES_SetKey(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32Keys[], uint32_t u32KeySize);
void AES_SetInitVect(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32IV[]);
void AES_SetDMATransfer(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32TransCnt);
void TDES_Open(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32EncDec, int32_t Is3DES, int32_t Is3Key, uint32_t u32OpMode, uint32_t u32SwapType);
void TDES_Start(CRPT_T *crpt, int32_t u32Channel, uint32_t u32DMAMode);
void TDES_SetKey(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32Keys[3][2]);
void TDES_SetInitVect(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32IVH, uint32_t u32IVL);
void TDES_SetDMATransfer(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32TransCnt);
void SHA_Open(CRPT_T *crpt, uint32_t u32OpMode, uint32_t u32SwapType, uint32_t hmac_key_len);
void SHA_Start(CRPT_T *crpt, uint32_t u32DMAMode);
void SHA_SetDMATransfer(CRPT_T *crpt, uint32_t u32SrcAddr, uint32_t u32TransCnt);
void SHA_Read(CRPT_T *crpt, uint32_t u32Digest[]);
void ECC_Complete(CRPT_T *crpt);
int  ECC_IsPrivateKeyValid(CRPT_T *crpt, E_ECC_CURVE ecc_curve,  char private_k[]);
int32_t  ECC_GeneratePublicKey(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[]);
int32_t  ECC_Mutiply(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char x1[], char y1[], char *k, char x2[], char y2[]);
int32_t  ECC_GenerateSecretZ(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[], char secret_z[]);
int32_t  ECC_GenerateSignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message, char *d, char *k, char *R, char *S);
int32_t  ECC_VerifySignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message, char *public_k1, char *public_k2, char *R, char *S);


   

   

   







 

#line 684 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\trng.h"
 





 










 



 




 

 
 
 


















 



   




 


 
 
 

void TRNG_Open(void);
int32_t TRNG_GenWord(uint32_t *u32RndNum);
int32_t TRNG_GenBignum(uint8_t u8BigNum[], int32_t i32Len);
int32_t TRNG_GenBignumHex(char cBigNumHex[], int32_t i32Len);


   

   

   







 

#line 685 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"
 





 










 



 




 


 
 
 
#line 51 "..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"











 
 
 





 
 
 



 
 
 
#line 95 "..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"






   




 


 
 
 

#line 127 "..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"

   




 

 
 
 

static __inline uint32_t FMC_ReadCID(void);
static __inline uint32_t FMC_ReadPID(void);
static __inline uint32_t FMC_ReadUID(uint8_t u8Index);
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index);
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
static __inline uint32_t FMC_GetVECMAP(void);








 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPSTS & (0x7ffful << (9)));
}






 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x0BUL;            
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = 0x0u;                          
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));           



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x0CUL;           
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = 0x04u;                        
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));          



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUID(uint8_t u8Index)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x04UL;
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT = 0u;
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = 0x1u;



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG) {}

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x04UL;             
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = (0x04u * u32Index) + 0x10u;     
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));            



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}   

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}








 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x2EUL;   
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = u32PageAddr;        
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = 0x1u;                



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG) {}              
}


 
 
 

extern void     FMC_Close(void);
extern int32_t  FMC_ConfigXOM(uint32_t xom_num, uint32_t xom_base, uint8_t xom_page);
extern int32_t  FMC_Erase(uint32_t u32PageAddr);
extern int32_t  FMC_Erase_SPROM(void);
extern int32_t  FMC_Erase_Block(uint32_t u32BlockAddr);
extern int32_t  FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t  FMC_EraseXOM(uint32_t xom_num);
extern int32_t  FMC_GetXOMState(uint32_t xom_num);
extern int32_t  FMC_GetBootSource(void);
extern void     FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern int32_t  FMC_Read_64(uint32_t u32addr, uint32_t * u32data0, uint32_t * u32data1);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void     FMC_SetBootSource(int32_t i32BootSrc);
extern void     FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t  FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t  FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len);
extern int32_t  FMC_Write_OTP(uint32_t otp_num, uint32_t low_word, uint32_t high_word);
extern int32_t  FMC_Read_OTP(uint32_t otp_num, uint32_t *low_word, uint32_t *high_word);
extern int32_t  FMC_Lock_OTP(uint32_t otp_num);
extern int32_t  FMC_Is_OTP_Locked(uint32_t otp_num);
extern int32_t  FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t  FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t  FMC_SetSPKey(uint32_t key[3], uint32_t kpmax, uint32_t kemax, const int32_t lock_CONFIG, const int32_t lock_SPROM);
extern int32_t  FMC_CompareSPKey(uint32_t key[3]);


   

   

   







 
#line 686 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\spim.h"
 





 



 
 
 









 



 




 






 
 
 
#line 50 "..\\..\\..\\Library\\StdDriver\\inc\\spim.h"







#line 65 "..\\..\\..\\Library\\StdDriver\\inc\\spim.h"

 

typedef enum
{
    MFGID_UNKNOW    = 0x00U,
    MFGID_SPANSION  = 0x01U,
    MFGID_EON       = 0x1CU,
    MFGID_ISSI      = 0x7FU,
    MFGID_MXIC      = 0xC2U,
    MFGID_WINBOND   = 0xEFU
}
E_MFGID;

 
#line 100 "..\\..\\..\\Library\\StdDriver\\inc\\spim.h"

 




 












 



 

 








 

   




 


 
 
 




 





 





 





 





 








 





 





 





 








 








 






 








 






 








 








 








 








 








 








 








 






 








 





 








 





 





 





 








 





 





 





 





 





 





 





 








 






 








 








 






 








 






 








 






 








 






 






 






 








 






 








 






 





 





 





 





 








 





 








 




 
 
 


int  SPIM_InitFlash(int clrWP);
uint32_t SPIM_GetSClkFreq(void);
void SPIM_ReadJedecId(uint8_t idBuf[], uint32_t u32NRx, uint32_t u32NBit);
int  SPIM_Enable_4Bytes_Mode(int isEn, uint32_t u32NBit);
int  SPIM_Is4ByteModeEnable(uint32_t u32NBit);

void SPIM_ChipErase(uint32_t u32NBit, int isSync);
void SPIM_EraseBlock(uint32_t u32Addr, int is4ByteAddr, uint8_t u8ErsCmd, uint32_t u32NBit, int isSync);

void SPIM_IO_Write(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t wrCmd, uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat);
void SPIM_IO_Read(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint8_t rdCmd, uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, int u32NDummy);

void SPIM_DMA_Write(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint32_t wrCmd);
void SPIM_DMA_Read(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint32_t u32RdCmd, int isSync);

void SPIM_EnterDirectMapMode(int is4ByteAddr, uint32_t u32RdCmd, uint32_t u32IdleIntvl);
void SPIM_ExitDirectMapMode(void);

void SPIM_SetQuadEnable(int isEn, uint32_t u32NBit);

   

   

   







 
#line 687 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"
 





 











 



 



 

 
 
 
#line 41 "..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"

 
 
 



 
 
 





   



 










 











 











 











 












 











 












 












 











 












 











 












 












 












 












 












 











 












 











 











 











 











 











 











 








 








 








 








 








 








 








 


 
 
 

 
static __inline void I2C_STOP(I2C_T *i2c);









 
static __inline void I2C_STOP(I2C_T *i2c)
{

    (i2c)->CTL0 |= ((0x1ul << (3)) | (0x1ul << (4)));
    while(i2c->CTL0 & (0x1ul << (4)))
    {
    }
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SMBusClearInterruptFlag(I2C_T *i2c, uint8_t u8SMBusIntFlag);
uint8_t I2C_WriteByte(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t I2C_WriteMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_ReadByte(I2C_T *i2c, uint8_t u8SlaveAddr);
uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t I2C_ReadMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_SMBusGetStatus(I2C_T *i2c);
void I2C_SMBusSetPacketByteCount(I2C_T *i2c, uint32_t u32PktSize);
void I2C_SMBusOpen(I2C_T *i2c, uint8_t u8HostDevice);
void I2C_SMBusClose(I2C_T *i2c);
void I2C_SMBusPECTxEnable(I2C_T *i2c, uint8_t u8PECTxEn);
uint8_t I2C_SMBusGetPECValue(I2C_T *i2c);
void I2C_SMBusIdleTimeout(I2C_T *i2c, uint32_t us, uint32_t u32Hclk);
void I2C_SMBusTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);
void I2C_SMBusClockLoTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);

   

   

   







 
#line 688 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"
 





 










 



 



 





 



 
#line 43 "..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"

 



 





 





 



 



 
#line 85 "..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"

#line 102 "..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"

 



 



   



 
 
 
 






 
static __inline void I2S_ENABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if((u32ChMask > 0U) && (u32ChMask < 9U))
    {
        i2s->CTL1 |= ((uint32_t)1U << (u32ChMask-1U));
    }
}







 
static __inline void I2S_DISABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if((u32ChMask > 0U) && (u32ChMask < 9U))
    {
        i2s->CTL1 &= ~((uint32_t)1U << (u32ChMask-1U));
    }
}






 







 







 







 







 







 







 







 







 







 







 







 










 
static __inline void I2S_SET_MONO_RX_CHANNEL(I2S_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->CTL0 |= (0x1ul << (23))) :
    (i2s->CTL0 &= ~(0x1ul << (23)));
}







 







 








 








 








 








 







 







 


void I2S_Close(I2S_T *i2s);
void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask);
void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);
void I2S_SetFIFO(I2S_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void I2S_ConfigureTDM(I2S_T *i2s, uint32_t u32ChannelWidth, uint32_t u32ChannelNum, uint32_t u32SyncWidth);
uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32MonoData, uint32_t u32DataFormat);

   


   

   






 

#line 689 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"
 





 











 



 



 
#line 35 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 




 
 
 



 
 
 





 
 
 





 
 
 
#line 84 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"






 
 
 
#line 102 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"

#line 112 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"




 
 
 
#line 129 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 







 
 
 



 
 
 





 
 
 




 
 
 
#line 170 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 
#line 182 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 







   




 







 








 








 








 















 










 
#line 269 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"








 










 









 









 












 
















 











 











 









 











 












 









 













 
#line 430 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"









 










 










 






























 
#line 505 "..\\..\\..\\Library\\StdDriver\\inc\\epwm.h"












 











 




 
 
 
uint32_t EPWM_ConfigCaptureChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableADCTrigger(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void EPWM_DisableADCTrigger(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearADCTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t EPWM_GetADCTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDACTrigger(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void EPWM_DisableDACTrigger(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDACTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t EPWM_GetDACTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableCapture(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableCapture(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnablePDMA(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode);
void EPWM_DisablePDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableCaptureInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void EPWM_DisableCaptureInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void EPWM_ClearCaptureIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t EPWM_GetCaptureIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAcc(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntFlagCnt, uint32_t u32IntAccSrc);
void EPWM_DisableAcc(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccPDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccPDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccStopMode(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccStopMode(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearFTDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetFTDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_DisableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_ConfigSyncPhase(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32SyncSrc, uint32_t u32Direction, uint32_t u32StartPhase);
void EPWM_EnableSyncPhase(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableSyncPhase(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableSyncNoiseFilter(EPWM_T *epwm, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void EPWM_DisableSyncNoiseFilter(EPWM_T *epwm);
void EPWM_EnableSyncPinInverse(EPWM_T *epwm);
void EPWM_DisableSyncPinInverse(EPWM_T *epwm);
void EPWM_SetClockSource(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
void EPWM_EnableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void EPWM_DisableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_EnableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_DisableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_SetBrakePinSource(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32SelAnotherModule);
void EPWM_SetLeadingEdgeBlanking(EPWM_T *epwm, uint32_t u32TrigSrcSel, uint32_t u32TrigType, uint32_t u32BlankingCnt, uint32_t u32BlankingEnable);
uint32_t EPWM_GetWrapAroundFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearWrapAroundFlag(EPWM_T *epwm, uint32_t u32ChannelNum);

   

   

   







 
#line 690 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"
 





 











 



 



 

 
 
 






 
 
 




#line 67 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"







 
 
 
#line 83 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"

   



 
 
 
 







 










 








 











 










 










 









 









 









 











 










 











 











 











 











 











 









 









 









 









 









 











 









 











 










 









 









 









 









 









 
















 
#line 438 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 463 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 488 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 513 "..\\..\\..\\Library\\StdDriver\\inc\\eadc.h"








 









 











 









 








 








 








 








 


 
 
 
void EADC_Open(EADC_T *eadc, uint32_t u32InputMode);
void EADC_Close(EADC_T *eadc);
void EADC_ConfigSampleModule(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerSrc, uint32_t u32Channel);
void EADC_SetTriggerDelayTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerDelayTime, uint32_t u32DelayClockDivider);
void EADC_SetExtendSampleTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime);

   

   

   







 
#line 691 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"
 





 











 



 



 
#line 35 "..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 




 
 
 



 
 
 





 
 
 





 
 
 
#line 75 "..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 



 
 
 



 
 
 



 
 
 






   




 














 










 









 









 








 








 












 













 










 








 











 








 












 









 






























 
#line 307 "..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"


 
 
 
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void BPWM_DisableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t BPWM_GetADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_DisableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_DisableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_SetClockSource(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
uint32_t BPWM_GetWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);


   

   

   







 
#line 692 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"
 





 











 



 



 
 
 
 
#line 39 "..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





 
 
 


   




 










 











 











 












 












 












 














 


 
static __inline void WDT_Close(void);
static __inline void WDT_EnableInt(void);
static __inline void WDT_DisableInt(void);









 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL = 0UL;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL |= (0x1ul << (6));
    return;
}









 
static __inline void WDT_DisableInt(void)
{
     
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL &= ~((0x1ul << (6)) | (0x1ul << (2)) | (0x1ul << (3)) | (0x1ul << (5)));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 693 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"
 





 











 



 



 
 
 
 
#line 47 "..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"

 
 
 


   




 










 











 












 












 











 














 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 694 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\opa.h"
 





 











 



 



 




   



 

 
 
 
static __inline int32_t OPA_Calibration(OPA_T *opa, uint32_t u32OpaNum, uint32_t u32ClockSel, uint32_t u32LevelSel);









 









 









 









 









 









 









 










 









 
















 
static __inline int32_t OPA_Calibration(OPA_T *opa,
                                        uint32_t u32OpaNum,
                                        uint32_t u32ClockSel,
                                        uint32_t u32RefVol)
{
    uint32_t u32CALResult;
    int32_t i32Ret = 0L;

    (opa)->CALCTL = (((opa)->CALCTL) & ~((0x3ul << (4)) << (u32OpaNum << 1)));
    (opa)->CALCTL = (((opa)->CALCTL) & ~((0x1ul << (16)) << (u32OpaNum))) | (((u32RefVol) << (16)) << (u32OpaNum));
    (opa)->CALCTL |= ((0x1ul << (0)) << (u32OpaNum));
    while((opa)->CALCTL & ((0x1ul << (0)) << (u32OpaNum))) {}

    u32CALResult = ((opa)->CALST >> ((u32OpaNum)*4U)) & ((0x1ul << (1))|(0x1ul << (2)));
    if (u32CALResult == 0U)
    {
        i32Ret = 0L;
    }
    else if (u32CALResult == (0x1ul << (1)))
    {
        i32Ret = -2L;
    }
    else if (u32CALResult == (0x1ul << (2)))
    {
        i32Ret = -1L;
    }
    else if (u32CALResult == ((0x1ul << (1))|(0x1ul << (2))))
    {
        i32Ret = -3L;
    }

    return i32Ret;
}






 






   

   

   







 
#line 695 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\crc.h"
 





 











 



 



 
 
 
 





 
 
 





 
 
 




   




 













 











 











 


void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
uint32_t CRC_GetChecksum(void);

   

   

   







 
#line 696 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"
 





 











 



 



 
 
 
 





 
 
 




 
 
 



 
 
 



 
 
 
#line 66 "..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"

#line 74 "..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"





   




 










 












 











 












 











 












 











 












 











 












 











 












 











 












 











 












 











 












 











 











 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 697 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\usci_i2c.h"
 





 











 



 



 

 
 
 
enum UI2C_MASTER_EVENT
{
    MASTER_SEND_ADDRESS = 10,     
    MASTER_SEND_H_WR_ADDRESS,     
    MASTER_SEND_H_RD_ADDRESS,     
    MASTER_SEND_L_ADDRESS,        
    MASTER_SEND_DATA,             
    MASTER_SEND_REPEAT_START,     
    MASTER_READ_DATA,             
    MASTER_STOP,                  
    MASTER_SEND_START             
};

 
 
 
enum UI2C_SLAVE_EVENT
{
    SLAVE_ADDRESS_ACK = 100,       
    SLAVE_H_WR_ADDRESS_ACK,        
    SLAVE_L_WR_ADDRESS_ACK,        
    SLAVE_GET_DATA,                
    SLAVE_SEND_DATA,               
    SLAVE_H_RD_ADDRESS_ACK,        
    SLAVE_L_RD_ADDRESS_ACK         
};

 
 
 





 
 
 



 
 
 



 
 
 
#line 89 "..\\..\\..\\Library\\StdDriver\\inc\\usci_i2c.h"

   




 











 











 











 











 












 












 












 











 











 











 











 

















 

















 

















 



uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T* ui2c, uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
uint8_t UI2C_WriteByte(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_ReadByte(UI2C_T *ui2c, uint8_t u8SlaveAddr);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t UI2C_ReadMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen);

   

   

   







 
#line 698 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\scuart.h"
 





 











 



 



 













   




 

 






 









 









 








 









 









 









 


 






 









 










 










 









 


 











 












 














 











 










 











 


void SCUART_Close(SC_T* sc);
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate);
uint32_t SCUART_Read(SC_T* sc, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t SCUART_SetLineConfig(SC_T* sc, uint32_t u32Baudrate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t  u32StopBits);
void SCUART_SetTimeoutCnt(SC_T* sc, uint32_t u32TOC);
void SCUART_Write(SC_T* sc,uint8_t pu8TxBuf[], uint32_t u32WriteBytes);

   

   

   







 
#line 699 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"
 





 











 



 



 
#line 34 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"

#line 45 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"


   




 


















 



















 








 
#line 109 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"








 
#line 126 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"







 
#line 142 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"







 
#line 158 "..\\..\\..\\Library\\StdDriver\\inc\\sc.h"






 








 









 


 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count);
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count);






 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (0x1ul << (30)))
    {
        ;
    }
     
    (sc)->CTL &= ~((0x7ul << (20)) | (0x1ul << (23)));

    if((u32Count) != 0UL)
    {
        while((sc)->CTL & (0x1ul << (30)))
        {
            ;
        }
        (sc)->CTL |= (((u32Count) - 1UL) << (20)) | (0x1ul << (23));
    }
}






 
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (0x1ul << (30)))
    {
        ;
    }
     
    (sc)->CTL &= ~((0x7ul << (16)) | (0x1ul << (19)));

    if((u32Count) != 0UL)
    {
        while((sc)->CTL & (0x1ul << (30)))
        {
            ;
        }
        (sc)->CTL |= (((u32Count) - 1UL) << (16)) | (0x1ul << (19));
    }

}


uint32_t SC_IsCardInserted(SC_T *sc);
void SC_ClearFIFO(SC_T *sc);
void SC_Close(SC_T *sc);
void SC_Open(SC_T *sc, uint32_t u32CardDet, uint32_t u32PWR);
void SC_ResetReader(SC_T *sc);
void SC_SetBlockGuardTime(SC_T *sc, uint32_t u32BGT);
void SC_SetCharGuardTime(SC_T *sc, uint32_t u32CGT);
void SC_StopAllTimer(SC_T *sc);
void SC_StartTimer(SC_T *sc, uint32_t u32TimerNum, uint32_t u32Mode, uint32_t u32ETUCount);
void SC_StopTimer(SC_T *sc, uint32_t u32TimerNum);
uint32_t SC_GetInterfaceClock(SC_T *sc);


   

   

   







 
#line 700 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\spi.h"
 





 











 



 



 













 
#line 52 "..\\..\\..\\Library\\StdDriver\\inc\\spi.h"

 
#line 62 "..\\..\\..\\Library\\StdDriver\\inc\\spi.h"


 





 



 





 



 




 





 



 



 
#line 111 "..\\..\\..\\Library\\StdDriver\\inc\\spi.h"

   




 







 








 








 








 








 








 









 









 









 








 









 








 








 








 








 










 








 








 









 









 








 








 


 
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask);
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask);
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch);









 
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0U))
    {
        i2s->I2SCTL |= (0x1ul << (16));
    }
    else
    {
        i2s->I2SCTL |= (0x1ul << (17));
    }
}









 
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0U))
    {
        i2s->I2SCTL &= ~(0x1ul << (16));
    }
    else
    {
        i2s->I2SCTL &= ~(0x1ul << (17));
    }
}







 








 








 








 








 








 








 








 








 








 








 








 











 
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->I2SCTL |= (0x1ul << (23))) :
    (i2s->I2SCTL &= ~(0x1ul << (23)));
}








 








 









 










 








 








 




 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void SPII2S_Close(SPI_T *i2s);
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask);
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask);
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock);
void SPII2S_DisableMCLK(SPI_T *i2s);
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);


   

   

   







 
#line 701 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"
 





 











 



 



 













 
#line 52 "..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"

 
#line 62 "..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"

   




 







 








 








 








 








 








 









 









 









 








 









 








 








 








 








 










 








 








 









 









 








 








 







 







 







 







 







 







 





 
uint32_t QSPI_Open(QSPI_T *qspi, uint32_t u32MasterSlave, uint32_t u32QSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void QSPI_Close(QSPI_T *qspi);
void QSPI_ClearRxFIFO(QSPI_T *qspi);
void QSPI_ClearTxFIFO(QSPI_T *qspi);
void QSPI_DisableAutoSS(QSPI_T *qspi);
void QSPI_EnableAutoSS(QSPI_T *qspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t QSPI_SetBusClock(QSPI_T *qspi, uint32_t u32BusClock);
void QSPI_SetFIFO(QSPI_T *qspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t QSPI_GetBusClock(QSPI_T *qspi);
void QSPI_EnableInt(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_DisableInt(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetIntFlag(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_ClearIntFlag(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetStatus(QSPI_T *qspi, uint32_t u32Mask);


   

   

   







 
#line 702 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\can.h"
 





 











 



 



 
 
 
 



 
 
 



 
 
 



   




 


 
typedef struct
{
    uint32_t  IdType;        
    uint32_t  FrameType;     
    uint32_t  Id;            
    uint8_t   DLC;           
    uint8_t   Data[8];       
} STR_CANMSG_T;



 
typedef struct
{
    uint8_t   u8Xtd;       
    uint8_t   u8Dir;       
    uint32_t  u32Id;       
    uint8_t   u8IdType;    
} STR_CANMASK_T;

   

 

 



 











 












 











 











 













 



 
 
 
uint32_t CAN_SetBaudRate(CAN_T *tCAN, uint32_t u32BaudRate);
uint32_t CAN_Open(CAN_T *tCAN, uint32_t u32BaudRate, uint32_t u32Mode);
void CAN_Close(CAN_T *tCAN);
void CAN_CLR_INT_PENDING_BIT(CAN_T *tCAN, uint8_t u32MsgNum);
void CAN_EnableInt(CAN_T *tCAN, uint32_t u32Mask);
void CAN_DisableInt(CAN_T *tCAN, uint32_t u32Mask);
int32_t CAN_Transmit(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_Receive(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_SetMultiRxMsg(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32MsgCount, uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsg(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsgAndMsk(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32IDType, uint32_t u32ID, uint32_t u32IDMask);
int32_t CAN_SetTxMsg(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_TriggerTxMsg(CAN_T  *tCAN, uint32_t u32MsgNum);
int32_t CAN_BasicSendMsg(CAN_T *tCAN, STR_CANMSG_T* pCanMsg);
int32_t CAN_BasicReceiveMsg(CAN_T *tCAN, STR_CANMSG_T* pCanMsg);
void CAN_EnterInitMode(CAN_T *tCAN, uint8_t u8Mask);
void CAN_EnterTestMode(CAN_T *tCAN, uint8_t u8TestMask);
void CAN_LeaveTestMode(CAN_T *tCAN);
uint32_t CAN_GetCANBitRate(CAN_T *tCAN);
uint32_t CAN_IsNewDataReceived(CAN_T *tCAN, uint8_t u8MsgObj);
void CAN_LeaveInitMode(CAN_T *tCAN);
int32_t CAN_SetRxMsgObjAndMsk(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint32_t u32idmask, uint8_t u8singleOrFifoLast);
int32_t CAN_SetRxMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast);
void CAN_WaitMsg(CAN_T *tCAN);
int32_t CAN_ReadMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8Release, STR_CANMSG_T* pCanMsg);

   

   

   







 
#line 703 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"
 





 











 



 



 
 
 
 



 
 
 





 
 
 
#line 53 "..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 
#line 64 "..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 





#line 80 "..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"












#line 100 "..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"





   




 


 
typedef struct
{
    uint32_t u32Year;            
    uint32_t u32Month;           
    uint32_t u32Day;             
    uint32_t u32DayOfWeek;       
    uint32_t u32Hour;            
    uint32_t u32Minute;          
    uint32_t u32Second;          
    uint32_t u32TimeScale;       
    uint32_t u32AmPm;            
} S_RTC_TIME_DATA_T;

   




 











 











 











 

















 












 












 












 
















 













 














 


 
static __inline void RTC_WaitAccessEnable(void);









 
static __inline void RTC_WaitAccessEnable(void)
{
    while((((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN & (0x1ul << (24))) == (0x1ul << (24)))
    {
    }

    if(!(((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->CSERVER & 0x1))
    {
         
        ((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN = 0x0000A965UL;
    }

     
    while((((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN & (0x1ul << (16))) == (uint32_t)0x0)
    {
    }
}

void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX10000);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
void RTC_EnableSpareAccess(void);
void RTC_DisableSpareRegister(void);
void RTC_StaticTamperEnable(uint32_t u32TamperSelect, uint32_t u32DetecLevel, uint32_t u32DebounceEn);
void RTC_StaticTamperDisable(uint32_t u32TamperSelect);
void RTC_DynamicTamperEnable(uint32_t u32PairSel, uint32_t u32DebounceEn, uint32_t u32Pair1Source, uint32_t u32Pair2Source);
void RTC_DynamicTamperDisable(uint32_t u32PairSel);
void RTC_DynamicTamperConfig(uint32_t u32ChangeRate, uint32_t u32SeedReload, uint32_t u32RefPattern, uint32_t u32Seed);

   

   

   







 
#line 704 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\usci_uart.h"
 





 












 



 



 

 
 
 





 
 
 







 
 
 
#line 58 "..\\..\\..\\Library\\StdDriver\\inc\\usci_uart.h"


   




 












 












 













 













 














 














 












 













 













 













 













 















 















 














 














 

















 

















 












 






















 












 














 













 












 



void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T*  uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T*  uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
uint32_t UUART_Write(UUART_T* uuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
void UUART_EnableFlowCtrl(UUART_T* uuart);
void UUART_DisableFlowCtrl(UUART_T* uuart);


   

   

   







 
#line 705 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\sdh.h"
 





 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 9 "..\\..\\..\\Library\\StdDriver\\inc\\sdh.h"












 



 




 






 



 






 
#line 59 "..\\..\\..\\Library\\StdDriver\\inc\\sdh.h"










   



 
typedef struct SDH_info_t
{
    unsigned int    CardType;        
    unsigned int    RCA;             
    unsigned char   IsCardInsert;    
    unsigned int    totalSectorN;    
    unsigned int    diskSize;        
    int             sectorSize;      
} SDH_INFO_T;                        

   

 
extern uint8_t g_u8R3Flag;
extern uint8_t volatile g_u8SDDataReadyFlag;
extern SDH_INFO_T SD0, SD1;
 



 












 












 

















 














 











 









 



void SDH_Open(SDH_T *sdh, uint32_t u32CardDetSrc);
uint32_t SDH_Probe(SDH_T *sdh);
uint32_t SDH_Read(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount);
uint32_t SDH_Write(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount);

uint32_t SDH_CardDetection(SDH_T *sdh);
void SDH_Open_Disk(SDH_T *sdh, uint32_t u32CardDetSrc);
void SDH_Close_Disk(SDH_T *sdh);


   

   

   






 
#line 706 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"
 





 











 



 



 
typedef struct s_usbd_info
{
    uint8_t *gu8DevDesc;             
    uint8_t *gu8ConfigDesc;          
    uint8_t **gu8StringDesc;         
    uint8_t **gu8HidReportDesc;      
    uint8_t *gu8BosDesc;             
    uint32_t *gu32HidReportSize;     
    uint32_t *gu32ConfigHidDescIdx;  

} S_USBD_INFO_T;   

extern const S_USBD_INFO_T gsInfo;

   






 



#line 65 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 
 




 
#line 84 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 97 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 



 



 
#line 117 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 







 


 

 
 
 














#line 165 "..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"
















   




 










 













 












 











 











 











 











 











 











 











 














 











 














 











 















 












 











 












 












 













 











 













 













 











 











 











 












 















 
static __inline void USBD_MemCopy(uint8_t dest[], uint8_t src[], uint32_t size)
{
    uint32_t volatile i=0ul;

    while(size--)
    {
        dest[i] = src[i];
        i++;
    }
}










 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (0x1ul << (1)));
            break;
        }
    }
}









 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(0x1ul << (1)));
            break;
        }
    }
}











 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (0x1ul << (1)));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;


typedef void (*VENDOR_REQ)(void);            
typedef void (*CLASS_REQ)(void);             
typedef void (*SET_INTERFACE_REQ)(uint32_t u32AltInterface);     
typedef void (*SET_CONFIG_CB)(void);        


 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);

   

   

   







 
#line 707 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\hsusbd.h"
 





 











 



 



 
 






#line 48 "..\\..\\..\\Library\\StdDriver\\inc\\hsusbd.h"

 
 





 
#line 67 "..\\..\\..\\Library\\StdDriver\\inc\\hsusbd.h"

 
#line 76 "..\\..\\..\\Library\\StdDriver\\inc\\hsusbd.h"


   



 


typedef struct HSUSBD_CMD_STRUCT
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;

} S_HSUSBD_CMD_T;  




typedef struct s_hsusbd_info
{
    uint8_t *gu8DevDesc;             
    uint8_t *gu8ConfigDesc;          
    uint8_t **gu8StringDesc;         
    uint8_t *gu8QualDesc;            
    uint8_t *gu8FullConfigDesc;      
    uint8_t *gu8HSOtherConfigDesc;   
    uint8_t *gu8FSOtherConfigDesc;   
    uint8_t **gu8HidReportDesc;      
    uint32_t *gu32HidReportSize;     
    uint8_t **gu8FSHidReportDesc;    
    uint32_t *gu32FSHidReportSize;   

} S_HSUSBD_INFO_T;  


   

 
extern uint32_t g_u32HsEpStallLock;
extern uint8_t g_hsusbd_Configured;
extern uint8_t g_hsusbd_ShortPacket;
extern uint8_t g_hsusbd_CtrlZero;
extern uint8_t g_hsusbd_UsbAddr;
extern uint32_t volatile g_hsusbd_DmaDone;
extern uint32_t g_hsusbd_CtrlInSize;
extern S_HSUSBD_INFO_T gsHSInfo;
extern S_HSUSBD_CMD_T gUsbCmd;
 



 

#line 159 "..\\..\\..\\Library\\StdDriver\\inc\\hsusbd.h"







 
static __inline void HSUSBD_MemCopy(uint8_t u8Dst[], uint8_t u8Src[], uint32_t u32Size)
{
    uint32_t i = 0ul;

    while (u32Size--)
    {
        u8Dst[i] = u8Src[i];
        i++;
    }
}





 
static __inline void HSUSBD_ResetDMA(void)
{
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACNT = 0ul;
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACTL = 0x80ul;
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACTL = 0x00ul;
}






 
static __inline void HSUSBD_SetEpBufAddr(uint32_t u32Ep, uint32_t u32Base, uint32_t u32Len)
{
    if (u32Ep == 0xfful)
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPBUFST = u32Base;
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPBUFEND   = u32Base + u32Len - 1ul;
    }
    else
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPBUFST = u32Base;
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPBUFEND = u32Base + u32Len - 1ul;
    }
}








 
static __inline void HSUSBD_ConfigEp(uint32_t u32Ep, uint32_t u32EpNum, uint32_t u32EpType, uint32_t u32EpDir)
{
    if (u32EpType == ((uint32_t)0x00000002ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000000ul));
    }
    else if (u32EpType == ((uint32_t)0x00000004ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000002ul));
    }
    else if (u32EpType == ((uint32_t)0x00000006ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000004ul));
    }

    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPCFG = (u32EpType|u32EpDir|((uint32_t)0x00000001ul)|(u32EpNum << 4));
}






 
static __inline void HSUSBD_SetEpStall(uint32_t u32Ep)
{
    if (u32Ep == 0xfful)
    {
        (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPCTL = (((uint32_t)0x00000002ul)));
    }
    else
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL & 0xf7ul) | ((uint32_t)0x00000010ul);
    }
}








 
static __inline void HSUSBD_SetStall(uint32_t u32EpNum)
{
    uint32_t i;

    if (u32EpNum == 0ul)
    {
        (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPCTL = (((uint32_t)0x00000002ul)));
    }
    else
    {
        for (i=0ul; i<12ul; i++)
        {
            if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
            {
                ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL & 0xf7ul) | ((uint32_t)0x00000010ul);
            }
        }
    }
}






 
static __inline void  HSUSBD_ClearEpStall(uint32_t u32Ep)
{
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = ((uint32_t)0x00000008ul);
}








 
static __inline void HSUSBD_ClearStall(uint32_t u32EpNum)
{
    uint32_t i;

    for (i=0ul; i<12ul; i++)
    {
        if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
        {
            ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL = ((uint32_t)0x00000008ul);
        }
    }
}







 
static __inline uint32_t HSUSBD_GetEpStall(uint32_t u32Ep)
{
    return (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL & ((uint32_t)0x00000010ul));
}









 
static __inline uint32_t HSUSBD_GetStall(uint32_t u32EpNum)
{
    uint32_t i;
    uint32_t val = 0ul;

    for (i=0ul; i<12ul; i++)
    {
        if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
        {
            val = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL & ((uint32_t)0x00000010ul));
            break;
        }
    }
    return val;
}


 
typedef void (*HSUSBD_VENDOR_REQ)(void);  
typedef void (*HSUSBD_CLASS_REQ)(void);  
typedef void (*HSUSBD_SET_INTERFACE_REQ)(uint32_t u32AltInterface);  

void HSUSBD_Open(S_HSUSBD_INFO_T *param, HSUSBD_CLASS_REQ pfnClassReq, HSUSBD_SET_INTERFACE_REQ pfnSetInterface);
void HSUSBD_Start(void);
void HSUSBD_ProcessSetupPacket(void);
void HSUSBD_StandardRequest(void);
void HSUSBD_UpdateDeviceState(void);
void HSUSBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void HSUSBD_CtrlIn(void);
void HSUSBD_CtrlOut(uint8_t pu8Buf[], uint32_t u32Size);
void HSUSBD_SwReset(void);
void HSUSBD_SetVendorRequest(HSUSBD_VENDOR_REQ pfnVendorReq);



   

   

   







 
#line 708 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\otg.h"
 






 











 



 




 



 
 
 






   




 

 
 
 








 








 









 








 








 








 








 








 










 










 





















 





















 





















 





















 














 




   

   

   









 
#line 709 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\..\\..\\Library\\StdDriver\\inc\\hsotg.h"
 






 











 



 




 



 
 
 






   




 

 
 
 








 








 









 








 








 








 








 








 










 










 





















 





















 





















 





















 














 




   

   

   









 
#line 710 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"








#line 12 "..\\..\\..\\Library\\Device\\Nuvoton\\M480\\Include\\NuMicro.h"




#line 33 "..\\Source\\Debug\\clicmd.c"
#line 1 "..\\Source\\Debug\\clicmd.h"




























 




 
#line 1 "..\\Source\\Debug\\console.h"




























 




 

 




#line 48 "..\\Source\\Debug\\console.h"

enum LLID_Ind 
{
    LL_UART,
    LL_TELNET,
    LL_MAX
};

enum _CONSOLE_STATE_IND {
    CLI_STATE_BLOCK=0,
    CLI_STATE_LOGIN,
    CLI_STATE_PASSWD,
    CLI_STATE_COMMAND,
    CLI_STATE_COMMAND_WAIT,	
    CLI_STATE_PASSWD_CHANGE1,
    CLI_STATE_PASSWD_CHANGE2,
    CLI_STATE_PASSWD_CHANGE3,
    CLI_STATE_MAX
};

 

typedef struct
{
	unsigned char AddIndex;
	unsigned char GetIndex;
	char Buf[10][512];
} CONSOLE_CmdHistory;


 
struct _CLILINK
{
	struct _CLILINK *pPre;
	struct _CLILINK *pNext;
	unsigned short WaitTime;
	unsigned short ReplyLen;
	unsigned char Buf[100];
};
typedef struct _CLILINK tsCLILINK;

typedef struct
{
    unsigned char (*PutChar)(unsigned char c);  
    char (*GetChar)(void);      
    unsigned char Privilege;
    unsigned char State;
    unsigned char PromptEnable;
    unsigned char LowLayerId;
    unsigned short BufIndex;
    unsigned short CursorPosition;	
    unsigned short Argc;
    char **Argv;
    char CmdBuf[512];
    char UserName[16];
    char Passwd[16];
    char PasswdNew[16];
    char PromptStr[16];
	unsigned char CmdId;
	tsCLILINK Cmd; 

    CONSOLE_CmdHistory CmdHistory;

	void *pCmdTable;
	unsigned short CmdTableSize;	
	unsigned short Timer;		
} CONSOLE_Inst;

typedef int (*CmdPtr)(CONSOLE_Inst *pInst, int argc, char **argv);
typedef int (*HelpPtr)(CONSOLE_Inst *pInst);

typedef struct
{
    char Cmd[24];
    CmdPtr CmdFunc;
    HelpPtr Help;
    unsigned char Level;
} CONSOLE_CmdEntry;

typedef struct
{
    char Name[16];
    char Passwd[16];
    unsigned char Level;
} CONSOLE_Account;

 

  
void CONSOLE_Init(void);
void CONSOLE_Task(void);
short CONSOLE_PutMessage(CONSOLE_Inst *pInst, char *fmt, ...);
short CONSOLE_ChangeUsername(CONSOLE_Inst *pInst, unsigned char *username);



short CONSOLE_TimeTick(void);



 
#line 36 "..\\Source\\Debug\\clicmd.h"

 







 

 
extern CONSOLE_Account CLICMD_userTable[];

  
unsigned short CLICMD_GetCmdTableSize(void);
void* CLICMD_GetCmdTable(void);



 
#line 34 "..\\Source\\Debug\\clicmd.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 35 "..\\Source\\Debug\\clicmd.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 36 "..\\Source\\Debug\\clicmd.c"
#line 1 "..\\Source\\Debug\\test.h"




























 




 
#line 36 "..\\Source\\Debug\\test.h"

 












	 














 
typedef enum{
	TEST_IDLE = 0,
	TEST_ROUND_START,
	TEST_BUSY,	
	TEST_RUN,
	TEST_RUN_WAIT,
	TEST_CHANGE_CONDITION,
	TEST_ROUND_DONE,
	TEST_EXIT,
	TEST_ERROR,	
	TEST_DONE,
} TEST_STATE;

typedef enum{
	TEST_PATTERN_FIXED = 0,
	TEST_PATTERN_INCREMENT,
	TEST_PATTERN_DECREMENT,	
	TEST_PATTERN_RANDOM,
	TEST_MAX_PATTERN_TYPE,	
} TEST_PATTERN_TYPE;

typedef enum
{
	TEST_SPI_DATA_SIZE_AUTO = 0,
	TEST_SPI_1BYTE_DATA     = 1,
	TEST_SPI_2BYTE_DATA     = 2,
	TEST_SPI_4BYTE_DATA     = 4,	
} TEST_SPI_DATA_SIZE_TYPE;

typedef enum
{
	TEST_SPI_ADDR_SIZE_AUTO = 0,	
	TEST_SPI_2BYTE_ADDR     = 2,
	TEST_SPI_3BYTE_ADDR     = 3,
} TEST_SPI_ADDR_SIZE_TYPE;

typedef struct{
  TEST_SPI_DATA_SIZE_TYPE  DataSizeInWrite;
  TEST_SPI_DATA_SIZE_TYPE  DataSizeInRead;	
  TEST_SPI_ADDR_SIZE_TYPE  AddrSize;		
} TEST_SPIS_PARAMETER;

typedef struct{
	uint32_t	LoopCount;
	uint32_t	LogUpdateTime;
	uint8_t   DashboardEnabled;
	
	 
	uint32_t  StartAddress;
	uint32_t  EndAddress;	
	uint32_t  PatternType;
	uint32_t  Range;
	uint8_t   InitData[128];	
	uint32_t	InitDataLen;
	
	TEST_SPIS_PARAMETER	Spis;
} TEST_PARAMETER;

typedef struct{
	uint32_t Second;
	uint32_t Minute;
	uint32_t Hour;
	uint32_t Day;
	uint32_t RunFlag;
	uint32_t TickCnt;	
	
	uint32_t DownCounter[6];	
} TEST_TIME;

typedef struct{
	uint32_t RoundCnt;
	uint32_t OkCnt;
	uint32_t ErrorCnt;
	uint8_t  *pErrMsg;
	
} TEST_RECORD;

typedef struct{
	uint32_t DataSizeInWrite;
	uint32_t DataSizeInRead;	
	uint32_t AddrSizeInWrite;
	uint32_t AddrSizeInRead;
} TEST_SPIS_TEMP;

typedef struct{
	uint32_t  Type;
	uint8_t 	Data[128];	
	uint32_t	DataLen;	
	uint32_t	Seed;		
} TEST_PATTERN_TEMP;

typedef struct{
	 
	uint32_t	CtrlFlags;	
	uint8_t  	HostInterfaceErrorStatus;	
	uint32_t	RandomCaseCounter[5];
	
	 	
	uint8_t	  TxBuf[1024];
	uint32_t	TxLen;	
	uint8_t	  RxBuf[1024];
	uint32_t	RxLen;
	uint32_t	RemainLen;
	uint32_t	AddrOffset;
	TEST_PATTERN_TEMP Pattern;
	
	 	
	TEST_SPIS_TEMP Spis;
	
} TEST_TEMP;

typedef struct{
	void           *pDbgObj;
	uint32_t				State;	
	uint32_t				Terminated;
	TEST_RECORD			Record;
	TEST_PARAMETER	Parameter;
	TEST_TEMP				Temp;	
	
} TEST_CONTROL;

 

 
void TEST_Init(void);

void TEST_Timer(void const *argument);
void TEST_ResetTime(void);
void TEST_CtrlTime(uint8_t start);
TEST_TIME* TEST_GetTime(void);

int32_t TEST_PatternInit(TEST_PATTERN_TEMP *pPattern, uint8_t PatternType, uint8_t *pInitData, uint32_t InitDataLen);
int32_t TEST_PatternGenerator(TEST_PATTERN_TEMP *pPattern, uint8_t *pBuf, uint32_t Len);

	 
int16_t TEST_PdiMemoryTest(TEST_CONTROL *pCtrl);



 
#line 37 "..\\Source\\Debug\\clicmd.c"
#line 1 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"




























 




 






 




 
#line 48 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
#line 49 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
#line 1 "..\\For_SSC_Tool\\Src\\esc.h"



 




 














 





 
#line 1 "..\\For_SSC_Tool\\Src\\ecat_def.h"



 






 








 
#line 22 "..\\For_SSC_Tool\\Src\\ecat_def.h"
#line 23 "..\\For_SSC_Tool\\Src\\ecat_def.h"





 








 







 






 






 





 





 






 






 





 





 





 






 







 






 





 





 





 





 





 







 







 





 






 






 





 





 





 






 





 






 






 





 





 






 






 








 





 






 





 







 





 







 







 





 





 





 





 






 





 






 






 






 





 





 





 





 









 


 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 





 






 






 






 






 






 






 






 





 





 






 






 






 





 





 





 






 






 





 





 






 






 





 





 





 





 





 





 





 





 





 





 





 





 





 





 









 


 





 







 





 





 





 





 





 





 





 





 





 





 






 






 






 






 





 





 





 





 





 





 





 





 






 





 





 






 






 





 










 




#line 32 "..\\For_SSC_Tool\\Src\\esc.h"








 








 



































 


 


 
#line 102 "..\\For_SSC_Tool\\Src\\esc.h"
 














#line 123 "..\\For_SSC_Tool\\Src\\esc.h"









 
 
typedef struct 
{

  unsigned short                        PhysicalStartAddress;  
  unsigned short                        Length;  


      unsigned char                         Settings[4];  



 
#line 154 "..\\For_SSC_Tool\\Src\\esc.h"



 




 







 



}
TSYNCMAN;




 
#line 50 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 
	 


#line 1 "..\\Source\\Debug\\printd.h"




























 




 
#line 1 "..\\Source\\Debug\\ax_uart.h"




























 




 
#line 36 "..\\Source\\Debug\\ax_uart.h"

 
#line 44 "..\\Source\\Debug\\ax_uart.h"

#line 52 "..\\Source\\Debug\\ax_uart.h"

 
typedef struct{
	UART_T*                  Instance;
	
	unsigned char            RxBuf[512];
	volatile unsigned long   RxWrPtr;
	volatile unsigned long   RxRdPtr;
	volatile unsigned long   RxBufFree;
	
	unsigned char            TxBuf[1024];
	volatile unsigned long   TxWrPtr;
	volatile unsigned long   TxRdPtr;
	volatile unsigned long   TxBufFree;	
} AX_UART_OBJECT;
																								
 
  
int AX_UART_Init(void);
int AX_UART_DeInit(void);
char AX_UART_GetChar(void);
unsigned char AX_UART_PutChar(unsigned char);
int AX_UART_CheckTxBufEmpty(void);


 
#line 36 "..\\Source\\Debug\\printd.h"

 



 
																								
 

  
short printd(const char *fmt, ...);
void printdCtrl(char enb);



 
#line 57 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"




	 


	 


	 


	 
#line 77 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
	 


	 






 
#line 103 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

	 






	 





	 





	 
#line 138 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

#line 145 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

	 
#line 153 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 
typedef union
{
	unsigned char	Byte[2];
	unsigned short	Word;
} UALEVENT;

typedef union
{
	unsigned long	d32[2];
	struct{
		unsigned long vendor_id:      32;  
		unsigned long reserved_63_32: 32;  
	} b;
} oIC_VENDOR_ID;

typedef union
{
	unsigned long	d32[2];
	struct{
		unsigned long chip_revision:	 8;   
		unsigned long package_type:	 4;	  
		unsigned long product_id:		 20;  
		unsigned long reserved_63_32: 32;  
	} b;
} oIC_PRODUCT_ID;

#line 195 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

typedef struct{
	unsigned long	EscIsrCnt;
	unsigned long	FunIsrCnt;
	unsigned long	Sync0IsrCnt;
	unsigned long	Sync1IsrCnt;
	unsigned long	AdcTrgCnt;
	unsigned long	TIM1IsrCnt;
	unsigned long	EncIsrCnt;
	unsigned long	TmrTskIsrCnt;
} HW_DEBUG;

typedef struct{
	SPI_T*		pInst;
	GPIO_T*		pCsPort;
	unsigned long		CsPin;
	volatile unsigned char	Lock;
	unsigned long		ReentryCnt;
	unsigned char		ThreeByteAddrMode;
	unsigned char		TxBuf[(4 + 512)];
	unsigned char		RxBuf[(4 + 512)];
} HW_SPI_OBJECT;

 

 


 






 






 






 






 






 




 




 






 





 




 




 




#line 321 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"



 




 




 






 





 




 




 




#line 386 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 

 unsigned char HW_Init(void);
 void HW_Release(void);
 unsigned short HW_GetALEventRegister(void);


 unsigned short HW_GetALEventRegister_Isr(void);


 void HW_EscRead( unsigned long * pData, unsigned short Address, unsigned short Len );

 void HW_EscReadIsr( unsigned long *pData, unsigned short Address, unsigned short Len );


 void HW_EscWrite( unsigned long *pData, unsigned short Address, unsigned short Len );


 void HW_EscWriteIsr( unsigned long *pData, unsigned short Address, unsigned short Len );



 void HW_RestartTarget(void);






 unsigned long HW_GPIO_PinToIntrNum(unsigned long Pin);
 long HW_MultiFuncPins(GPIO_T* Instance, unsigned long Pins, unsigned char MultiFuncValue);
 void HW_TMR_ClkSource(TIMER_T* Instance, unsigned long NewState);
 void HW_SPI_ClkSource(SPI_T* Instance, unsigned char NewState);
 void HW_UART_ClkSource(UART_T* Instance, unsigned char NewState);
 void HW_GPIO_WritePin(GPIO_T* Instance, unsigned long Pin, unsigned char NewState);
 void HW_GPIO_TogglePin(GPIO_T* Instance, unsigned long Pin);
 unsigned char HW_GPIO_ReadPin(GPIO_T* Instance, unsigned long Pin);


unsigned char HW_CheckTimeout(unsigned short StartTime, unsigned short Timeout);
HW_DEBUG* HW_GetDebugCounter(void);



 
 
#line 38 "..\\Source\\Debug\\clicmd.c"
#line 1 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




























 
 



 






 

 








 
#line 54 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\For_SSC_Tool\\Src\\objdef.h"



 




 






















 





 
#line 1 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 




 



















 






 
#line 1 "..\\For_SSC_Tool\\Src\\ecatcoe.h"



 




 












 





 
#line 1 "..\\For_SSC_Tool\\Src\\mailbox.h"



 




 
















 





 
#line 34 "..\\For_SSC_Tool\\Src\\mailbox.h"









 
#line 50 "..\\For_SSC_Tool\\Src\\mailbox.h"

#line 59 "..\\For_SSC_Tool\\Src\\mailbox.h"
















 





 
#line 91 "..\\For_SSC_Tool\\Src\\mailbox.h"




 
typedef struct 
{
    unsigned short                          Length;  
    unsigned short                          Address;  

    unsigned short                          Flags[1];  
#line 108 "..\\For_SSC_Tool\\Src\\mailbox.h"
}
TMBXHEADER;









 
typedef struct 
{
    TMBXHEADER                      MbxHeader;  
    unsigned short                          Data[((0x0080 - 6) >> 1)];  
}
TMBX;




 
typedef struct
{
    unsigned short    firstInQueue;  
    unsigned short    lastInQueue;  
    unsigned short    maxQueueSize;  
    TMBX  * queue[(10)+1];  
} TMBXQUEUE;










 






extern unsigned char                    bReceiveMbxIsLocked;  
extern unsigned char                    bSendMbxIsFull;  
extern unsigned char                    bMbxRunning;  
extern unsigned char                    bMbxRepeatToggle;  
extern unsigned short                  u16SendMbxSize;  
extern unsigned short                  u16ReceiveMbxSize;  
extern unsigned short                  u16EscAddrReceiveMbx;  
extern unsigned short                  u16EscAddrSendMbx;  
extern unsigned char                   u8MbxWriteCounter;  
extern unsigned char                   u8MbxReadCounter;  
extern unsigned char                   u8MailboxSendReqStored;  
extern TMBX  *           psWriteMbx;  
extern TMBX  *           psReadMbx;  
extern TMBX  *           psRepeatMbx;  
extern TMBX  *           psStoreMbx;  
extern TMBXQUEUE         sMbxSendQueue;  
extern TMBXQUEUE         sMbxReceiveQueue;  






 

extern   void     MBX_Init(void);
extern   unsigned short    MBX_StartMailboxHandler(void);
extern   void     MBX_StopMailboxHandler(void);
extern   void     MBX_MailboxWriteInd(TMBX  *pMbx);
extern   void     MBX_MailboxReadInd(void);
extern   void     MBX_MailboxRepeatReq(void);
extern   unsigned char    MBX_MailboxSendReq(TMBX  * pMbx, unsigned char flags);
extern   void     MBX_CheckAndCopyMailbox(void);
extern   unsigned char    MBX_CopyToSendMailbox(TMBX  *pMbx);
extern   void     MBX_Main(void);


 
#line 30 "..\\For_SSC_Tool\\Src\\ecatcoe.h"








 



 






 
#line 58 "..\\For_SSC_Tool\\Src\\ecatcoe.h"





 





typedef unsigned short TCOEHEADER;  




 
typedef struct 
{
  TMBXHEADER        MbxHeader;  
  TCOEHEADER        CoeHeader;  
  unsigned short            Data[(((0x0080 - 6))-(2)) >> 1];  
}
TCOEMBX;














 
extern    TMBX  *  pCoeSendStored;                



 





 

extern   void     COE_Init(void);
extern   unsigned char    COE_ServiceInd(TCOEMBX  *pCoeMbx);
extern   unsigned char     COE_ContinueInd(TMBX  * pMbx);


 
#line 38 "..\\For_SSC_Tool\\Src\\sdoserv.h"









 




 



 





 
#line 73 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 
#line 89 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 


 
typedef struct 
{
    unsigned short Sdo[2];  
 
#line 114 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 
}
TINITSDOHEADER;







 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDOMBX;






 
 









 
typedef struct 
{
    unsigned short      SegHeader;  
#line 163 "..\\For_SSC_Tool\\Src\\sdoserv.h"
    unsigned short      Data[((((0x0080 - 6))-(( 2 + 1 )) - 1 ) >> 1)];   
}
TSDOSEGHEADERDATA;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                Data[(4) >> 1];  
}
TINITSDODOWNLOADEXPREQMBX;



 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                CompleteSize[2];  
  unsigned short                Data[((((0x0080 - 6))-(( 2 + 4 + 4 ))) >> 1)];  
}
TINITSDODOWNLOADNORMREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDODOWNLOADRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TSDOSEGHEADERDATA     SdoHeader;  
}
TDOWNLOADSDOSEGREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  unsigned char                 SegHeader;  
}
TDOWNLOADSDOSEGRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDOUPLOADREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                Data[((4) >> 1)];  
}
TINITSDOUPLOADEXPRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                CompleteSize[2];  
  unsigned short                Data[((((0x0080 - 6))-(( 2 + 4 + 4 ))) >> 1)];  
}
TINITSDOUPLOADNORMRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  unsigned char                 SegHeader;  
}
TUPLOADSDOSEGREQMBX;



 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TSDOSEGHEADERDATA     SdoHeader;  
}
TUPLOADSDOSEGRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned long             AbortCode;  
}
TABORTSDOTRANSFERREQMBX;



 





 
#line 343 "..\\For_SSC_Tool\\Src\\sdoserv.h"






 
#line 380 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 
 





 
#line 396 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 


 
typedef struct 
{
    unsigned short                ListType;  
#line 416 "..\\For_SSC_Tool\\Src\\sdoserv.h"
}
TSDOINFOLIST;




 
typedef struct 
{
    unsigned short            DataType;  
    unsigned short            ObjFlags;  

     










}
TSDOINFOOBJDESC;







 
typedef struct 
{
    unsigned short            Index;  
    TSDOINFOOBJDESC   Res;  
}
TSDOINFOOBJ;







 
typedef struct 
{
    unsigned short                DataType;  
    unsigned short                BitLength;  
    unsigned short                ObjAccess;  
                                     









 

#line 497 "..\\For_SSC_Tool\\Src\\sdoserv.h"
}
TSDOINFOENTRYDESC;




 
typedef struct 
{
    unsigned short                Index;  
    unsigned short                 Info;  





    TSDOINFOENTRYDESC    Res;  
}
TSDOINFOENTRY;




 
typedef struct 
{
    unsigned long                ErrorCode;  
}
TSDOINFOERROR;







 
typedef struct 
{
    unsigned short InfoHead;  





    unsigned short                FragmentsLeft;  

    union 
    {
        TSDOINFOLIST    List; 
        TSDOINFOOBJ     Obj; 
        TSDOINFOENTRY   Entry; 
        TSDOINFOERROR   Error; 
        unsigned short          Data[1]; 
    }
    Data; 

}
TSDOINFOHEADER;




 
typedef struct 
{
  TMBXHEADER        MbxHeader;  
  TCOEHEADER        CoeHeader;  
  TSDOINFOHEADER    SdoHeader;  
}
TSDOINFORMATION;


#line 579 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 







 




 








 









 
extern unsigned char         u8PendingSdo;  
extern unsigned char          bStoreCompleteAccess;  
extern unsigned char         u8StoreSubindex;  
extern unsigned short        u16StoreIndex;  
extern unsigned long        u32StoreDataSize;  
extern unsigned short  *pStoreData;  
extern unsigned char (* pSdoPendFunc)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
 


extern unsigned long                      aSdoInfoHeader[(((((( (6) + (2) + (4) ))+(2)))+3) >> 2)];  





 
extern unsigned short  *                 pSdoSegData;  
extern unsigned short                          nSdoInfoFragmentsLeft;  
 






 
extern    unsigned char SDOS_SdoInfoInd(TSDOINFORMATION  *pSdoInfoInd);
extern    unsigned char SDOS_SdoInd(TINITSDOMBX  *pSdoInd);

extern    void  SDOS_SdoRes(unsigned char abort, unsigned long objLength, unsigned short  *pData);
 
extern    void  SODS_ClearPendingResponse(void);
 



 
#line 40 "..\\For_SSC_Tool\\Src\\objdef.h"










 



 
#line 119 "..\\For_SSC_Tool\\Src\\objdef.h"




 

#line 135 "..\\For_SSC_Tool\\Src\\objdef.h"









 
typedef struct OBJ_ENTRY
{
    struct OBJ_ENTRY                      *pPrev;  
    struct OBJ_ENTRY                      *pNext;  

    unsigned short                                Index;  
    TSDOINFOOBJDESC                       ObjDesc;  
    const TSDOINFOENTRYDESC      *pEntryDesc;  
    const unsigned char                  *pName;  
    void                            *pVarPtr;  
    unsigned char (* Read)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
    unsigned char (* Write)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
    unsigned short                                 NonVolatileOffset;  
}
TOBJECT;




 
typedef struct 
{
    unsigned short    subindex0; 
    unsigned short    u16SyncType;  
    unsigned long    u32CycleTime; 
    unsigned long    u32Subindex003; 
    unsigned short    u16SyncTypesSupported; 
    unsigned long    u32MinCycleTime; 
    unsigned long    u32CalcAndCopyTime; 
    unsigned long    u32Si7Reserved; 
    unsigned short    u16GetCycleTime; 
    unsigned long    u32DelayTime;  
    unsigned long    u32Sync0CycleTime;  
    unsigned short    u16SmEventMissedCounter;  
    unsigned short    u16CycleExceededCounter;  
    unsigned short    u16Si13Reserved;  
    unsigned short    u16Si14Reserved;  
    unsigned long    u32Si15Reserved;  
    unsigned long    u32Si16Reserved;  
    unsigned long    u32Si17Reserved;  
    unsigned long    u32Si18Reserved;  
    unsigned char    u8SyncError;  
}
TSYNCMANPAR;



 
typedef struct 
{
    unsigned short    syncFailedCounter;  
}
TCYCLEDIAG;




 
typedef struct  {
   unsigned short   u16SubIndex0;  
   unsigned long   u32LocalErrorReaction;  
   unsigned short   u16SyncErrorCounterLimit;  
} 
TOBJ10F1;








 









 
extern unsigned char bSyncSetByUser;



 
extern TCYCLEDIAG sCycleDiag;




 
extern TSYNCMANPAR  sSyncManOutPar;




 
extern TSYNCMANPAR  sSyncManInPar;








 
extern TOBJ10F1 sErrorSettings



;




 
extern    char          aSubindexDesc[13]



;






 

extern    const TOBJECT  *  OBJ_GetObjectHandle( unsigned short index );
extern    unsigned long  OBJ_GetObjectLength( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned char bCompleteAccess);
extern    unsigned short  OBJ_GetNoOfObjects(unsigned char listType);
extern    unsigned short  OBJ_GetObjectList(unsigned short listType, unsigned short *pIndex, unsigned short size, unsigned short  *pData,unsigned char *pAbort);
extern    unsigned short  OBJ_GetDesc( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned short  * pData );
extern    const TSDOINFOENTRYDESC  * OBJ_GetEntryDesc(const TOBJECT  * pObjEntry, unsigned char Subindex);
extern    const TSDOINFOOBJDESC  * OBJ_GetObjDesc(const TOBJECT  * pObjEntry);
extern    unsigned short  OBJ_GetEntryOffset(unsigned char subindex, const TOBJECT  * pObjEntry);
extern    unsigned char   CheckSyncTypeValue(unsigned short index, unsigned short NewSyncType);
extern    unsigned char   OBJ_Read(unsigned short index, unsigned char subindex, unsigned long objSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    unsigned char   OBJ_Write(unsigned short index, unsigned char subindex, unsigned long dataSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    void    COE_WriteBackupEntry(unsigned char subindex, const TOBJECT  * pObjEntry);


 
#line 55 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\For_SSC_Tool\\Src\\ecatappl.h"



 




 



















 








 





 

#line 47 "..\\For_SSC_Tool\\Src\\ecatappl.h"









 

 
#line 85 "..\\For_SSC_Tool\\Src\\ecatappl.h"












 



extern unsigned char bEcatWaitForInputUpdate;  
extern unsigned char bEtherCATRunLed;  
extern unsigned char bEtherCATErrorLed;  
extern unsigned char bRunApplication;  







 




extern    void       ECAT_CheckTimer(void);
extern    void       PDI_Isr(void);
extern    void       Sync0_Isr(void);
extern    void       Sync1_Isr(void);
extern    void       ECAT_Application(void);
extern    void       PDO_ResetOutputs(void);
extern    void       PDO_ReadInputs(void);
extern    void       PDO_InputMapping(void);

extern    void       CalcSMCycleTime(void);
extern    unsigned short     ESC_EepromAccess(unsigned long wordaddress, unsigned short wordsize, unsigned short  *pData, unsigned char access);
extern    unsigned short     ESC_EepromWriteCRC(void);





 
#line 56 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 57 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\Source\\McInterface\\mc.h"




























 




 
#line 1 "..\\Source\\McInterface\\mc_cfg.h"




























 




 
#line 1 "..\\Source\\McInterface\\mc_types.h"




























 




 
#line 36 "..\\Source\\McInterface\\mc_types.h"

 


 
typedef enum
{
	MCSTS_BUSY      = 1,
	MCSTS_OK				= 0,
	MCSTS_ERR       = -1,
} MC_STATUS_E;

typedef enum
{
	MCFS_NO_ERR     	= 0x0000,
	MCFS_OVER_VOLT  	= 0x0001,
	MCFS_UNDER_VOLT 	= 0x0002,
	MCFS_OVER_TEMP  	= 0x0004,
	MCFS_PHASE_ERR  	= 0x0008,
	MCFS_OVER_CURR  	= 0x0010,
	MCFS_SW_ERR     	= 0x0020,
	MCFS_VEC_ALI_ERR	= 0x0040,
	MCFS_ENC_ALI_ERR	= 0x0080,
} MC_FAULT_E;

typedef enum
{
	MC_STOP = 0,
	MC_START,
} MC_STSP_E;

typedef enum
{
	MC_DISABLE = 0,
	MC_ENABLE,
} MC_EADA_E;

typedef enum
{
	MC_FORWARD = 0,
	MC_BACKWARD,
} MC_DIR_E;

 



 

 




 
#line 36 "..\\Source\\McInterface\\mc_cfg.h"

 








#line 1 "..\\Source\\McStack\\param_def.h"




























 




 
extern int mapMemory[];

#line 50 "..\\Source\\McStack\\param_def.h"





 
#line 62 "..\\Source\\McStack\\param_def.h"

#line 73 "..\\Source\\McStack\\param_def.h"

#line 90 "..\\Source\\McStack\\param_def.h"

 
#line 102 "..\\Source\\McStack\\param_def.h"














 
#line 125 "..\\Source\\McStack\\param_def.h"


typedef enum
{
	LC_ALL_OFF	    = 0,
	LC_RESET        = 1,
	LC_PWM_CTRL		= 2,
	LC_AMP_CTRL	    = 3,
	LC_SPD_CTRL	    = 4,
	LC_POS_CTRL	    = 5,
	LC_PP_CTRL	    = 6,
} LOOP_CTRL_E;

#line 145 "..\\Source\\McStack\\param_def.h"

 
#line 154 "..\\Source\\McStack\\param_def.h"






#line 167 "..\\Source\\McStack\\param_def.h"






 
#line 184 "..\\Source\\McStack\\param_def.h"

#line 195 "..\\Source\\McStack\\param_def.h"

 
#line 208 "..\\Source\\McStack\\param_def.h"

#line 220 "..\\Source\\McStack\\param_def.h"

 
#line 232 "..\\Source\\McStack\\param_def.h"

#line 240 "..\\Source\\McStack\\param_def.h"

#line 251 "..\\Source\\McStack\\param_def.h"

#line 259 "..\\Source\\McStack\\param_def.h"

 
#line 271 "..\\Source\\McStack\\param_def.h"

#line 280 "..\\Source\\McStack\\param_def.h"

#line 291 "..\\Source\\McStack\\param_def.h"

#line 300 "..\\Source\\McStack\\param_def.h"

 
#line 315 "..\\Source\\McStack\\param_def.h"

#line 329 "..\\Source\\McStack\\param_def.h"

 
#line 338 "..\\Source\\McStack\\param_def.h"

#line 346 "..\\Source\\McStack\\param_def.h"

 
#line 356 "..\\Source\\McStack\\param_def.h"







#line 371 "..\\Source\\McStack\\param_def.h"







 
#line 389 "..\\Source\\McStack\\param_def.h"

#line 400 "..\\Source\\McStack\\param_def.h"

 
#line 409 "..\\Source\\McStack\\param_def.h"

#line 417 "..\\Source\\McStack\\param_def.h"

 
#line 430 "..\\Source\\McStack\\param_def.h"

#line 442 "..\\Source\\McStack\\param_def.h"


 






#line 457 "..\\Source\\McStack\\param_def.h"

 
#line 466 "..\\Source\\McStack\\param_def.h"

 
#line 474 "..\\Source\\McStack\\param_def.h"






 

 




 

#line 47 "..\\Source\\McInterface\\mc_cfg.h"

#line 50 "..\\Source\\McInterface\\mc_cfg.h"





 






 


 
#line 74 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 84 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 92 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 108 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 120 "..\\Source\\McInterface\\mc_cfg.h"

 






 








 

 

 



 
#line 36 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McInterface\\mc_math.h"




























 




 
#line 36 "..\\Source\\McInterface\\mc_math.h"

 

 
typedef struct
{
	int32_t Alpha;
	int32_t Beta;
} MCMATH_AB_PHASE_T;

typedef struct
{
	int32_t Direct;
	int32_t Quadrature;
} MCMATH_DQ_PHASE_T;

 

 

 
static int MI_GetSIN(int d);
MCMATH_AB_PHASE_T MI_Clake(void);
void MI_Park(MCMATH_AB_PHASE_T *pAB);
MCMATH_AB_PHASE_T MI_RevPark(void);

void MI_MapUvwBlkInit(void);
void MI_CalcRotorVector(void);
void MI_MapAngBlkInit(void);
void MI_GetEncAngle_AbzMode(void);
void MI_CalcEncAngle(uint32_t ResetAngle);

void MI_FocCurrController(void);


 
#line 37 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\bus_vol_sensor.h"




























 




 
#line 36 "..\\Source\\McStack\\bus_vol_sensor.h"
#line 1 "..\\Source\\Mchal\\McHal_eadc.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_eadc.h"

 

 
typedef struct
{
	EADC_T		*pInst;
	GPIO_T		*pPort[5];
	IRQn_Type	IrqType;
	uint32_t	Channel[5];
	uint8_t		MultiFuncValue;
	uint32_t	SampleModuleMask[5];
	int32_t		ConvBuffer[5];
} MH_AdcRegConfig_t;

 

 

 
int32_t MH_AdcRegInit(MH_AdcRegConfig_t *pHandle, uint16_t v);
void 		MH_AdcRegDeInit(MH_AdcRegConfig_t *pHandle);
uint16_t 	MH_AdcConv(MH_AdcRegConfig_t *pHandle, uint16_t v);
void 		MH_AdcStop(MH_AdcRegConfig_t *pHandle, uint16_t v);




 

#line 37 "..\\Source\\McStack\\bus_vol_sensor.h"

 

 
typedef struct
{
    MH_AdcRegConfig_t	    VbusRegister;
    uint16_t			    ConversionFactor;
    uint16_t			    FaultState;
    uint8_t				    ChannelNum;
} MS_Vbus_Handle_t;

 

 

 
void MS_MapAdcBlkInit(void);
int32_t MS_GetBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetAvBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetBusVoltage_V(MS_Vbus_Handle_t *pHandle);
uint16_t MS_CheckVbus(MS_Vbus_Handle_t *pHandle);



 
#line 38 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\temp_sensor.h"




























 




 
#line 36 "..\\Source\\McStack\\temp_sensor.h"
#line 37 "..\\Source\\McStack\\temp_sensor.h"

 

 
typedef struct{
	MH_AdcRegConfig_t   TempRegister;
	int32_t             MCP9700V0C;         
	int32_t             MCP9700TC;          
	uint16_t	        FaultState;
	uint8_t             ChannelNum;
} MS_Temp_Handle_t;

 

 

 
uint16_t MS_GetTemp_d(MS_Temp_Handle_t *pHandle);
uint16_t MS_GetTemp_V( MS_Temp_Handle_t * pHandle );
uint16_t MS_CheckTemp( MS_Temp_Handle_t * pHandle );


 
#line 39 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\curr_fdbk.h"




























 




 
#line 36 "..\\Source\\McStack\\curr_fdbk.h"
#line 37 "..\\Source\\McStack\\curr_fdbk.h"

 

 
typedef struct{
	MH_AdcRegConfig_t       CurrentRegister;
	uint16_t			    FaultState;
	uint8_t                 ChannelNum;
}MS_Curr_Handle_t;

 

 

 
MS_Curr_Handle_t	MS_GetCurr_d(MS_Curr_Handle_t *pHandle);
MS_Curr_Handle_t	MS_GetSignedCurr_d(MS_Curr_Handle_t *pHandle);



 
#line 40 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\svpwm.h"




























 




 
#line 36 "..\\Source\\McStack\\svpwm.h"
#line 37 "..\\Source\\McStack\\svpwm.h"
#line 1 "..\\Source\\Mchal\\McHal_epwm.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_epwm.h"

 



 
typedef struct {
	EPWM_T    *pInst;
	IRQn_Type  IrqType;
	uint32_t   PreemptPrio;
	uint32_t   SubPrio;
	GPIO_T    *pPortPwm;
	uint32_t   PinPwmU;
	uint32_t   PinPwmV;
	uint32_t   PinPwmW;
	uint8_t    MultiFuncValue;
	uint32_t   EpwmChU;
	uint32_t   EpwmChV;
	uint32_t   EpwmChW;
} MH_EpwmRegConfig_t;

 










 

 
int32_t MH_EpwmRegInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmRegDeInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmPutU(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutV(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutW(MH_EpwmRegConfig_t *pHandle , uint32_t value);

 

#line 38 "..\\Source\\McStack\\svpwm.h"

 

 
typedef enum
{
	SCM_DISABLE 			= 0,
	SCM_VECTOR_ALIGN	= 1,
	SCM_MANUAL_CTRL 	= 2,
	SCM_AUTO_CTRL 		= 9,
} SVP_CTRL_MODE_E;

typedef struct
{
	MH_EpwmRegConfig_t EpwmRegister;
	uint8_t Sector;                     
} MS_Epwm_Handle_t;

 

 

 
void MS_MapPwmBlkInit(void);
void MS_SetPhaseVoltage(MS_Epwm_Handle_t *pHandle, MCMATH_AB_PHASE_T Vab);




 
#line 41 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\incremental_fdbk.h"




























 




 
#line 36 "..\\Source\\McStack\\incremental_fdbk.h"
#line 1 "..\\Source\\Mchal\\McHal_qei.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_qei.h"

 

 
typedef struct
{
	QEI_T	    *pInst;
	GPIO_T	    *pPortIncEnc;
	uint32_t	PinA;
	uint32_t	PinB;
	uint32_t	PinZ;
	uint8_t		MultiFuncValue;
	GPIO_T	    *pPortWireCtrl;
	uint32_t	PinWireCtrl;
} MH_QeiRegConfig_t;

 



 

 
int32_t MH_QeiRegInit(MH_QeiRegConfig_t *pHandle);
void MH_QeiRegDeInit(MH_QeiRegConfig_t *pHandle);
int32_t MH_QeiGetCount(MH_QeiRegConfig_t *pHandle);
int32_t MH_QeiGetCntLatch(MH_QeiRegConfig_t *pHandle);



 
#line 37 "..\\Source\\McStack\\incremental_fdbk.h"

 

 

typedef struct{
	MH_QeiRegConfig_t   QeiRegister;
	uint32_t            AlignRequest;
} MS_IncremEnc_Handle_t;
 

 

 
void MS_MapAbzBlkInit(void);
void MS_GetAbzCount(MS_IncremEnc_Handle_t *pHandle);


 
#line 42 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\stepper_mode.h"




























 




 
#line 36 "..\\Source\\McStack\\stepper_mode.h"

 




 
typedef enum
{
	STPM_IDLE		        = 0,
	STPM_START			    = 1,
	STPM_FORWARD		    = 10,
	STPM_BACKWARD		    = 20,
	STPM_ZERO_VECTOR	    = 30,
	STPM_DONE			    = 99,
} STPM_STATE_E;

typedef enum
{
	STPMVA_IDLE = 0,
	STPMVA_SETUP,
	STPMVA_START,
	STPMVA_WAIT_ALIGN,
	STPMVA_DONE,
	STPMVA_FAIL,
} STPM_VECALI_STATE_E;

typedef struct
{
	 
	STPM_STATE_E   			State;
	volatile uint32_t       TickMs[3];
	int32_t				    IsSingleTurn;
	int32_t				    StpAng;

	 
	STPM_VECALI_STATE_E	    VecAliState;
	int32_t 			    RotorAngT[2];
	int32_t 			    RotorStableCnt;
} STPM_CTRL_T;

 

 

 
MC_STATUS_E STPM_Init(STPM_CTRL_T *pStpMod);
void STPM_TickRun(STPM_CTRL_T *pStpMod);
STPM_STATE_E STPM_RunStepping(STPM_CTRL_T *pStpMod);
STPM_VECALI_STATE_E STPM_VectorAlignment(STPM_CTRL_T *pStpMod);



 
#line 43 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\flash.h"




























 




 
#line 36 "..\\Source\\McStack\\flash.h"

 



 
typedef struct
{
	uint8_t	  Signature[4];
	uint16_t	ImageType;
	uint16_t	Compress;
	uint32_t	ContentOffset;
	uint32_t	FileLen;
	uint32_t	Reserved0;
	uint32_t	Checksum32;	
	uint32_t	Reserved1[2];
} MS_Firmware_Header_t;

typedef struct{
    MS_Firmware_Header_t FwHeader;
    int32_t FileSize;
    int32_t FileSize_div_4;
    int32_t BaseAddr;
    int32_t BufferLen;
    uint8_t Buffer[256];
    uint32_t CalcChecksum32;
} MS_Flash_Handle_t;
 

 

 
void MS_GetFlashParamsAreaData(void);
static void doCommand(int last);
static int MS_GetLineData(void);
static int MS_CalcCheckSum(int base);
static int MS_CheckSignature(uint8_t *p);
static uint32_t MS_Swap(uint32_t d);
static void MS_ReadFlashRom(int addr,int n,uint8_t *buf);


 
#line 44 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\loop_control.h"




























 




 
#line 36 "..\\Source\\McStack\\loop_control.h"

 
void MS_MapCmdBlkInit(void);

#line 45 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\pid.h"




























 




 
#line 36 "..\\Source\\McStack\\pid.h"

 

 
typedef struct{
    int32_t *Cmd;
    int32_t *Feedback;
    int32_t *Kp;
    int32_t *Ki;
    int32_t KpDiv;
    int32_t KiDiv;
    int32_t *Error;
    int32_t *Integraler;
    int32_t *ErrorLimit;
    int32_t *IntegralLimit;
    int32_t *OutputLimit;
    int32_t *Division;
    int32_t *Output;
} MS_PI_Handle_t;

typedef enum{
    Pos = 0,
    Spd,
    Amp,
} Control_Loop_e;

 

 

 
void MS_MapPosBlkInit(void) ;
void MS_MapSpdBlkInit(void);
void MS_MapAmpBlkInit(void) ;
void MS_PiController(MS_PI_Handle_t *pHandle, int32_t type) ;



 

#line 46 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\servo_commu.h"




























 




 
#line 36 "..\\Source\\McStack\\servo_commu.h"
#line 1 "..\\Source\\Mchal\\McHal_husb.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_husb.h"

 





 
#line 50 "..\\Source\\Mchal\\McHal_husb.h"

 
#line 60 "..\\Source\\Mchal\\McHal_husb.h"

 









#line 78 "..\\Source\\Mchal\\McHal_husb.h"

#line 87 "..\\Source\\Mchal\\McHal_husb.h"





 
typedef enum
{
	VCPSTE_NOT_INIT           = -1,	
	VCPSTE_RECV_CMD           = 0,
	VCPSTE_PARSE_CMD          = 1,
	VCPSTE_EXEC_CMD           = 2,
	VCPSTE_PREPARE_END_RESP   = 9,	
	VCPSTE_WAIT_END_RESP      = 10,	
} MH_VCP_STATE_E;

typedef struct
{
	int flgRxReady; 
	int flgTxReady;
	int RxPnt; 
	int TxPnt;
	int RxLength; 
	int TxLength;
	uint8_t RxBuffer[16384];
	uint8_t TxBuffer[4096];
} MH_VCP_OBJECT_T;

 

 

 
void MH_VCP_Init(void);
void MH_VCP_Run(void);



 
#line 37 "..\\Source\\McStack\\servo_commu.h"

 
typedef enum
{
	SC_MAP_CMD    = 2,
	SC_DAS_CMD,
} SC_CMD_TYPE_E;

 
typedef struct
{    
	int  code; 
	char name[8];
} cmd_t;
 

 
extern int mapMemory[];

 
int esc_doCommand(char *,int,char *,char *,int,int);


 
#line 47 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\sw_protection.h"




























 




 
#line 36 "..\\Source\\McStack\\sw_protection.h"
#line 1 "..\\Source\\Mchal\\McHal_gpio.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_gpio.h"

 

 
typedef struct {
	GPIO_T		*pPortGateDriveEnb;
	uint32_t	PinGateDriveEnb;
	GPIO_T		*pPortPwmWdtTrg;
	uint32_t	PinPwmWdtTrg;
	GPIO_T		*pPortMotorBrake;	
	uint32_t	PinMotorBrake;	
} MH_SwProtRegConfig_t;

 

 

 
int32_t MH_SwProtRegInit(MH_SwProtRegConfig_t *pHandle);
void MH_SwProtRegDeInit(MH_SwProtRegConfig_t *pHandle);
void MH_DebugHwInit(void);


 
#line 37 "..\\Source\\McStack\\sw_protection.h"
#line 38 "..\\Source\\McStack\\sw_protection.h"
#line 39 "..\\Source\\McStack\\sw_protection.h"
#line 40 "..\\Source\\McStack\\sw_protection.h"

 

 
typedef enum
{
	SAT_NO_ERR			    = 0,
	SAT_UNDER_VOLT			= 1,
	SAT_OVER_VOLT		    = 2,
	SAT_OVER_TEMP		    = 3,
	SAT_PHASE_ERR       	= 4,
	SAT_OVER_CURR_ID		= 5,
	SAT_OVER_CURR_IQ		= 6,
} SPT_ALARM_TYPE_E;

typedef struct {
	MH_SwProtRegConfig_t    SwProtRegister;
	MS_Vbus_Handle_t        *pVbusInfo;
	MS_Temp_Handle_t        *pTempInfo;
	MS_Curr_Handle_t        *pCurrInfo;
} MS_SwProt_Handle_t;

 

 

 
void MS_MapProtectBlkInit(void);
void MS_GetAvBusVoltage_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvTemp_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvI2T_V(MS_SwProt_Handle_t *pHandle);
void SWP_Run(MS_SwProt_Handle_t *pHandle);
void SWP_DriveCtrl(MS_SwProt_Handle_t *pHandle);



 
#line 48 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\trajectory_ctrl.h"




























 




 
#line 36 "..\\Source\\McStack\\trajectory_ctrl.h"

 

 
typedef enum
{
	TCM_START		= 0,
	TCM_RUN  		= 1,
	TCM_GET_SPD	= 2,	
} TC_MODE_E;

typedef enum
{
	TCC_NO_CMD		= 0,
	TCC_ABS_POS   = 1,
	TCC_REL_POS		= 2,	
	TCC_RESTART   = 3,	
	TCC_BRAKE    	= 9,
} TC_COMMAND_E;

typedef enum
{
	TCS_ADJ_SPD 	= 0,
	TCS_ACC_SPD   = 1,
	TCS_CONST_SPD = 2,	
	TCS_DEC_SPD   = 3,	
	TCS_STOP    	= 9,
} TC_STATE_E;

typedef enum
{
	TCD_FORWARD 	= 1,
	TCD_BACKWARD  = -1,
} TC_DIRECTION_E;

 

 

 
void TC_Init(void);  
void TC_Run(void);
void TC_Brake(void);



 

#line 49 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\das.h"




























 




 
#line 36 "..\\Source\\McStack\\das.h"

 





 
typedef enum
{
	DASSTE_IDLE    = 0,
	DASSTE_START,
	DASSTE_RUN,
	DASSTE_DONE,	
} DAS_STATE_E;

 

 
extern int dasMemory[];

 
void DAS_Init(void);
DAS_STATE_E DAS_Run(void);



 
#line 50 "..\\Source\\McInterface\\mc.h"

 
#line 58 "..\\Source\\McInterface\\mc.h"

 
typedef enum
{
	MCSTE_IDLE = 0,
	MCSTE_TUNE,
	MCSTE_FAULT,
	MCSTE_WAIT_CLR_ERR,
	MCSTE_STOP,
	MCSTE_START_VEC_ALIGN,
	MCSTE_VEC_ALIGN,
	MCSTE_START_ENC_ALIGN,
	MCSTE_ENC_ALIGN,
	MCSTE_STARTUP,
	MCSTE_RUN,
	MCSTE_SHUTDOWN,
} MC_STATE_E;

typedef enum
{
	MCCMD_NO_CMD = 0,
	MCCMD_START,
	MCCMD_ENC_ALIGN,
	MCCMD_SET_POS,
	MCCMD_STOP,
	MCCMD_FAULT_RESET,
} MC_CMD_E;

typedef struct
{

	MC_STATE_E   				State;
	volatile MC_CMD_E		    Cmd;
	MC_FAULT_E   				FaultFlags;
	volatile uint32_t           TickMs[3];

	 
	STPM_CTRL_T					StpMod;
	 
	int32_t					    HomingReach;
	int32_t					    HomingVelocity;
	 
	int32_t					    VelocityCmd;
	 
	int32_t					    PositionCmd;
	uint32_t				    ConvFactor_mDegToTick;
} MC_T;

 
typedef struct
{
    MS_Vbus_Handle_t VbusInfo;
} MI_Vbus_Handle_t;

typedef struct
{
    MS_Temp_Handle_t TempInfo;
} MI_Temp_Handle_t;

typedef struct
{
    MS_Curr_Handle_t    CurrInfo;
    MCMATH_AB_PHASE_T   Iab;
} MI_Curr_Handle_t;

typedef struct
{
    MS_IncremEnc_Handle_t IncremEncInfo;
} MI_IncremEnc_Handle_t;

typedef struct
{
    MS_Epwm_Handle_t PwmInfo;
    MCMATH_AB_PHASE_T Vab;
} MI_Pwm_Handle_t;

typedef struct
{
    MS_SwProt_Handle_t SwProtInfo;
} MI_SwProt_Handle_t;

typedef struct
{
    MS_PI_Handle_t PiInfo;
} MI_Pi_Handle_t ;

 

 

 
extern MC_T MotorCtrl;
extern MI_Vbus_Handle_t BusVoltageParamsM1;
extern MI_Temp_Handle_t TemperatureParamsM1;
extern MI_Curr_Handle_t CurrentParamsM1;
extern MI_IncremEnc_Handle_t IncremEncParamsM1;
extern MI_Pwm_Handle_t PwmParamsM1;
extern MI_SwProt_Handle_t SwProtParamsM1;
extern MI_Pi_Handle_t PosParamsM1;
extern MI_Pi_Handle_t SpdParamsM1;
extern MI_Pi_Handle_t AmpParamsM1;
extern MI_Pi_Handle_t AmpParamsM2;

void MI_IncremEncModuleInit(void);
void MI_PwmModuleInit(void);
void MI_AbsoluteEncModuleInit(void);
void MI_SwProtModuleInit(void);

 
	 
int32_t  		MC_Init(void);
void     		MC_DeInit(void);
void     		MC_TickRun(void);
MC_STATE_E	MC_TaskRun(void);
void        MC_StartMotor(void);
void        MC_StopMotor(void);
MC_STATE_E  MC_GetState(void);
	 
int32_t     MC_GetActualTorque(void);
	 
void        MC_SetTargetVelocity(int32_t Velocity);
int32_t     MC_GetActualVelocity(void);
	 
void        MC_ClearActualPosition(void);
void				MC_SetTargetPosition(int32_t Position, int32_t Velocity);
int32_t     MC_GetActualPosition(void);
	 
void        MC_FaultReset(void);
MC_FAULT_E	MC_GetFaultStatus(void);
	 
void				MC_StartHoming(int32_t TargetVelocity);
int32_t 		MC_CheckHomingDone(void);



 
#line 58 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\Source\\CiA402\\cia402.h"




























 




 
#line 36 "..\\Source\\CiA402\\cia402.h"

 


 
typedef enum{
	CIA402_EMUL_DISABLED = 0,
	CIA402_ACTUAL_IS_TARGET_POS,
} CIA402_EMUL_MODE;

typedef union {
  uint32_t d32;
  struct{
    uint32_t pp:               1;   
    uint32_t vl:               1;   
    uint32_t pv:               1;   
    uint32_t tq:               1;   		
    uint32_t reserved_4:       1;   				
    uint32_t hm:               1;   						
    uint32_t ip:               1;   
    uint32_t csp:              1;   
    uint32_t csv:              1;   		
    uint32_t cst:              1;   				
    uint32_t cstca:            1;   						
    uint32_t reserved_31_11:  21;   
  } b;
} OBJ0x6502_SUPPORTED_DRIVE_MODES;

typedef union {
  uint32_t d32;
  struct{
    uint32_t axis_is_act:      1;   
    uint32_t brake_applied:    1;   
    uint32_t low_pwr_applied:  1;   
    uint32_t high_pwr_applied: 1;   		
    uint32_t axis_func_enb:    1;   				
    uint32_t cfg_allowed:      1;   						
    uint32_t reserved_31_6:   26;   
  } b;
} CIA402_FLAGS;

typedef struct{
	void *pCtrlObj;
  int32_t (*FuncStart)(void *pAxis);
  int32_t (*FuncStop)(void *pAxis);
  int32_t (*FuncRun)(void *pAxis);		
} CIA402_FUNC_INTF;

 
  
int32_t CIA402_Init(void);
int32_t CIA402_DeInit(void);
int32_t CIA402_EmulationControl(uint8_t mode);
uint8_t CIA402_CheckEmulationEnable(void);
uint8_t CIA402_GetHallStatus(void);



 
#line 59 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"





 


 
#line 76 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 89 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 103 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 116 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 129 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 

















 
#line 240 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"

#line 269 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"

#line 311 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"






#line 329 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
	






 










 












#line 1 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 




 







 








 




 
#line 61 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
unsigned long SI4;  
unsigned long SI5;  
} 
TOBJ1600;




 
extern TOBJ1600 RxPdoMappingCspCsv0x1600



;
 





 




 
#line 120 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1601;




 
extern TOBJ1601 RxPdoMappingCsp0x1601



;
 





 




 
#line 177 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1602;




 
extern TOBJ1602 RxPdoMappingCsv0x1602



;
 





 




 
#line 240 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
unsigned long SI4;  
unsigned long SI5;  
} 
TOBJ1A00;




 
extern TOBJ1A00 TxPdoMappingCspCsv0x1A00



;
 





 




 
#line 299 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1A01;




 
extern TOBJ1A01 TxPdoMappingCsp0x1A01



;
 





 




 
#line 356 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1A02;




 
extern TOBJ1A02 TxPdoMappingCsv0x1A02



;
 





 




 
#line 407 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned short aEntries[1];   
} 
TOBJ1C12;




 
extern TOBJ1C12 sRxPDOassign



;
 





 




 
#line 456 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned short aEntries[1];   
} 
TOBJ1C13;




 
extern TOBJ1C13 sTxPDOassign



;
 





 




 
#line 498 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short ErrorCode0x603F



;
 





 




 
#line 529 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short Controlword0x6040



;
 





 




 
#line 560 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short Statusword0x6041



;
 





 




 
#line 591 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long VlRampFunctionTime0x604F



;
 





 




 
#line 622 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short QuickStopOptionCode0x605A



;
 





 




 
#line 653 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short ShutdownOptionCode0x605B



;
 





 




 
#line 684 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short DisableOperationOptionCode0x605C



;
 





 




 
#line 715 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short FaultReactionOptionCode0x605E



;
 





 




 
#line 746 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char ModesOfOperation0x6060



;
 





 




 
#line 777 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char ModesOfOperationDisplay0x6061



;
 





 




 
#line 808 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long PositionActualValue0x6064



;
 





 




 
#line 839 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long VelocityActualValue0x606C



;
 





 




 
#line 870 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TorqueActualValue0x6077



;
 





 




 
#line 901 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TargetPosition0x607A



;
 





 




 
#line 932 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long HomeOffset0x607C



;
 





 




 
#line 973 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
long MinPositionLimit;  
long MaxPositionLimit;  
} 
TOBJ607D;




 
extern TOBJ607D SoftwarePositionLimit0x607D



;
 





 




 
#line 1016 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long QuickStopDeceleration0x6085



;
 





 




 
#line 1047 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char HomingMethod0x6098



;
 





 




 
#line 1088 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SpeedDuringSearchForSwitch;  
unsigned long SpeedDuringSearchForZero;  
} 
TOBJ6099;




 
extern TOBJ6099 HomingSpeed0x6099



;
 





 




 
#line 1131 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long HomingAcceleration0x609A



;
 





 




 
#line 1172 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned char InterpolationTimePeriodValue;  
char InterpolationTimeIndex;  
} 
TOBJ60C2;




 
extern TOBJ60C2 InterpolationTimePeriod0x60C2



;
 





 




 
#line 1225 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
char SupportHomingMethod1st;  
char SupportHomingMethod2st;  
} 
TOBJ60E3;




 
extern TOBJ60E3 SupportedHomingMode0x60E3



;
 





 




 
#line 1268 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long DigitalInputs0x60FD



;
 





 




 
#line 1299 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TargetVelocity0x60FF



;
 





 




 
#line 1330 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long SupportedDriveModes0x6502



;
 





 




 
#line 1371 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned short IndexDistance;  
unsigned short MaximumNumberOfModules;  
} 
TOBJF000;




 
extern TOBJF000 ModularDeviceProfile0xF000



;
 





 




 
#line 1421 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned long aEntries[1];   
} 
TOBJF010;




 
extern TOBJF010 ModuleProfileList0xF010



;
 







#line 1524 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"

 
#line 361 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"










#pragma pack(push, 2)
typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
	long  ObjVelocityActualValue;
	short  ObjModesOfOperationDisplay;
}
TCiA402PDO1A00;

typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
}
TCiA402PDO1A01;

typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
}
TCiA402PDO1A02;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetPosition;
	long  ObjTargetVelocity;
	short  ObjModesOfOperation;
}
TCiA402PDO1600;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetPosition;
}
TCiA402PDO1601;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetVelocity;
}
TCiA402PDO1602;
#pragma pack(pop)

typedef struct 
{
	TOBJ1600 *pRxPdoMappingCspCsv0x1600;
	TOBJ1601 *pRxPdoMappingCsp0x1601;
	TOBJ1602 *pRxPdoMappingCsv0x1602;
	TOBJ1A00 *pTxPdoMappingCspCsv0x1A00;
	TOBJ1A01 *pTxPdoMappingCsp0x1A01;
	TOBJ1A02 *pTxPdoMappingCsv0x1A02;
	TOBJ1C12 *psRxPDOassign;
	TOBJ1C13 *psTxPDOassign;
	unsigned short   *pErrorCode0x603F;
	unsigned short   *pControlword0x6040;
	unsigned short   *pStatusword0x6041;

	unsigned long	 *pVlRampFunctionTime0x604F;

	short    *pQuickStopOptionCode0x605A;
	short    *pShutdownOptionCode0x605B;
	short    *pDisableOperationOptionCode0x605C;
	short    *pFaultReactionOptionCode0x605E;
	char     *pModesOfOperation0x6060;
	char     *pModesOfOperationDisplay0x6061;
	long    *pPositionActualValue0x6064;
	long    *pVelocityActualValue0x606C;
	long    *pTorqueActualValue0x6077;
	long    *pTargetPosition0x607A;

	long    *pHomeOffset0x607C;
	char     *pHomingMethod0x6098;
	TOBJ6099 *pHomingSpeed0x6099;
	unsigned long   *pHomingAcceleration0x609A;
	TOBJ60E3 *pSupportedHomingMode0x60E3;
	unsigned long   *pDigitalInputs0x60FD;

	TOBJ607D *pSoftwarePositionLimit0x607D;
	unsigned long   *pQuickStopDeceleration0x6085;
	TOBJ60C2 *pInterpolationTimePeriod0x60C2;
	long    *pTargetVelocity0x60FF;
	unsigned long   *pSupportedDriveModes0x6502;
	TOBJF000 *pModularDeviceProfile0xF000;
	TOBJF010 *pModuleProfileList0xF010;
}
CiA402_OBJ_T;

typedef struct 
{
	unsigned char             bAxisIsActive;  
	unsigned char             bBrakeApplied;  
	unsigned char             bLowLevelPowerApplied;  
	unsigned char             bHighLevelPowerApplied;  
	unsigned char             bAxisFunctionEnabled;  
	unsigned char             bConfigurationAllowed;  

	unsigned short           i16State;  
	unsigned short           u16PendingOptionCode;  
	double           fCurPosition;  
	unsigned long           u32CycleTime;  
	
	CiA402_OBJ_T     CiA402Var;
	TOBJECT   *pObjDic;

	long						objTargetVelocityBackup;

	CIA402_FUNC_INTF *pFuncIntf;
	CIA402_FLAGS     flags;
	unsigned char             bHighLevelPowerAppliedPreStatus;
	unsigned char             bAxisFunctionEnabledPreStatus;
	unsigned char             EmulationEnable;
	long            TargetVelocity;	
}
CiA402_AXIS_T;

extern void CiA402_Init(void);
extern void CiA402_LocalError(unsigned short ErrorCode);
extern void CiA402_StateMachine(CiA402_AXIS_T *pAxis);
extern void CiA402_DeInit(void);

extern void APPL_Application(void);




extern void   APPL_AckErrorInd(unsigned short stateTrans);
extern unsigned short APPL_StartMailboxHandler(void);
extern unsigned short APPL_StopMailboxHandler(void);
extern unsigned short APPL_StartInputHandler(unsigned short *pIntMask);
extern unsigned short APPL_StopInputHandler(void);
extern unsigned short APPL_StartOutputHandler(void);
extern unsigned short APPL_StopOutputHandler(void);

extern unsigned short APPL_GenerateMapping(unsigned short *pInputSize,unsigned short *pOutputSize);
extern void APPL_InputMapping(unsigned short* pData);
extern void APPL_OutputMapping(unsigned short* pData);


 

#line 39 "..\\Source\\Debug\\clicmd.c"
#line 40 "..\\Source\\Debug\\clicmd.c"
#line 1 "..\\Source\\Debug\\ax_esc_regs.h"




























 




 
#line 36 "..\\Source\\Debug\\ax_esc_regs.h"


 


 

 



typedef union
{
  uint16_t d16;
  struct{
    uint8_t maintenance_ver_z:       4;   
    uint8_t minor_ver_y:             4;   	
    uint8_t patch_level:             8;   			
  } b;
} oESC_CORE_BUILD;




typedef union
{
  uint8_t d8;
  struct{
    uint8_t port0_config:            2;   
    uint8_t port1_config:            2;
    uint8_t port2_config:            2;		
    uint8_t port3_config:            2;		
  } b;
} oPORT_DESCRIPTOR;

typedef union
{
  uint16_t d16;
  struct{
    uint16_t fmmu_operation:             1;   
    uint16_t unused_register_access:     1;   
    uint16_t distributed_clocks:         1;   		
    uint16_t dc_width:                   1;   
    uint16_t reserved_5_4:               2;   
    uint16_t enhanced_link_detec_mii:    1;   
    uint16_t separate_handle_of_fcs_err: 1;   
    uint16_t enhanced_dc_sync_act:       1;   		
    uint16_t lrw_command_support:        1;   
    uint16_t read_write_cmd_support:     1;   		
    uint16_t fixed_fmmu_sm_config:       1;   				
    uint16_t reserved_15_12:             4;
  } b;
} oESC_FEATURES_SUPPORTED;



typedef union
{
  uint8_t d8;
  struct{
    uint8_t reg_write_enable:            1;   
    uint8_t reserved_7_1:                7;
  } b;
} oREG_WRITE_ENABLE;

typedef union
{
  uint8_t d8;
  struct{
    uint8_t reg_write_protect:           1;   
    uint8_t reserved_7_1:                7;		
  } b;
} oREG_WRITE_PROTECTION;

typedef union
{
  uint8_t d8;
  struct{
    uint8_t esc_write_enable:            1;   
    uint8_t reserved_7_1:                7;		
  } b;
} oESC_WRITE_ENABLE;

typedef union
{
  uint8_t d8;
  struct{
    uint8_t esc_write_protect:           1;   
    uint8_t reserved_7_1:                7;		
  } b;
} oESC_WRITE_PROTECTION;

 

typedef union
{
  uint16_t d16;
  struct{
    uint16_t initiate_state:                  4;  




 
    uint16_t err_ind_ack:                     1;   
    uint16_t device_identification:           1;   
    uint16_t reserved_15_6:                  10;   
  } b;
} oAL_CONTROL;

typedef union
{
  uint16_t d16;
  struct{
    uint16_t actual_state:                    4;  




 
    uint16_t err_ind:                         1;   
    uint16_t device_identification:           1;   
    uint16_t reserved_15_6:                  10;   
  } b;
} oAL_STATUS;



typedef union
{
  uint8_t d8;
  struct{
    uint8_t device_emulation:                 1;   
    uint8_t enhanced_link_detec_all_port:     1;   
    uint8_t distributed_clk_sync_out_unit:    1;   
    uint8_t distributed_clk_latch_in_unit:    1;   
    uint8_t enhanced_link_port0:              1;   
    uint8_t enhanced_link_port1:              1;   
    uint8_t enhanced_link_port2:              1;   
    uint8_t enhanced_link_port3:              1;   
  } b;
} oESC_CONFIG;


typedef union
{
  uint32_t d32;

		 
  struct{
		uint32_t pdi_cfg_7_0:             8;   
		uint32_t sync0_driver_pol:        2;  



 
		uint32_t sync0_latch0_cfg:        1;  

 
		uint32_t sync0_map_al_event:      1;  

 
		
		uint32_t sync1_driver_pol:        2;  



 
		uint32_t sync1_latch1_cfg:        1;  

 
		uint32_t sync1_map_al_event:      1;  

 		
		uint32_t pdi_cfg_31_16:          16;   
  } bGeneralTerm;

	 
  struct{
    uint32_t output_valid_pol:        1;   
    uint32_t output_valid_mode:       1;   
    uint32_t unidir_bidir_mode:       1;   
    uint32_t watchdog_behavior:       1;   	
    uint32_t input_sample_at:         2;   	
    uint32_t output_update_at:        2;   
    uint32_t general_term_15_8:       8;   	
    uint32_t io_dir:                 16;   
  } bDIO;
	
	 
  struct{
    uint32_t spi_mode:                2;  



 
    uint32_t spi_irq_driver_pol:      2;  



 
    uint32_t spi_sel_pol:             1;  

 
    uint32_t data_out_sampl_mode:     1;  

 	
    uint32_t reserved_7_6:            2;   
    uint32_t general_term_15_8:       8;   
    uint32_t io_dir:                 16;   
  } bSPIS;
	
} oPDI_CONFIG;





typedef union
{
  uint16_t d16;
  struct{
    uint16_t dc_latch_event:                 1;   
    uint16_t reserved_1:                     1;   
    uint16_t dc_status_event:                1;   
    uint16_t al_status_event:                1;   
    uint16_t sync_0_7_manager_status:        8;   
		uint16_t reserved_15_12:                 4;
  } b;
} oECAT_EVENT;

typedef union
{
  uint32_t d32;
  struct{
    uint32_t al_control_event:               1;   
    uint32_t dc_latch_event:                 1;   
    uint32_t state_of_dc_sync_0:             1;   
    uint32_t state_of_dc_sync_1:             1;   
    uint32_t sm_activation:                  1;   
    uint32_t eep_emulation:                  1;   		
    uint32_t watchdog_process_data:          1;   				
    uint32_t sm_0_15_interrupts:            16;   				
    uint32_t reserved_32_24:                15;   						
  } b;
} oAL_EVENT;


typedef union
{
  uint8_t d8;
  struct{
		uint8_t spi_clk_num_of_access:           3;   
		uint8_t busy_violation_during_read:      1;   
		uint8_t read_termination_missing:        1;   		
		uint8_t access_continue_after_read:      1;   
		uint8_t cmd_err:                         2;   		
  } bSPI;
	
  struct{
		uint8_t busy_viola_during_read:          1;   
		uint8_t busy_viola_during_write:         1;   		
		uint8_t addr_err_for_read:               1;   
		uint8_t addr_err_for_write:              1;   		
		uint8_t reserved_7_4:                    4;   		
  } bASYNC_SYNC;	
} oPDI_ERR_CODE;


 




typedef union
{
  uint16_t d16;
  struct{
        uint16_t wd_process_data_status:         1;   				
        uint16_t reserved_15_1:                 15;   						
  } b;
} oWD_STATUS_PROCESS_DATA;



 






	 
typedef union
{
  uint8_t d8;
  struct{
    uint8_t eep_offered_to_pdi:      1;   
    uint8_t force_ecat_access:       1;   	
		uint8_t reserved_7_2:            6;
  } b;
} oEEPCFGR;

	 
typedef union
{
  uint8_t d8;
  struct{
    uint8_t pdi_access_to_eep:       1;   
		uint8_t reserved_7_1:            7;
  } b;
} oEEPPASR;

 




typedef union
{
  uint16_t d16;
  struct{
    uint16_t write_enable:            1;   
    uint16_t reserved_4_1:            4;   		
    uint16_t eep_emulation:           1;   			
    uint16_t read_8bytes:             1;   					
    uint16_t two_addr_bytes:          1;   							
    uint16_t cmd:                     3;   									
    uint16_t checksum_err:            1;   									
    uint16_t eep_loading_status:      1;   
    uint16_t err_ack_cmd:             1;   		
    uint16_t err_write_enable:        1;   				
    uint16_t eep_intf_busy:           1;   						
  } b;
} oEEPCSR;

typedef struct
{
  oEEPCFGR   eepcfgr;	
  oEEPPASR   eeppasr;		
  oEEPCSR    eepcsr;
	uint32_t        eepar;
	uint8_t         eepdr[8];	
} EEP_REGS;

 
#line 389 "..\\Source\\Debug\\ax_esc_regs.h"

	 
typedef union
{
  uint16_t d16;
  struct{
    uint16_t write_enable:            1;   
    uint16_t pdi_ctrl_possible:       1;   		
    uint16_t link_detec_enable:       1;   		
    uint16_t port0_phy_addr:          5;   
    uint16_t cmd:                     2;   		
    uint16_t reseverd_12_10:          3;   
    uint16_t read_err:                1;   
    uint16_t cmd_err:                 1;   
    uint16_t busy:                    1;   		
  } b;
} oMMCSR;





   
typedef union
{
  uint8_t d8;
  struct{
    uint8_t  phy_addr:                5;   
    uint8_t  reserved_6_5:            2;   
    uint8_t  show_phy_addr:           1;   	
  } b;
} oPAR;

   
typedef union
{
  uint8_t d8;
  struct{
    uint8_t  phy_reg_addr:            5;   
    uint8_t  reserved_7_5:            3;   		
  } b;
} oPRAR;

	 
typedef union
{
  uint16_t d16;
  struct{
    uint16_t rw_data:                 16;   
  } b;
} oPDR;

   
typedef union
{
  uint8_t d8;
  struct{
    uint8_t exclusive_access_mii:      1;   
    uint8_t reserved_7_1:              7;   		
  } b;
} oMMEASR;

   
typedef union
{
  uint8_t d8;
  struct{
    uint8_t pdi_access_mii:            1;   
    uint8_t ecat_force_pdi_access:     1;   
    uint8_t reserved_7_2:              6;   
  } b;
} oMMPASR;

 
typedef union
{
  uint8_t d8;
  struct{
    uint8_t phy_link_detected:         1;   
    uint8_t link_detected:             1;   		
    uint8_t link_error:                1;   				
    uint8_t read_error:                1;   						
    uint8_t link_partner_error:        1;   								
    uint8_t phy_config_updated:        1;   										
    uint8_t reserved_7_6:              2;   												
  } b;
} oPPSR;

typedef struct
{
  oMMCSR    mmcsr;
  oPAR      par;
  oPRAR     prar;
  oPDR      pdr;	
  oMMEASR   mmeasr;	
  oMMPASR   mmpasr;		
  oPPSR     ppsr[4];			
} MII_REGS;

 




typedef union
{
  uint8_t d8;
  struct{
		uint8_t logical_bit:                    3;   				
		uint8_t reserved_7_3:                   5;   						
  } b;
} oFMMU_LOGIC_START_STOP_BIT;






 



typedef union
{
  uint8_t d8;
  struct{
		uint8_t operation_mode:                 2;   				
		uint8_t direction:                      2;   
		uint8_t intr_in_ecat_event_request:     1;   		
		uint8_t intr_in_al_event_request:       1;   				
		uint8_t watchdog_trigger_enable:        1;   						
		uint8_t reserved_7:                     1;   								
  } b;
} oSM_CONTROL;

typedef union
{
  uint8_t d8;
  struct{
		uint8_t intr_write:                     1;   				
		uint8_t intr_read:                      1;   
		uint8_t reserved_2:                     1;   		
		uint8_t mailbox_status:                 1;   				
		uint8_t three_buf_status:               2;   						
		uint8_t read_buf_in_use:                1;   								
		uint8_t write_buf_in_use:               1;   										
  } b;
} oSM_STATUS;

typedef union
{
  uint8_t d8;
  struct{
		uint8_t sm_enable:                      1;   				
		uint8_t repeat_request:                 1;   
		uint8_t reserved_5_2:                   4;   		
		uint8_t latch_event_ecat:               1;   				
		uint8_t latch_event_pdi:                1;   						
  } b;
} oSM_ACTIVATE;

typedef union
{
  uint8_t d8;
  struct{
		uint8_t deactivate_sm:                  1;   				
		uint8_t repeat_ack:                     1;   
		uint8_t reserved_7_2:                   4;   		
  } b;
} oSM_PDI_CONTROL;


 




typedef union
{
  uint32_t d32;
  struct{
		uint32_t local_time7_0_at_wr_reg_0x900:  8;   				
		uint32_t local_time24_8_at_wr_reg_0x900: 24;  
  } b;
} oDC_PORT0_RECV_TIME;



typedef union
{
  uint32_t d32[2];
  struct{
		uint32_t value_compare_with_systim:    32;  				
		uint32_t local_copy_of_systim:         32;  
  } b;
} oDC_SYSTEM_TIME;




typedef union
{
  int32_t d32;
  struct{
		uint32_t diff_rev_systim_loc_copy_systim: 31;  				
		uint32_t loc_copy_systim_LorE_rev_systim: 1;  
  } b;
} oDC_SYSTEM_TIME_DIFF;


typedef union
{
  uint16_t d16;
  struct{
		uint16_t bw_for_adj_of_loc_copy_of_systim: 15;  				
		uint16_t reserved_15:                       1;  
  } b;
} oDC_SPEED_COUNTER_START;



typedef union
{
  uint8_t d8;
  struct{
		uint8_t filt_depth_for_avg_rev_systim_deva: 4;  				
		uint8_t reserved_7_4:                       4;  
  } b;
} oDC_SYSTIME_DIFF_FILTER_DEPTH;


typedef union
{
  uint8_t d8;
  struct{
		uint8_t filt_depth_for_avg_clk_period_deva: 4;  				
		uint8_t reserved_7_4:                       4;  
  } b;
} oDC_SPEED_COUNTER_FILTER_DEPTH;


typedef union
{
  uint8_t d8;
  struct{
		uint8_t rev_time_latch_mode:                1; 


 				
		uint8_t reserved_7_1:                       7;  
  } b;
} oDC_REV_TIME_LATCH_MODE;


typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync_out_unit_ctrl:                 1;  
		uint8_t reserved_3_1:                       3;  
		uint8_t latch_in_unit_0:                    1;  
		uint8_t latch_in_unit_1:                    1;  		
		uint8_t reserved_7_6:                       2;  		
  } b;
} oDC_CYCLIC_UNIT_CTRL;


typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync_out_unit_act:                  1;  
		uint8_t sync0_gen:                          1;  
		uint8_t sync1_gen:                          1;  		
		uint8_t auto_act_by_wr_start_time_cyc_op:   1;  
		uint8_t ext_of_start_time_cyc_op:           1;  
		uint8_t start_time_plausibility_chk:        1;  
		uint8_t near_future_configuration:          1;  
		uint8_t sync_debug_pulse:                   1; 

 		
  } b;
} oDC_SYNC_ACTIVATION;



typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync0_act_state:                    1;  
		uint8_t sync1_act_state:                    1;  
		uint8_t start_time_plausibility_chk_result: 1;  		
		uint8_t reserved_7_3:                       5;  				
  } b;
} oDC_ACTIVATION_STATUS;



typedef union
{
  uint8_t d8;
  struct{
		uint8_t syncx_state_for_ack_mode:           1;  
		uint8_t reserved_7_1:                       7;  				
  } b;
} oDC_SYNCx_STATUS;











typedef union
{
  uint8_t d8;
  struct{
		uint8_t latchx_positive_edge:               1;  
		uint8_t latchx_negative_edge:               1;  				
		uint8_t reserved_7_2:                       6;  						
  } b;
} oDC_LATCHx_CONTROL;



typedef union
{
  uint8_t d8;
  struct{
		uint8_t event_latchx_positive_edge:         1; 

 
		uint8_t event_latchx_negative_edge:         1; 

 
		uint8_t latchx_pin_state:                   1;
		uint8_t reserved_7_3:                       5;  						
  } b;
} oDC_LATCHx_STATUS;

#line 741 "..\\Source\\Debug\\ax_esc_regs.h"

 



 



 



 

 

 



 
#line 41 "..\\Source\\Debug\\clicmd.c"

#line 44 "..\\Source\\Debug\\clicmd.c"


 
#line 56 "..\\Source\\Debug\\clicmd.c"





 

 

 
extern unsigned char bDcSyncActive;
extern unsigned char bEscIntEnabled;
extern unsigned char bEcatFirstOutputsReceived;
extern HW_SPI_OBJECT HW_SpiObj;
extern CiA402_AXIS_T CiA402Axis;

 
static uint8_t StrBuf[1024];
static TEST_CONTROL clicmd_testCtrl;

 
static uint8_t clicmd_DecText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len);
static uint8_t clicmd_HexText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len);
static uint8_t clicmd_DecText2SignLong(uint8_t *pbuf, int32_t *pValue, uint8_t len);
static void clicmd_ShowMemoryInHex8b(CONSOLE_Inst *pInst, uint8_t *pbuf, uint32_t len);
static void clicmd_ShowMemoryInHex16b(CONSOLE_Inst *pInst, uint16_t *pbuf, uint32_t len);
static int clicmd_SystemReboot(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SystemRebootHelp(CONSOLE_Inst *inst);
static int clicmd_SystemRebootUsage(CONSOLE_Inst *inst);
static int clicmd_PdiRead(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiReadHelp(CONSOLE_Inst *inst);
static int clicmd_PdiReadUsage(CONSOLE_Inst *inst);
static int clicmd_PdiWrite(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiWriteHelp(CONSOLE_Inst *inst);
static int clicmd_PdiWriteUsage(CONSOLE_Inst *inst);
static void clicmd_ShowEscFeatures(CONSOLE_Inst *inst);
static void clicmd_ShowEscAlInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscWdInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscFmmuSmInfo(CONSOLE_Inst *inst);
static void clicmd_ShowEscDcInfo(CONSOLE_Inst *inst);


static int clicmd_ClkRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_ClkRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_ClkRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SysRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SysRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SysRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_FmcRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_FmcRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_FmcRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SpimRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SpimRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SpimRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_UARTxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_UARTxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_UARTxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_TMRxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_TMRxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_TMRxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_SPIxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_SPIxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_SPIxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_GPIOxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_GPIOxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_GPIOxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_NvicRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_NvicRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_NvicRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_EadcRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EadcRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EadcRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_QEIxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_QEIxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_QEIxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_ECAPxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_ECAPxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_ECAPxRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_EPWMxRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EPWMxRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EPWMxRegisterUsage(CONSOLE_Inst *inst);



static int clicmd_EscRegister(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EscRegisterHelp(CONSOLE_Inst *inst);
static int clicmd_EscRegisterUsage(CONSOLE_Inst *inst);
static int clicmd_PdiMemoryTest(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiMemoryTestHelp(CONSOLE_Inst *inst);
static int clicmd_PdiMemoryTestUsage(CONSOLE_Inst *inst);
static int clicmd_PdiReset(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_PdiResetHelp(CONSOLE_Inst *inst);


static int clicmd_EscStack(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_EscStackHelp(CONSOLE_Inst *inst);

static int clicmd_McStack(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_McStackHelp(CONSOLE_Inst *inst);
static int clicmd_McStackUsage(CONSOLE_Inst *inst);

static int clicmd_CiA402(CONSOLE_Inst *inst, int argc, char **argv);
static int clicmd_CiA402Help(CONSOLE_Inst *inst);
static int clicmd_CiA402Usage(CONSOLE_Inst *inst);

CONSOLE_Account CLICMD_userTable[1];

CONSOLE_CmdEntry CLICMD_userCmdTable[]=
{
	{";", 0, 0, 5},	
	{"reboot", clicmd_SystemReboot, clicmd_SystemRebootHelp, 5},
	

	{"sys", clicmd_SysRegister, clicmd_SysRegisterHelp, 5},	
	{"clk", clicmd_ClkRegister, clicmd_ClkRegisterHelp, 5},	
	{"fmc", clicmd_FmcRegister, clicmd_FmcRegisterHelp, 5},
	{"spim", clicmd_SpimRegister, clicmd_SpimRegisterHelp, 5},	
	{"uartx", clicmd_UARTxRegister, clicmd_UARTxRegisterHelp, 5},		
	{"tmrx", clicmd_TMRxRegister, clicmd_TMRxRegisterHelp, 5},
	{"spix", clicmd_SPIxRegister, clicmd_SPIxRegisterHelp, 5},		
	{"gpiox", clicmd_GPIOxRegister, clicmd_GPIOxRegisterHelp, 5},		
	{"nvic", clicmd_NvicRegister, clicmd_NvicRegisterHelp, 5},		
	{"eadc", clicmd_EadcRegister, clicmd_EadcRegisterHelp, 5},	
	{"qeix", clicmd_QEIxRegister, clicmd_QEIxRegisterHelp, 5},
	{"ecapx", clicmd_ECAPxRegister, clicmd_ECAPxRegisterHelp, 5},	
	{"epwmx", clicmd_EPWMxRegister, clicmd_EPWMxRegisterHelp, 5},		

	

	{"pdird", clicmd_PdiRead, clicmd_PdiReadHelp, 5}, 	
	{"pdiwr", clicmd_PdiWrite, clicmd_PdiWriteHelp, 5},	
	{"pdimt", clicmd_PdiMemoryTest, clicmd_PdiMemoryTestHelp, 5},		
	{"escreg", clicmd_EscRegister, clicmd_EscRegisterHelp, 5},	
	{"pdirst", clicmd_PdiReset, clicmd_PdiResetHelp, 5},


	{"esc", clicmd_EscStack, clicmd_EscStackHelp, 5},		
	{"mc", clicmd_McStack, clicmd_McStackHelp, 5},
	{"cia402", clicmd_CiA402, clicmd_CiA402Help, 5},		
};

 









 
static uint8_t clicmd_DecText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 3))
	{
		return 0xFF;
	}

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3A) && (*pbuf > 0x2F))
		{
			*pValue += (*pbuf - 0x30);
		}
		else
		{
			return 0xFF;
		}

		pbuf++;
	}

	if (*pValue > 255)
	{
		return 0xFF;
	}

	return 0;

}  









 
static uint8_t clicmd_HexText2Char(uint8_t *pbuf, uint8_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 2))
	{
		return 0xFF;
	}

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3A) && (*pbuf > 0x2F))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xFF;

		pbuf++;
	}

	return 0;

}  









 
static uint8_t clicmd_DecText2Short(uint8_t *pbuf, unsigned short *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 5))
		return 0xff;

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else
			return 0xff;

		pbuf++;
	}

	if (*pValue > 65535)
		return 0xff;

	return 0;

}  









 
static uint8_t clicmd_HexText2Short(uint8_t *pbuf, uint16_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 4))
		return 0xff;

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xff;

		pbuf++;
	}

	return 0;

}  









 
static uint8_t clicmd_DecText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 10))
		return 0xff;

	while (len--)
	{
		*pValue *= 10;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else
			return 0xff;

		pbuf++;
	}

	if (*pValue > 0xffffffff)
		return 0xff;

	return 0;

}  









 
static uint8_t clicmd_HexText2Long(uint8_t *pbuf, uint32_t *pValue, uint8_t len)
{
	*pValue = 0;

	if ((len == 0) || (len > 8))
		return 0xff;

	while (len--)
	{
		*pValue *= 16;

		if ((*pbuf < 0x3a) && (*pbuf > 0x2f))
			*pValue += (*pbuf - 0x30);
		else if ((*pbuf < 0x47) && (*pbuf > 0x40))
			*pValue += (*pbuf - 0x37);
		else if ((*pbuf < 0x67) && (*pbuf > 0x60))
			*pValue += (*pbuf - 0x57);
		else
			return 0xff;

		pbuf++;
	}

	return 0;

}  









 
static uint8_t clicmd_DecText2SignLong(uint8_t *pbuf, int32_t *pValue, uint8_t len)
{
	uint8_t negSign = 0, ret;
	uint32_t Value;
	
	*pValue = 0;

	if (*pbuf == '-')
	{
		len--;
		pbuf++;
		negSign=1;
	}
	
	ret = clicmd_DecText2Long(pbuf, &Value, len);
	*pValue = Value;
	if (negSign)
	{
		*pValue = (-1)*(*pValue);
	}
	return ret;

}  









 
static void clicmd_ShowMemoryInHex8b(CONSOLE_Inst *inst, uint8_t *pbuf, uint32_t len)
{
	uint32_t i;
	
	for (i=0; i<len; i++)
	{
		CONSOLE_PutMessage(inst, "0x%02x%s", pbuf[i], ((i+1) == len) ? "\r\n":", ");
		if ((i%16)==15)
		{
			CONSOLE_PutMessage(inst, "\r\n");
		}
	}
	CONSOLE_PutMessage(inst, "\r\n");	
}  









 
static void clicmd_ShowMemoryInHex16b(CONSOLE_Inst *inst, uint16_t *pbuf, uint32_t WordLen)
{
	uint32_t i;
	
	for (i=0; i<WordLen; i++)
	{
		CONSOLE_PutMessage(inst, "0x%04x%s", pbuf[i], ((i+1) == WordLen) ? "\r\n":", ");
		if ((i%16)==15)
		{
			CONSOLE_PutMessage(inst, "\r\n");
		}
	}
	CONSOLE_PutMessage(inst, "\r\n");	
}  









 
static void clicmd_ShowEscFeatures(CONSOLE_Inst *inst)
{
	char *pStr;
	uint8_t tmp8;
	uint16_t tmp16;	
	uint32_t i;
	oESC_CORE_BUILD oEscCoreBuild;
	oPORT_DESCRIPTOR oPortDescriptor;
	oESC_FEATURES_SUPPORTED oEscFeaturesSupported;
	oREG_WRITE_ENABLE       oRegWriteEnable;
	oREG_WRITE_PROTECTION   oRegWriteProtect;
	oESC_WRITE_ENABLE       oEscWriteEnable;
	oESC_WRITE_PROTECTION   oEscWriteProtect;

	oIC_PRODUCT_ID          oProductId;
	oIC_VENDOR_ID           oVendorId;
	
	CONSOLE_PutMessage(inst, "\r\nESC Features...\r\n");
	
	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0000, (unsigned short)1);
	switch(tmp8)
	{
	case 0x11: pStr = "ET1100";	break;
	case 0x12: pStr = "ET1200";	break;
	case 0xC8: pStr = "AX58100";	break;		
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Type = %s\r\n", 0x0000, tmp8, pStr);

	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0001, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Revision\r\n", 0x0001, tmp8);

	HW_EscRead((unsigned long*)&oEscCoreBuild . d16, (unsigned short)0x0002, (unsigned short)2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Build\r\n", 0x0002, tmp8);
	CONSOLE_PutMessage(inst, "      maintenance_ver_z: %u\r\n", oEscCoreBuild.b.maintenance_ver_z);	
	CONSOLE_PutMessage(inst, "      minor_ver_y: %u\r\n", oEscCoreBuild.b.minor_ver_y);	
	CONSOLE_PutMessage(inst, "      patch_level: %u\r\n", oEscCoreBuild.b.patch_level);		
	
	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0004, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, FMMUS supporrted\r\n", 0x0004, tmp8);

	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0005, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Sync managers supporrted\r\n", 0x0005, tmp8);

	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0006, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, RAM size\r\n", 0x0006, tmp8);

	HW_EscRead((unsigned long*)&oPortDescriptor . d8, (unsigned short)0x0007, (unsigned short)1);

	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Port descriptor\r\n", 0x0007, oPortDescriptor.d8);
	for (i=0; i<4; i++)
	{
		tmp8 = ((oPortDescriptor.d8 >> (2*i)) & 0x03);
		switch(tmp8)
		{
		case 0x00: pStr = "Not implemented";	break;
		case 0x01: pStr = "Not configured";	break;
		case 0x02: pStr = "EBUS";	break;		
		default:   pStr = "MII/RMII/RGMII"; break;
		}
		CONSOLE_PutMessage(inst, "      port%u_config: %s\r\n", i, pStr);		
	}
	
	HW_EscRead((unsigned long*)&oEscFeaturesSupported . d16, (unsigned short)0x0008, (unsigned short)2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, ESC features supported\r\n", 0x0008, oEscFeaturesSupported.d16);
	CONSOLE_PutMessage(inst, "      fmmu_operation: %u, %s oriented\r\n"
																								, oEscFeaturesSupported.b.fmmu_operation
																								, (oEscFeaturesSupported.b.fmmu_operation==0)?"Bit":"Byte");	
	CONSOLE_PutMessage(inst, "      unused_register_access: %u, %s\r\n"
																								, oEscFeaturesSupported.b.unused_register_access
																								, (oEscFeaturesSupported.b.unused_register_access==0)?"Allowed":"Not supported");		
	CONSOLE_PutMessage(inst, "      distributed_clocks: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.distributed_clocks
																								, (oEscFeaturesSupported.b.distributed_clocks==0)?"Not":"");			
	CONSOLE_PutMessage(inst, "      dc_width: %u, %sbit\r\n"
																								, oEscFeaturesSupported.b.dc_width
																								, (oEscFeaturesSupported.b.dc_width==0)?"32":"64");				
#line 601 "..\\Source\\Debug\\clicmd.c"
	CONSOLE_PutMessage(inst, "      enhanced_link_detec_mii: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.enhanced_link_detec_mii
																								, (oEscFeaturesSupported.b.enhanced_link_detec_mii==0)?"Not":"");							
	CONSOLE_PutMessage(inst, "      separate_handle_of_fcs_err: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.separate_handle_of_fcs_err
																								, (oEscFeaturesSupported.b.separate_handle_of_fcs_err==0)?"Not":"");
	CONSOLE_PutMessage(inst, "      enhanced_dc_sync_act: %u, %s Available\r\n"
																								, oEscFeaturesSupported.b.enhanced_dc_sync_act
																								, (oEscFeaturesSupported.b.enhanced_dc_sync_act==0)?"Not":"");
	CONSOLE_PutMessage(inst, "      lrw_command_support: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.lrw_command_support
																								, (oEscFeaturesSupported.b.lrw_command_support==0)?"":"Not");
	CONSOLE_PutMessage(inst, "      read_write_cmd_support: %u, %s Supported\r\n"
																								, oEscFeaturesSupported.b.read_write_cmd_support
																								, (oEscFeaturesSupported.b.read_write_cmd_support==0)?"":"Not");										
	CONSOLE_PutMessage(inst, "      fixed_fmmu_sm_config: %u, %s configuration\r\n"
																								, oEscFeaturesSupported.b.fixed_fmmu_sm_config
																								, (oEscFeaturesSupported.b.fixed_fmmu_sm_config==0)?"Variable":"Fixed");

	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0010, (unsigned short)2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Configured station address\r\n", 0x0010, tmp16);	
	
	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0012, (unsigned short)2);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Configured station Alias\r\n", 0x0012, tmp16);		

	HW_EscRead((unsigned long*)&oRegWriteEnable . d8, (unsigned short)0x0020, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Register write enable\r\n", 0x0020, oRegWriteEnable.d8);		
	CONSOLE_PutMessage(inst, "      reg_write_enable: %u, %s\r\n"
																								, oRegWriteEnable.b.reg_write_enable
																								, (oRegWriteEnable.b.reg_write_enable==0)?"enable":"disable");	

	HW_EscRead((unsigned long*)&oRegWriteProtect . d8, (unsigned short)0x0021, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Register write protection\r\n", 0x0021, oRegWriteProtect.d8);		
	CONSOLE_PutMessage(inst, "      reg_write_protect: %u, %s\r\n"
																								, oRegWriteProtect.b.reg_write_protect
																								, (oRegWriteProtect.b.reg_write_protect==0)?"disable":"enable");	

	HW_EscRead((unsigned long*)&oEscWriteEnable . d8, (unsigned short)0x0030, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC write enable\r\n", 0x0030, oEscWriteEnable.d8);		
	CONSOLE_PutMessage(inst, "      esc_write_enable: %u, %s\r\n"
																								, oEscWriteEnable.b.esc_write_enable
																								, (oEscWriteEnable.b.esc_write_enable==0)?"enable":"disable");	

	HW_EscRead((unsigned long*)&oEscWriteProtect . d8, (unsigned short)0x0031, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC write protection\r\n", 0x0031, oEscWriteProtect.d8);		
	CONSOLE_PutMessage(inst, "      esc_write_protect: %u, %s\r\n"
																								, oEscWriteProtect.b.esc_write_protect
																								, (oEscWriteProtect.b.esc_write_protect==0)?"disable":"enable");	
	
	HW_EscRead((unsigned long*)&oProductId . d32, (unsigned short)0x0E00, (unsigned short)sizeof(oProductId));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Product ID\r\n", 0x0E00, oProductId.d32[0], oProductId.d32[1]);
	CONSOLE_PutMessage(inst, "      chip_version: 0x%02x\r\n", oProductId.b.chip_revision);
	CONSOLE_PutMessage(inst, "      package_type: 0x%02x\r\n", oProductId.b.package_type);
	CONSOLE_PutMessage(inst, "      product_id: 0x%x\r\n", oProductId.b.product_id);	
	
	HW_EscRead((unsigned long*)&oVendorId . d32, (unsigned short)0x0E08, (unsigned short)sizeof(oVendorId));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Vendor ID\r\n", 0x0E08, oVendorId.d32[0], oVendorId.d32[1]);
	
}  









 
static void clicmd_ShowEscAlInfo(CONSOLE_Inst *inst)
{
	char *pStr;
	uint8_t tmp8, pdi_type;
	uint16_t tmp16;	
	uint32_t tmp32;
	oESC_CONFIG             oEscConfig;
	oPDI_CONFIG             oPdiConfig;
	oAL_CONTROL             oAlControl;
	oAL_STATUS              oAlStatus;
	oECAT_EVENT             oEcatEvent;
	oAL_EVENT               oAlEvent;
	oPDI_ERR_CODE           oPdiErrCode;
	
	CONSOLE_PutMessage(inst, "\r\nESC Application Layer Info...\r\n");
	
	HW_EscRead((unsigned long*)&oAlControl . d16, (unsigned short)0x0120, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Control\r\n", 0x0120, oAlControl.d16);
	switch (oAlControl.b.initiate_state)
	{
	case 1:	pStr = "Init"; break;
	case 3:	pStr = "Bootstrap";	break;
	case 2:	pStr = "Pre-Operation";	break;
	case 4:	pStr = "Safe-Operation"; break;
	case 8:	pStr = "Operation";	break;
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "      initiate_state: %u, %s\r\n", oAlControl.b.initiate_state, pStr);
	CONSOLE_PutMessage(inst, "      err_ind_ack: %u\r\n", oAlControl.b.err_ind_ack);
	CONSOLE_PutMessage(inst, "      device_identification: %u\r\n", oAlControl.b.device_identification);	

	HW_EscRead((unsigned long*)&oAlStatus . d16, (unsigned short)0x0130, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Status\r\n", 0x0130, oAlStatus.d16);
	switch (oAlStatus.b.actual_state)
	{
	case 1:	pStr = "Init"; break;
	case 3:	pStr = "Bootstrap";	break;
	case 2:	pStr = "Pre-Operation";	break;
	case 4:	pStr = "Safe-Operation"; break;
	case 8:	pStr = "Operation";	break;
	default: pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "      actual_state: %u, %s\r\n", oAlStatus.b.actual_state, pStr);
	CONSOLE_PutMessage(inst, "      err_ind: %u\r\n", oAlStatus.b.err_ind);
	CONSOLE_PutMessage(inst, "      device_identification: %u\r\n", oAlStatus.b.device_identification);	
	
	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0134, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, AL Status Code\r\n", 0x0134, tmp16);
	
	HW_EscRead((unsigned long*)&pdi_type, (unsigned short)0x0140, (unsigned short)1);
	switch (pdi_type)
	{
	case 0x00:	pStr = "Interface deactiveated(no PDI)"; break;
	case 0x01:	pStr = "4 Digital Input";	break;
	case 0x02:	pStr = "4 Digital Output";	break;
	case 0x03:	pStr = "2 Digital Input and 2 Digital Output"; break;
	case 0x04:	pStr = "Digital I/O";	break;
	case 0x05:	pStr = "SPI Slave";	break;
	case 0x06:	pStr = "Oversample I/O";	break;
	case 0x07:	pStr = "EtherCAT Bridge(port 3)";	break;		
	case 0x08:	pStr = "16Bit asynchronous Microcontroller";	break;		
	case 0x09:	pStr = "8Bit asynchronous Microcontroller";	break;		
	case 0x0A:	pStr = "16Bit synchronous Microcontroller";	break;		
	case 0x0B:	pStr = "8Bit synchronous Microcontroller";	break;
	case 0x10:	pStr = "32 Digital Input and 0 Digital Output";	break;				
	case 0x11:	pStr = "24 Digital Input and 8 Digital Output";	break;				
	case 0x12:	pStr = "16 Digital Input and 16 Digital Output";	break;		
	case 0x13:	pStr = "8 Digital Input and 24 Digital Output";	break;		
	case 0x14:	pStr = "0 Digital Input and 32 Digital Output";	break;				
	case 0x80:	pStr = "On-chip bus";	break;						
	default:    pStr = "Unsupported"; break;
	}
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Control, %s\r\n", 0x0140, pdi_type, pStr);
	
	HW_EscRead((unsigned long*)&oEscConfig . d8, (unsigned short)0x0141, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, ESC configuration\r\n", 0x0141, oEscConfig.d8);		
	CONSOLE_PutMessage(inst, "      device_emulation: %u\r\n", oEscConfig.b.device_emulation);
	CONSOLE_PutMessage(inst, "      enhanced_link_detec_all_port: %u\r\n", oEscConfig.b.enhanced_link_detec_all_port);
	CONSOLE_PutMessage(inst, "      distributed_clk_sync_out_unit: %u\r\n", oEscConfig.b.distributed_clk_sync_out_unit);
	CONSOLE_PutMessage(inst, "      distributed_clk_latch_in_unit: %u\r\n", oEscConfig.b.distributed_clk_latch_in_unit);
	CONSOLE_PutMessage(inst, "      enhanced_link_port0: %u\r\n", oEscConfig.b.enhanced_link_port0);
	CONSOLE_PutMessage(inst, "      enhanced_link_port1: %u\r\n", oEscConfig.b.enhanced_link_port1);
	CONSOLE_PutMessage(inst, "      enhanced_link_port2: %u\r\n", oEscConfig.b.enhanced_link_port2);
	CONSOLE_PutMessage(inst, "      enhanced_link_port3: %u\r\n", oEscConfig.b.enhanced_link_port3);	

	HW_EscRead((unsigned long*)&oPdiConfig . d32, (unsigned short)0x0150, (unsigned short)sizeof(oPdiConfig . d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ESC configuration\r\n", 0x0150, oPdiConfig.d32);
	CONSOLE_PutMessage(inst, "      sync0_driver_pol: %u\r\n", oPdiConfig.bGeneralTerm.sync0_driver_pol);
	CONSOLE_PutMessage(inst, "      sync0_latch0_cfg: %u\r\n", oPdiConfig.bGeneralTerm.sync0_latch0_cfg);
	CONSOLE_PutMessage(inst, "      sync0_map_al_event: %u\r\n", oPdiConfig.bGeneralTerm.sync0_map_al_event);
	CONSOLE_PutMessage(inst, "      sync1_driver_pol: %u\r\n", oPdiConfig.bGeneralTerm.sync1_driver_pol);
	CONSOLE_PutMessage(inst, "      sync1_latch1_cfg: %u\r\n", oPdiConfig.bGeneralTerm.sync1_latch1_cfg);	
	CONSOLE_PutMessage(inst, "      sync1_map_al_event: %u\r\n", oPdiConfig.bGeneralTerm.sync1_map_al_event);		
	
	if (pdi_type == 0x05)
	{
		CONSOLE_PutMessage(inst, "      PDI Type: SPI\r\n");		
		CONSOLE_PutMessage(inst, "      spi_mode: Mode%u\r\n", oPdiConfig.bSPIS.spi_mode);
		switch (oPdiConfig.bSPIS.spi_irq_driver_pol)
		{
		case 0x00:	pStr = "Push-Pull/Active Low"; break;
		case 0x01:	pStr = "Open-Drain/Active Low";	break;
		case 0x02:	pStr = "Push-Pull/Active High";	break;
		default:	  pStr = "Open-Source/Active High";	break;			
		}
		CONSOLE_PutMessage(inst, "      spi_irq_driver_pol: %u, %s\r\n"
																									, oPdiConfig.bSPIS.spi_irq_driver_pol, pStr);
		CONSOLE_PutMessage(inst, "      spi_sel_pol: %u, Active %s\r\n"
																									, oPdiConfig.bSPIS.spi_sel_pol
																									, (oPdiConfig.bSPIS.spi_sel_pol==0)?"Low":"High");		
		CONSOLE_PutMessage(inst, "      data_out_sampl_mode: %u, %s sample\r\n"
																									, oPdiConfig.bSPIS.data_out_sampl_mode
																									, (oPdiConfig.bSPIS.data_out_sampl_mode==0)?"Normal":"Late");				
	}
	
	HW_EscRead((unsigned long*)&oEcatEvent . d16, (unsigned short)0x0200, (unsigned short)sizeof(oEcatEvent . d16));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event mask\r\n", 0x0200, oEcatEvent.d16);
	CONSOLE_PutMessage(inst, "      dc_latch_event_mask: %u\r\n", oEcatEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      dc_status_event_mask: %u\r\n", oEcatEvent.b.dc_status_event);	
	CONSOLE_PutMessage(inst, "      al_status_event_mask: %u\r\n", oEcatEvent.b.al_status_event);
	CONSOLE_PutMessage(inst, "      sync_0_7_manager_status_mask: %u\r\n", oEcatEvent.b.sync_0_7_manager_status);	
	
	HW_EscRead((unsigned long*)&oAlEvent . d32, (unsigned short)0x0204, (unsigned short)sizeof(oAlEvent . d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event mask\r\n", 0x0204, oAlEvent.d32);
	CONSOLE_PutMessage(inst, "      al_control_event_mask: %u\r\n", oAlEvent.b.al_control_event);	
	CONSOLE_PutMessage(inst, "      dc_latch_event_mask: %u\r\n", oAlEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_0_mask: %u\r\n", oAlEvent.b.state_of_dc_sync_0);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_1_mask: %u\r\n", oAlEvent.b.state_of_dc_sync_1);	
	CONSOLE_PutMessage(inst, "      sm_activation_mask: %u\r\n", oAlEvent.b.sm_activation);
	CONSOLE_PutMessage(inst, "      eep_emulation_mask: %u\r\n", oAlEvent.b.eep_emulation);
	CONSOLE_PutMessage(inst, "      watchdog_process_data_mask: %u\r\n", oAlEvent.b.watchdog_process_data);	
	CONSOLE_PutMessage(inst, "      sm_0_15_interrupts_mask: %u\r\n", oAlEvent.b.sm_0_15_interrupts);		
	
	HW_EscRead((unsigned long*)&oEcatEvent . d16, (unsigned short)0x0210, (unsigned short)sizeof(oEcatEvent . d16));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event request\r\n", 0x0210, oEcatEvent.d16);
	CONSOLE_PutMessage(inst, "      dc_latch_event: %u\r\n", oEcatEvent.b.dc_latch_event);	
	CONSOLE_PutMessage(inst, "      dc_status_event: %u\r\n", oEcatEvent.b.dc_status_event);	
	CONSOLE_PutMessage(inst, "      al_status_event: %u\r\n", oEcatEvent.b.al_status_event);
	CONSOLE_PutMessage(inst, "      sync_0_7_manager_status: %u\r\n", oEcatEvent.b.sync_0_7_manager_status);		

	HW_EscRead((unsigned long*)&oAlEvent . d32, (unsigned short)0x0220, (unsigned short)sizeof(oAlEvent . d32));
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT event request\r\n", 0x0220, oAlEvent.d32);
	CONSOLE_PutMessage(inst, "      al_control_event: %u\r\n", oAlEvent.b.al_control_event);	
	CONSOLE_PutMessage(inst, "      dc_latch_event: %u\r\n", oAlEvent.b.dc_latch_event);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_0: %u\r\n", oAlEvent.b.state_of_dc_sync_0);
	CONSOLE_PutMessage(inst, "      state_of_dc_sync_1: %u\r\n", oAlEvent.b.state_of_dc_sync_1);	
	CONSOLE_PutMessage(inst, "      sm_activation: %u\r\n", oAlEvent.b.sm_activation);
	CONSOLE_PutMessage(inst, "      eep_emulation: %u\r\n", oAlEvent.b.eep_emulation);
	CONSOLE_PutMessage(inst, "      watchdog_process_data: %u\r\n", oAlEvent.b.watchdog_process_data);	
	CONSOLE_PutMessage(inst, "      sm_0_15_interrupts: %u\r\n", oAlEvent.b.sm_0_15_interrupts);		
	
	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x030D, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Error Counter\r\n", 0x030D, tmp8);	

	HW_EscRead((unsigned long*)&oPdiErrCode . d8, (unsigned short)0x030E, (unsigned short)1);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, PDI Error Code\r\n", 0x030E, oPdiErrCode.d8);	
	if (pdi_type == 0x05)
	{
		CONSOLE_PutMessage(inst, "      spi_clk_num_of_access: %u\r\n", oPdiErrCode.bSPI.spi_clk_num_of_access);
		CONSOLE_PutMessage(inst, "      busy_violation_during_read: %u\r\n", oPdiErrCode.bSPI.busy_violation_during_read);
		CONSOLE_PutMessage(inst, "      read_termination_missing: %u\r\n", oPdiErrCode.bSPI.read_termination_missing);
		CONSOLE_PutMessage(inst, "      access_continue_after_read: %u\r\n", oPdiErrCode.bSPI.access_continue_after_read);		
		CONSOLE_PutMessage(inst, "      cmd_err: %u\r\n", oPdiErrCode.bSPI.cmd_err);				
	}
	else if (pdi_type == 0x08 || pdi_type == 0x09 || pdi_type == 0x0A || pdi_type == 0x0B)
	{
		CONSOLE_PutMessage(inst, "      busy_viola_during_read: %u\r\n", oPdiErrCode.bASYNC_SYNC.busy_viola_during_read);
		CONSOLE_PutMessage(inst, "      busy_viola_during_write: %u\r\n", oPdiErrCode.bASYNC_SYNC.busy_viola_during_write);
		CONSOLE_PutMessage(inst, "      addr_err_for_read: %u\r\n", oPdiErrCode.bASYNC_SYNC.addr_err_for_read);
		CONSOLE_PutMessage(inst, "      addr_err_for_write: %u\r\n", oPdiErrCode.bASYNC_SYNC.addr_err_for_write);		
	}
	
	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x0310, (unsigned short)4);
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Lost link counter\r\n", 0x0310, tmp32);		
	
}  









 
static void clicmd_ShowEscWdInfo(CONSOLE_Inst *inst)
{
	uint8_t tmp8;
	uint16_t tmp16;	
	oWD_STATUS_PROCESS_DATA     oWdStatusProcessData;
	
	CONSOLE_PutMessage(inst, "\r\nESC Watchdog Info...\r\n");
	
	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0400, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Divider\r\n", 0x0400, tmp16);

	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0410, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Time PDI\r\n", 0x0410, tmp16);

	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0420, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Time Process Data\r\n", 0x0420, tmp16);

	HW_EscRead((unsigned long*)&oWdStatusProcessData . d16, (unsigned short)0x0440, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Watchdog Status Process Data\r\n", 0x0440, oWdStatusProcessData.d16);
	CONSOLE_PutMessage(inst, "      wd_process_data_status: %u\r\n", oWdStatusProcessData.b.wd_process_data_status);	
	
	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0442, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Watchdog Counter Process Data\r\n", 0x0442, tmp8);

	HW_EscRead((unsigned long*)&tmp8, (unsigned short)0x0443, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Watchdog Counter PDI\r\n", 0x0443, tmp8);

}  









 
static void clicmd_ShowEscFmmuSmInfo(CONSOLE_Inst *inst)
{
	char *pStr;
	uint32_t tmp8;
	uint16_t tmp16;	
	uint32_t i, tmp32, addr;		
	oFMMU_LOGIC_START_STOP_BIT  oFmmuLogicBit;
	oSM_CONTROL                 oSmControl;
	oSM_STATUS                  oSmStatus;
	oSM_ACTIVATE                oSmActivate;
	
	CONSOLE_PutMessage(inst, "\r\nESC FMMU Info...\r\n");
	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "FMMU[%u]:\r\n", i);
		
		addr = 0x0600 + (0x0010*i);
		HW_EscRead((unsigned long*)&tmp32, (unsigned short)addr, (unsigned short)4);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Logical start address\r\n", addr, tmp32);

		addr = 0x0604 + (0x0010*i);
		HW_EscRead((unsigned long*)&tmp16, (unsigned short)addr, (unsigned short)2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Length\r\n", addr, tmp16);

		addr = 0x0606 + (0x0010*i);
		HW_EscRead((unsigned long*)&oFmmuLogicBit . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Logical start bit\r\n", addr, oFmmuLogicBit.b.logical_bit);
		
		addr = 0x0607 + (0x0010*i);
		HW_EscRead((unsigned long*)&oFmmuLogicBit . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Logical stop bit\r\n", addr, oFmmuLogicBit.b.logical_bit);		
		
		addr = 0x0608 + (0x0010*i);
		HW_EscRead((unsigned long*)&tmp16, (unsigned short)addr, (unsigned short)2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Physical start address\r\n", addr, tmp16);		
		
		addr = 0x060A + (0x0010*i);
		HW_EscRead((unsigned long*)&oFmmuLogicBit . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: %u, Physical start bit\r\n", addr, oFmmuLogicBit.b.logical_bit);				
		
		addr = 0x060B + (0x0010*i);
		HW_EscRead((unsigned long*)&tmp8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Type\r\n", addr, tmp8);			
		
		addr = 0x060C + (0x0010*i);
		HW_EscRead((unsigned long*)&tmp8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Activate\r\n", addr, tmp8);	
	}
	
	
	CONSOLE_PutMessage(inst, "\r\nESC SM Info...\r\n");
	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "SM[%u]:\r\n", i);
		
		addr = 0x0800 + (0x0008*i);
		HW_EscRead((unsigned long*)&tmp16, (unsigned short)addr, (unsigned short)2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Physical start address\r\n", addr, tmp16);

		addr = 0x0802 + (0x0008*i);
		HW_EscRead((unsigned long*)&tmp16, (unsigned short)addr, (unsigned short)2);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, Length\r\n", addr, tmp16);

		addr = 0x0804 + (0x0008*i);
		HW_EscRead((unsigned long*)&oSmControl . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Control\r\n", addr, oSmControl.d8);
		switch (oSmControl.b.operation_mode)
		{
		case 0x00:	pStr = "3 buffer mode"; break;
		case 0x02:	pStr = "Mailbox";	break;
		default:	  pStr = "Reserved";	break;			
		}		
		CONSOLE_PutMessage(inst, "      operation_mode: %u, %s\r\n", oSmControl.b.operation_mode, pStr);	
		switch (oSmControl.b.direction)
		{
		case 0x00:	pStr = "ECAT read, PDI write"; break;
		case 0x01:	pStr = "ECAT write, PDI read"; break;
		default:	  pStr = "Reserved";	break;			
		}			
		CONSOLE_PutMessage(inst, "      direction: %u, %s\r\n", oSmControl.b.direction, pStr);
		CONSOLE_PutMessage(inst, "      intr_in_ecat_event_request: %u\r\n", oSmControl.b.intr_in_ecat_event_request);	
		CONSOLE_PutMessage(inst, "      intr_in_al_event_request: %u\r\n", oSmControl.b.intr_in_al_event_request);			
		CONSOLE_PutMessage(inst, "      watchdog_trigger_enable: %u\r\n", oSmControl.b.watchdog_trigger_enable);					
		
		addr = 0x0805 + (0x0008*i);
		HW_EscRead((unsigned long*)&oSmStatus . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Status\r\n", addr, oSmStatus.d8);
		CONSOLE_PutMessage(inst, "      intr_write: %u\r\n", oSmStatus.b.intr_write);			
		CONSOLE_PutMessage(inst, "      intr_read: %u\r\n", oSmStatus.b.intr_read);					
		CONSOLE_PutMessage(inst, "      mailbox_status: %s\r\n", oSmStatus.b.mailbox_status ? "full":"empty");
		switch (oSmStatus.b.three_buf_status)
		{
		case 0:	pStr = "1st buffer"; break;
		case 1:	pStr = "2nd buffer"; break;
		case 2:	pStr = "3rd buffer"; break;			
		default:	  pStr = "no buffer written";	break;			
		}				
		CONSOLE_PutMessage(inst, "      direction: %u, %s\r\n", oSmStatus.b.three_buf_status, pStr);		
		CONSOLE_PutMessage(inst, "      read_buf_in_use: %u\r\n", oSmStatus.b.read_buf_in_use);					
		CONSOLE_PutMessage(inst, "      write_buf_in_use: %u\r\n", oSmStatus.b.write_buf_in_use);					
		
		addr = 0x0806 + (0x0008*i);
		HW_EscRead((unsigned long*)&oSmActivate . d8, (unsigned short)addr, (unsigned short)1);	
		CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SyncManager Activate\r\n", addr, oSmActivate.d8);		
		CONSOLE_PutMessage(inst, "      sm_enable: %u\r\n", oSmActivate.b.sm_enable);					
		CONSOLE_PutMessage(inst, "      repeat_request: %u\r\n", oSmActivate.b.repeat_request);							
		CONSOLE_PutMessage(inst, "      latch_event_ecat: %u\r\n", oSmActivate.b.latch_event_ecat);									
		CONSOLE_PutMessage(inst, "      latch_event_pdi: %u\r\n", oSmActivate.b.latch_event_pdi);											

	}

}  









 
static void clicmd_ShowEscDcInfo(CONSOLE_Inst *inst)
{
	uint16_t tmp16; 
	uint32_t tmp32, tmp32_p[2];		
	oDC_SYSTEM_TIME           oSystemTime;
	oDC_SYSTEM_TIME_DIFF      oSystemTimeDiff;
	oDC_SPEED_COUNTER_START   oSpeedCounterStart;
	oDC_SYSTIME_DIFF_FILTER_DEPTH   oSystimeDiffFilterDepth;
	oDC_SPEED_COUNTER_FILTER_DEPTH  oSpeedCntFilterDepth;
	oDC_REV_TIME_LATCH_MODE         oRevTimeLatchMode;
	oDC_CYCLIC_UNIT_CTRL            oCyclicUnitControl;
	oDC_SYNC_ACTIVATION             oSyncActivation;
	oDC_ACTIVATION_STATUS           oActivationStatus;
	oDC_SYNCx_STATUS                oSyncXStatus;
	oDC_LATCHx_CONTROL              oLatchControl;
	oDC_LATCHx_STATUS               oLatchStatus;
	
	CONSOLE_PutMessage(inst, "\r\nESC DC Info...\r\n");

	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x0900, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 0\r\n", 0x0900, tmp32);	
	
	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x0904, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 1\r\n", 0x0904, tmp32);	

	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x0908, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, Receive Time Port 2\r\n", 0x0908, tmp32);	

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x0918, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Receive Time ECAT Processing Unit\r\n"
													, 0x0918, tmp32_p[1], tmp32_p[0]);
													
	HW_EscRead((unsigned long*)&oSystemTime . d32[0], (unsigned short)0x0910, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time\r\n", 0x0910, oSystemTime.d32[1], oSystemTime.d32[0]);
	CONSOLE_PutMessage(inst, "      value_compare_with_sys_time: %uns\r\n", oSystemTime.b.value_compare_with_systim);	
	CONSOLE_PutMessage(inst, "      local_copy_of_sys_time: %uns\r\n", oSystemTime.b.local_copy_of_systim);		

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x0920, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time Offset\r\n", 0x0920, tmp32_p[1], tmp32_p[0]);

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x0928, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, System Time Delay\r\n", 0x0928, tmp32_p[1], tmp32_p[0]);

	HW_EscRead((unsigned long*)&oSystemTimeDiff . d32, (unsigned short)0x092C, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: %ld, System Time Diff\r\n", 0x092C, oSystemTimeDiff.d32);

	HW_EscRead((unsigned long*)&oSpeedCounterStart . d16, (unsigned short)0x0930, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, System Counter Start\r\n", 0x0930, oSpeedCounterStart.d16);
	CONSOLE_PutMessage(inst, "      bw_for_adj_of_loc_copy_of_sys_time: %u\r\n", oSpeedCounterStart.b.bw_for_adj_of_loc_copy_of_systim);			
	
	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0932, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x, System Counter Diff\r\n", 0x0932, tmp16);
	
	HW_EscRead((unsigned long*)&oSystimeDiffFilterDepth . d8, (unsigned short)0x0934, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, System Time Difference Filter Depth\r\n", 0x0934, oSystimeDiffFilterDepth.d8);
	CONSOLE_PutMessage(inst, "      filt_depth_for_avg_rev_sys_time_deva: %u\r\n", oSystimeDiffFilterDepth.b.filt_depth_for_avg_rev_systim_deva);				

	HW_EscRead((unsigned long*)&oSpeedCntFilterDepth . d8, (unsigned short)0x0935, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Speed Counter Filter Depth\r\n", 0x0935, oSpeedCntFilterDepth.d8);
	CONSOLE_PutMessage(inst, "      filt_depth_for_avg_clk_period_deva: %u\r\n", oSpeedCntFilterDepth.b.filt_depth_for_avg_clk_period_deva);				

	HW_EscRead((unsigned long*)&oRevTimeLatchMode . d8, (unsigned short)0x0936, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Receive Time Latch Mode\r\n", 0x0936, oRevTimeLatchMode.d8);
	CONSOLE_PutMessage(inst, "      rev_time_latch_mode: %u mode\r\n"
																	, oRevTimeLatchMode.b.rev_time_latch_mode
																	, (oRevTimeLatchMode.b.rev_time_latch_mode==0)?"Forwarding":"Reverse");				

	HW_EscRead((unsigned long*)&oCyclicUnitControl . d8, (unsigned short)0x0980, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Cyclic Unit Control\r\n", 0x0980, oCyclicUnitControl.d8);
	CONSOLE_PutMessage(inst, "      sync_out_unit_ctrl: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.sync_out_unit_ctrl
																	, (oCyclicUnitControl.b.sync_out_unit_ctrl==0)?"ECAT":"PDI");			
	CONSOLE_PutMessage(inst, "      latch_in_unit_0: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.latch_in_unit_0
																	, (oCyclicUnitControl.b.latch_in_unit_0==0)?"ECAT":"PDI");			
	CONSOLE_PutMessage(inst, "      latch_in_unit_1: %u, %s controlled\r\n"
																	, oCyclicUnitControl.b.latch_in_unit_1
																	, (oCyclicUnitControl.b.latch_in_unit_1==0)?"ECAT":"PDI");					
	
	HW_EscRead((unsigned long*)&oSyncActivation . d8, (unsigned short)0x0981, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Sync. Acviation\r\n", 0x0981, oSyncActivation.d8);
	CONSOLE_PutMessage(inst, "      sync_out_unit_act: %u, %s\r\n"
																	, oSyncActivation.b.sync_out_unit_act
																	, (oSyncActivation.b.sync_out_unit_act==0)?"Deactivated":"Activated");			
	CONSOLE_PutMessage(inst, "      sync0_gen: %u\r\n", oSyncActivation.b.sync0_gen);			
	CONSOLE_PutMessage(inst, "      sync1_gen: %u\r\n", oSyncActivation.b.sync1_gen);				
	CONSOLE_PutMessage(inst, "      auto_act_by_wr_start_time_cyc_op: %u\r\n", oSyncActivation.b.auto_act_by_wr_start_time_cyc_op);					
	CONSOLE_PutMessage(inst, "      ext_of_start_time_cyc_op: %u\r\n", oSyncActivation.b.ext_of_start_time_cyc_op);					
	CONSOLE_PutMessage(inst, "      start_time_plausibility_chk: %u\r\n", oSyncActivation.b.start_time_plausibility_chk);						
	CONSOLE_PutMessage(inst, "      near_future_configuration: %u\r\n", oSyncActivation.b.near_future_configuration);							
	CONSOLE_PutMessage(inst, "      sync_debug_pulse: %u\r\n", oSyncActivation.b.sync_debug_pulse);								

	HW_EscRead((unsigned long*)&tmp16, (unsigned short)0x0982, (unsigned short)2);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%04x(%uns), Pulse length of SyncSignals\r\n", 0x0982, tmp16, (tmp16*10));
	
	HW_EscRead((unsigned long*)&oActivationStatus . d8, (unsigned short)0x0984, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, Activation Status\r\n", 0x0984, oActivationStatus.d8);
	CONSOLE_PutMessage(inst, "      sync0_act_state: %u, First pulse %s pending\r\n"
																	, oActivationStatus.b.sync0_act_state
																	, (oActivationStatus.b.sync0_act_state == 0)?"not":"");				
	CONSOLE_PutMessage(inst, "      sync1_act_state: %u, First pulse %s pending\r\n"
																	, oActivationStatus.b.sync1_act_state
																	, (oActivationStatus.b.sync1_act_state == 0)?"not":"");					
	CONSOLE_PutMessage(inst, "      start_time_plausibility_chk_result: %u\r\n", oActivationStatus.b.start_time_plausibility_chk_result);						
	
	HW_EscRead((unsigned long*)&oSyncXStatus . d8, (unsigned short)0x098E, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SYNC0 Status\r\n", 0x098E, oSyncXStatus.d8);
	CONSOLE_PutMessage(inst, "      sync0_state_for_ack_mode: %u\r\n", oSyncXStatus.b.syncx_state_for_ack_mode);					

	HW_EscRead((unsigned long*)&oSyncXStatus . d8, (unsigned short)0x098F, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, SYNC1 Status\r\n", 0x098F, oSyncXStatus.d8);
	CONSOLE_PutMessage(inst, "      sync1_state_for_ack_mode: %u\r\n", oSyncXStatus.b.syncx_state_for_ack_mode);					

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x0990, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, Start Time Cyclic Operation\r\n"
																				, 0x0990, tmp32_p[1], tmp32_p[0]);

	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x0998, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), System time of next SYNC1\r\n", 0x0998, tmp32, tmp32);
	
	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x09A0, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), Time between two consecutive SYNC0 pulse\r\n", 0x09A0, tmp32, tmp32);

	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x09A4, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx(%luns), Time between SYNC0 pulse and SYNC1 pulse\r\n", 0x09A4, tmp32, tmp32);

	HW_EscRead((unsigned long*)&oLatchControl . d8, (unsigned short)0x09A8, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 0 Control\r\n", 0x09A8, oLatchControl.d8);
	CONSOLE_PutMessage(inst, "      latch0_positive_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_positive_edge == 0)?"Continue":"Single");
	CONSOLE_PutMessage(inst, "      latch0_negative_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_negative_edge == 0)?"Continue":"Single");					
	
	HW_EscRead((unsigned long*)&oLatchControl . d8, (unsigned short)0x09A9, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 1 Control\r\n", 0x09A9, oLatchControl.d8);	
	CONSOLE_PutMessage(inst, "      latch1_positive_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_positive_edge
																	, (oLatchControl.b.latchx_positive_edge == 0)?"Continue":"Single");
	CONSOLE_PutMessage(inst, "      latch1_negative_edge: %u, %s latched\r\n"
																	, oLatchControl.b.latchx_negative_edge
																	, (oLatchControl.b.latchx_negative_edge == 0)?"Continue":"Single");	
	
	HW_EscRead((unsigned long*)&oLatchStatus . d8, (unsigned short)0x09AE, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 0 Status\r\n", 0x09AE, oLatchStatus.d8);	
	CONSOLE_PutMessage(inst, "      event_latch0_positive_edge: %u, Positive edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_positive_edge == 0)?"not ":"");					
	CONSOLE_PutMessage(inst, "      event_latch0_negative_edge: %u, Negative edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_negative_edge == 0)?"not ":"");		
	CONSOLE_PutMessage(inst, "      latch0_pin_state: %u\r\n", oLatchStatus.b.latchx_pin_state);		

	HW_EscRead((unsigned long*)&oLatchStatus . d8, (unsigned short)0x09AF, (unsigned short)1);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%02x, LATCH 1 Status\r\n", 0x09AF, oLatchStatus.d8);	
	CONSOLE_PutMessage(inst, "      event_latch1_positive_edge: %u, Positive edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_positive_edge == 0)?"not ":"");					
	CONSOLE_PutMessage(inst, "      event_latch1_negative_edge: %u, Negative edge %s detected\r\n"
																	, oLatchStatus.b.event_latchx_positive_edge
																	, (oLatchStatus.b.event_latchx_negative_edge == 0)?"not ":"");		
	CONSOLE_PutMessage(inst, "      latch1_pin_state: %u\r\n", oLatchStatus.b.latchx_pin_state);		

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x09B0, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 0 Time Positive Edge\r\n", 0x09B0, tmp32_p[1], tmp32_p[0]);	
	
	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x09B8, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 0 Time Negative Edge\r\n", 0x09B8, tmp32_p[1], tmp32_p[0]);	

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x09C0, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Positive Edge\r\n", 0x09C0, tmp32_p[1], tmp32_p[0]);	

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x09C8, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Negative Edge\r\n", 0x09C8, tmp32_p[1], tmp32_p[0]);	

	HW_EscRead((unsigned long*)tmp32_p, (unsigned short)0x09C8, (unsigned short)8);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx%08lx, LATCH 1 Time Negative Edge\r\n", 0x09C8, tmp32_p[1], tmp32_p[0]);	
	
	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x09F0, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, ECAT Buffer Change Event Time\r\n", 0x09F0, tmp32);	

	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x09F8, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, PDI Buffer Start Event Time\r\n", 0x09F8, tmp32);	
	
	HW_EscRead((unsigned long*)&tmp32, (unsigned short)0x09FC, (unsigned short)4);	
	CONSOLE_PutMessage(inst, "0x%04x: 0x%08lx, PDI Buffer Change Event Time\r\n", 0x09FC, tmp32);		

}  









 
uint16_t CLICMD_GetCmdTableSize(void)
{
    return sizeof(CLICMD_userCmdTable)/sizeof(CONSOLE_CmdEntry);
}  









 
void *CLICMD_GetCmdTable(void)
{
    return ((void*)CLICMD_userCmdTable);
}  









 
static int clicmd_SystemReboot(CONSOLE_Inst *inst, int argc, char **argv)
{
	NVIC_SystemReset();
	return 1;
}  









 
static int clicmd_SystemRebootHelp(CONSOLE_Inst *inst)
{
	
	CONSOLE_PutMessage(inst, "        reboot: System reboot\r\n");
	return 1;
}  









 
static int clicmd_SystemRebootUsage(CONSOLE_Inst *inst)
{
	
	clicmd_SystemRebootHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: reboot [-tp DecValue]\r\n");
	return 1;
}  









 
static int clicmd_PdiRead(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, ByteLen;
	
	if (argc == 1)
	{
		clicmd_PdiReadUsage(inst);
		return 1;
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}

	if (clicmd_HexText2Long((uint8_t*)argv[2], &ByteLen, strlen(argv[2])) != 0)
	{
		return -1;
	}
		
	CONSOLE_PutMessage(inst, "Start byte read, addr=0x%x, len=0x%x byte use ESC chip select\r\n", HexOffset, ByteLen);
	memset(StrBuf, 0, ByteLen);
	HW_EscRead((unsigned long*)StrBuf, HexOffset,  ByteLen);
	clicmd_ShowMemoryInHex8b(inst, StrBuf, ByteLen);		
	return 1;
}  









 
static int clicmd_PdiReadHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdird: Process data interface reading\r\n");
	return 1;
}  









 
static int clicmd_PdiReadUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiReadHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdird <HexOffset> <HexByteLen>\r\n");

	return 1;
}  









 
static int clicmd_PdiWrite(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, i, ByteLen;
	uint8_t *pValue8b;
	
	if (argc == 1)
	{
		clicmd_PdiWriteUsage(inst);
		return 1;
	}
	else if (argc < 3)
	{
		return -1;
	}

	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0) 
	{
		return -1;
	}
	
	ByteLen = argc - 2;
		
	pValue8b = StrBuf;
	for (i=0; i<ByteLen; i++)
	{
		if (clicmd_HexText2Char((uint8_t*)argv[i+2], pValue8b, strlen(argv[i+2])) != 0)
		{
			return -1;
		}
		pValue8b++;				
	}
	
	CONSOLE_PutMessage(inst, "Start byte write, addr=0x%x, len=0x%x byte use ESC chip select\r\n", HexOffset, ByteLen);
	HW_EscWrite((unsigned long*)StrBuf, HexOffset,  ByteLen);			
	clicmd_ShowMemoryInHex8b(inst, StrBuf, ByteLen);		

	return 1;
}  









 
static int clicmd_PdiWriteHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdiwr: Process data interface writing\r\n");
	return 1;
}  









 
static int clicmd_PdiWriteUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiWriteHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdiwr <HexOffset> <HexValue 0> <HexValue 1>..<HexValue N>]\r\n");
	return 1;
}  










 
static int clicmd_SysRegisterDisplay(CONSOLE_Inst *inst)
{
	SYS_T *SysOffset = 0;
	
	 	
	CONSOLE_PutMessage(inst, "Current SYS registers...\r\n");	
	CONSOLE_PutMessage(inst, "PDID(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PDID), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->PDID);
	CONSOLE_PutMessage(inst, "RSTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->RSTSTS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->RSTSTS);
	CONSOLE_PutMessage(inst, "IPRST0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST0), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IPRST0);
	CONSOLE_PutMessage(inst, "IPRST1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST1), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IPRST1);
	CONSOLE_PutMessage(inst, "IPRST2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IPRST2), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IPRST2);	
	CONSOLE_PutMessage(inst, "BODCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->BODCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->BODCTL);
	CONSOLE_PutMessage(inst, "IVSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IVSCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IVSCTL);
	CONSOLE_PutMessage(inst, "PORCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PORCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->PORCTL);
	CONSOLE_PutMessage(inst, "VREFCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->VREFCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->VREFCTL);
	CONSOLE_PutMessage(inst, "USBPHY(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->USBPHY), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->USBPHY);
	CONSOLE_PutMessage(inst, "GPA_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPA_MFPL);
	CONSOLE_PutMessage(inst, "GPA_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPA_MFPH);
	CONSOLE_PutMessage(inst, "GPB_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPB_MFPL);
	CONSOLE_PutMessage(inst, "GPB_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPB_MFPH);
	CONSOLE_PutMessage(inst, "GPC_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPC_MFPL);
	CONSOLE_PutMessage(inst, "GPC_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPC_MFPH);
	CONSOLE_PutMessage(inst, "GPD_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPD_MFPL);
	CONSOLE_PutMessage(inst, "GPD_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPD_MFPH);
	CONSOLE_PutMessage(inst, "GPE_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPE_MFPL);
	CONSOLE_PutMessage(inst, "GPE_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPE_MFPH);
	CONSOLE_PutMessage(inst, "GPF_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPF_MFPL);
	CONSOLE_PutMessage(inst, "GPF_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPF_MFPH);
	CONSOLE_PutMessage(inst, "GPG_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPG_MFPL);
	CONSOLE_PutMessage(inst, "GPG_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPG_MFPH);
	CONSOLE_PutMessage(inst, "GPH_MFPL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFPL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPH_MFPL);
	CONSOLE_PutMessage(inst, "GPH_MFPH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFPH), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPH_MFPH);
	CONSOLE_PutMessage(inst, "GPA_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPA_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPA_MFOS);
	CONSOLE_PutMessage(inst, "GPB_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPB_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPB_MFOS);
	CONSOLE_PutMessage(inst, "GPC_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPC_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPC_MFOS);
	CONSOLE_PutMessage(inst, "GPD_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPD_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPD_MFOS);
	CONSOLE_PutMessage(inst, "GPE_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPE_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPE_MFOS);
	CONSOLE_PutMessage(inst, "GPF_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPF_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPF_MFOS);
	CONSOLE_PutMessage(inst, "GPG_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPG_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPG_MFOS);
	CONSOLE_PutMessage(inst, "GPH_MFOS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->GPH_MFOS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->GPH_MFOS);
	CONSOLE_PutMessage(inst, "SRAM_INTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_INTCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->SRAM_INTCTL);
	CONSOLE_PutMessage(inst, "SRAM_STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_STATUS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->SRAM_STATUS);
	CONSOLE_PutMessage(inst, "SRAM_ERRADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_ERRADDR), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->SRAM_ERRADDR);
	CONSOLE_PutMessage(inst, "SRAM_BISTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_BISTCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->SRAM_BISTCTL);
	CONSOLE_PutMessage(inst, "SRAM_BISTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->SRAM_BISTSTS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->SRAM_BISTSTS);	
	CONSOLE_PutMessage(inst, "IRCTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IRCTCTL);
	CONSOLE_PutMessage(inst, "IRCTIEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTIEN), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IRCTIEN);
	CONSOLE_PutMessage(inst, "IRCTISTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->IRCTISTS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->IRCTISTS);
	CONSOLE_PutMessage(inst, "REGLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->REGLCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL);
	CONSOLE_PutMessage(inst, "PLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PLCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->PLCTL);
	CONSOLE_PutMessage(inst, "PLSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->PLSTS), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->PLSTS);
	CONSOLE_PutMessage(inst, "AHBMCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SysOffset->AHBMCTL), ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->AHBMCTL);

	return 0;
}









 
static int clicmd_SysRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_SysRegisterUsage(inst);
		clicmd_SysRegisterDisplay(inst);
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	 
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	 	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}

	 
	*((uint32_t *)(((uint32_t)((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write SYS Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
}  









 
static int clicmd_SysRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           sys: Access MCU SYS registers\r\n");
	return 1;
}  









 
static int clicmd_SysRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SysRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: sys [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_ClkRegisterDisplay(CONSOLE_Inst *inst)
{
	CLK_T *ClkOffset = 0;
	
	 	
	CONSOLE_PutMessage(inst, "Current CLK registers...\r\n");	
	CONSOLE_PutMessage(inst, "PWRCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PWRCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PWRCTL);
	CONSOLE_PutMessage(inst, "AHBCLK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->AHBCLK), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->AHBCLK);
	CONSOLE_PutMessage(inst, "APBCLK0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->APBCLK0), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->APBCLK0);	
	CONSOLE_PutMessage(inst, "APBCLK1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->APBCLK1), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->APBCLK1);
	CONSOLE_PutMessage(inst, "CLKSEL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL0), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKSEL0);
	CONSOLE_PutMessage(inst, "CLKSEL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL1), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKSEL1);
	CONSOLE_PutMessage(inst, "CLKSEL2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL2), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKSEL2);
	CONSOLE_PutMessage(inst, "CLKSEL3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKSEL3), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKSEL3);	
	CONSOLE_PutMessage(inst, "CLKDIV0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV0), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDIV0);
	CONSOLE_PutMessage(inst, "CLKDIV1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV1), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDIV1);
	CONSOLE_PutMessage(inst, "CLKDIV3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV3), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDIV3);
	CONSOLE_PutMessage(inst, "CLKDIV4(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDIV4), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDIV4);
	CONSOLE_PutMessage(inst, "PCLKDIV(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PCLKDIV), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PCLKDIV);	
	CONSOLE_PutMessage(inst, "PLLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PLLCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PLLCTL);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->STATUS), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->STATUS);
	CONSOLE_PutMessage(inst, "CLKOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKOCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKOCTL);
	CONSOLE_PutMessage(inst, "CLKDCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDCTL);
	CONSOLE_PutMessage(inst, "CLKDSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CLKDSTS), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CLKDSTS);	
	CONSOLE_PutMessage(inst, "CDUPB(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CDUPB), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CDUPB);
	CONSOLE_PutMessage(inst, "CDLOWB(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->CDLOWB), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->CDLOWB);
	CONSOLE_PutMessage(inst, "PMUCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PMUCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PMUCTL);
	CONSOLE_PutMessage(inst, "PMUSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PMUSTS), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PMUSTS);
	CONSOLE_PutMessage(inst, "LDOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->LDOCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->LDOCTL);	
	CONSOLE_PutMessage(inst, "SWKDBCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->SWKDBCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->SWKDBCTL);
	CONSOLE_PutMessage(inst, "PASWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PASWKCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PASWKCTL);
	CONSOLE_PutMessage(inst, "PBSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PBSWKCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PBSWKCTL);
	CONSOLE_PutMessage(inst, "PCSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PCSWKCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PCSWKCTL);
	CONSOLE_PutMessage(inst, "PDSWKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->PDSWKCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->PDSWKCTL);	
	CONSOLE_PutMessage(inst, "IOPDCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ClkOffset->IOPDCTL), ((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))->IOPDCTL);
		
	CONSOLE_PutMessage(inst, "\r\nCurrent Clock Information...\r\n");
	CONSOLE_PutMessage(inst, "HXT_Frequency = %d Hz\r\n", CLK_GetHXTFreq());
	CONSOLE_PutMessage(inst, "LXT_Frequency = %d Hz\r\n", CLK_GetLXTFreq());
	CONSOLE_PutMessage(inst, "HCLK_Frequency = %d Hz\r\n", CLK_GetHCLKFreq());
	CONSOLE_PutMessage(inst, "PCLK0_Frequency = %d Hz\r\n", CLK_GetPCLK0Freq());		
	CONSOLE_PutMessage(inst, "PCLK1_Frequency = %d Hz\r\n", CLK_GetPCLK1Freq());
	CONSOLE_PutMessage(inst, "CPU_Frequency = %d Hz\r\n", CLK_GetCPUFreq());
	CONSOLE_PutMessage(inst, "PLL_Frequency = %d Hz\r\n", CLK_GetPLLClockFreq());

	CONSOLE_PutMessage(inst, "PMUWK_Source = %d\r\n", CLK_GetPMUWKSrc());
	CONSOLE_PutMessage(inst, "TIMER2_ClockSource = %d\r\n", CLK_GetModuleClockSource(((1UL<<30)|(1UL<<28) |(0x7UL<<25) |(16UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(4UL<<0))));
	CONSOLE_PutMessage(inst, "TIMER2_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(((1UL<<30)|(1UL<<28) |(0x7UL<<25) |(16UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(4UL<<0))));	
	CONSOLE_PutMessage(inst, "TIMER3_ClockSource = %d\r\n", CLK_GetModuleClockSource(((1UL<<30)|(1UL<<28) |(0x7UL<<25) |(20UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(5UL<<0))));
	CONSOLE_PutMessage(inst, "TIMER3_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(((1UL<<30)|(1UL<<28) |(0x7UL<<25) |(20UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(5UL<<0))));
	CONSOLE_PutMessage(inst, "SPI0_ClockSource = %d\r\n", CLK_GetModuleClockSource(((1UL<<30)|(2UL<<28) |(0x3UL<<25) |(4UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(13UL<<0))));
	CONSOLE_PutMessage(inst, "SPI0_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(((1UL<<30)|(2UL<<28) |(0x3UL<<25) |(4UL<<20) |(0x0UL<<18)|(0x0UL<<10)|(0x0UL<<5)|(13UL<<0))));
	CONSOLE_PutMessage(inst, "UART2_ClockSource = %d\r\n", CLK_GetModuleClockSource(((1UL<<30)|(3UL<<28) |(0x3UL<<25) |(24UL<<20) |(3UL<<18) |(0xFUL<<10) |(0UL<<5) |(18UL<<0))));
	CONSOLE_PutMessage(inst, "UART2_ClockDivider = %d\r\n", CLK_GetModuleClockDivider(((1UL<<30)|(3UL<<28) |(0x3UL<<25) |(24UL<<20) |(3UL<<18) |(0xFUL<<10) |(0UL<<5) |(18UL<<0))));

	return 0;
}









 
static int clicmd_ClkRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_ClkRegisterUsage(inst);
		clicmd_ClkRegisterDisplay(inst);
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	 
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	 	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}

	 
	*((uint32_t *)(((uint32_t)((CLK_T *) (((uint32_t)0x40000000) + 0x00200UL))) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write CLK Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
}  









 
static int clicmd_ClkRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           clk: Access MCU CLK registers\r\n");
	return 1;
}  









 
static int clicmd_ClkRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_ClkRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: clk [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_FmcRegisterDisplay(CONSOLE_Inst *inst)
{
	FMC_T *FmcOffset = 0;	
	uint32_t config[2];

	 	
	CONSOLE_PutMessage(inst, "Current FMC registers...\r\n");	
	CONSOLE_PutMessage(inst, "ISPCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPCTL), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCTL);
	CONSOLE_PutMessage(inst, "ISPADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPADDR), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR);
	CONSOLE_PutMessage(inst, "ISPDAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPDAT), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT);
	CONSOLE_PutMessage(inst, "ISPCMD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPCMD), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD);
	CONSOLE_PutMessage(inst, "ISPTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPTRG), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG);
	CONSOLE_PutMessage(inst, "DFBA(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->DFBA), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->DFBA);
	CONSOLE_PutMessage(inst, "ISPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->ISPSTS), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPSTS);
	CONSOLE_PutMessage(inst, "CYCCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->CYCCTL), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->CYCCTL);
	CONSOLE_PutMessage(inst, "KPKEY0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY0), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEY0);
	CONSOLE_PutMessage(inst, "KPKEY1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY1), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEY1);
	CONSOLE_PutMessage(inst, "KPKEY2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEY2), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEY2);
	CONSOLE_PutMessage(inst, "KPKEYTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYTRG), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEYTRG);
	CONSOLE_PutMessage(inst, "KPKEYSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYSTS), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEYSTS);
	CONSOLE_PutMessage(inst, "KPKEYCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPKEYCNT), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPKEYCNT);
	CONSOLE_PutMessage(inst, "KPCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->KPCNT), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->KPCNT);
	CONSOLE_PutMessage(inst, "MPDAT0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT0), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPDAT0);
	CONSOLE_PutMessage(inst, "MPDAT1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT1), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPDAT1);
	CONSOLE_PutMessage(inst, "MPDAT2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT2), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPDAT2);
	CONSOLE_PutMessage(inst, "MPDAT3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPDAT3), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPDAT3);
	CONSOLE_PutMessage(inst, "MPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPSTS), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPSTS);
	CONSOLE_PutMessage(inst, "MPADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(FmcOffset->MPADDR), ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->MPADDR);

	CONSOLE_PutMessage(inst, "\r\nOther information...\r\n");	
	CONSOLE_PutMessage(inst, "APROM base address: 0x%08lx\r\n", 0x00000000UL);
	CONSOLE_PutMessage(inst, "APROM end address: 0x%08lx\r\n", 0x00080000UL);		
	CONSOLE_PutMessage(inst, "APROM bank0 end address: 0x%08lx\r\n", (0x00080000UL/2UL));	
	CONSOLE_PutMessage(inst, "LDROM base address: 0x%08lx\r\n", 0x00100000UL);	
	CONSOLE_PutMessage(inst, "LDROM end address: 0x%08lx\r\n", 0x00101000UL);	
	CONSOLE_PutMessage(inst, "SPROM base address: 0x%08lx\r\n", 0x00200000UL);	
	CONSOLE_PutMessage(inst, "SPROM end address: 0x%08lx\r\n", 0x00201000UL);	
	CONSOLE_PutMessage(inst, "User Configuration address: 0x%08lx\r\n", 0x00300000UL);	
	CONSOLE_PutMessage(inst, "User Config 0 address: 0x%08lx\r\n", 0x00300000UL);	
	CONSOLE_PutMessage(inst, "User Config 1 address: 0x%08lx\r\n", 0x00300004UL);	
	CONSOLE_PutMessage(inst, "User Config 2 address: 0x%08lx\r\n", 0x00300008UL);	
	FMC_ReadConfig(config, 2);	
	CONSOLE_PutMessage(inst, "User Configuration: 0x%08lx%08lx\r\n", config[0], config[1]);	
	CONSOLE_PutMessage(inst, "Security ROM base address: 0x%08lx\r\n", 0x00301000UL);	
	CONSOLE_PutMessage(inst, "OTP flash base address: 0x%08lx\r\n", 0x00310000UL);	
	CONSOLE_PutMessage(inst, "Flash Page Size (4K bytes): 0x%08lx\r\n", 0x1000UL);	
	CONSOLE_PutMessage(inst, "APROM Size: 0x%08lx\r\n", 0x00080000UL);	
	CONSOLE_PutMessage(inst, "APROM Bank Size: 0x%08lx\r\n", (0x00080000UL/2UL));	
	CONSOLE_PutMessage(inst, "LDROM Size (4 Kbytes): 0x%08lx\r\n", 0x1000UL);	
	CONSOLE_PutMessage(inst, "SPROM Size (4 Kbytes): 0x%08lx\r\n", 0x1000UL);	
	CONSOLE_PutMessage(inst, "OTP entry number: 0x%08lx\r\n", 256UL);	
	
	CONSOLE_PutMessage(inst, "Company ID/CID: 0x%08lx\r\n", FMC_ReadCID());
	CONSOLE_PutMessage(inst, "Product ID/PID: 0x%08lx\r\n", FMC_ReadPID());
	CONSOLE_PutMessage(inst, "Unique ID/UID[2..0]: 0x%08lx%08lx%08lx\r\n", FMC_ReadUID(2), FMC_ReadUID(1), FMC_ReadUID(0));	
	CONSOLE_PutMessage(inst, "UCID[3..0]: 0x%08lx%08lx%08lx%08lx\r\n", FMC_ReadUCID(3), FMC_ReadUCID(2), FMC_ReadUCID(1), FMC_ReadUCID(0));	
	CONSOLE_PutMessage(inst, "Vector mapping address/VECMAP: 0x%08lx\r\n", FMC_GetVECMAP());	
	CONSOLE_PutMessage(inst, "Boot source: %s\r\n", FMC_GetBootSource() ? "LDROM":"APROM");

	return 0;
}









 
static int clicmd_FmcRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	
	if (argc == 1)
	{
		clicmd_FmcRegisterUsage(inst);
		clicmd_FmcRegisterDisplay(inst);
		return 1;		
	}
	
	if (argc == 3)
	{
			 
			if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
			{
				return -1;
			}
			
			 	
			if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
			{
				return -1;
			}
			
			 
			*((uint32_t *)(((uint32_t)((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))) + HexOffset)) = HexValue;

			CONSOLE_PutMessage(inst, "Write FMC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
			return 1;
	}

	return -1;
}  









 
static int clicmd_FmcRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           fmc: Access MCU FMC registers\r\n");
	return 1;
}  









 
static int clicmd_FmcRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_FmcRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: fmc [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_SpimRegisterDisplay(CONSOLE_Inst *inst)
{
	SPIM_T *SpimOffset = 0;

	 	
	CONSOLE_PutMessage(inst, "Current SPIM registers...\r\n");	
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->CTL0), ((volatile SPIM_T *) (0x40007000UL))->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->CTL1), ((volatile SPIM_T *) (0x40007000UL))->CTL1);
	CONSOLE_PutMessage(inst, "RXCLKDLY(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RXCLKDLY), ((volatile SPIM_T *) (0x40007000UL))->RXCLKDLY);
	CONSOLE_PutMessage(inst, "RX[0](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[0]), ((volatile SPIM_T *) (0x40007000UL))->RX[0]);	
	CONSOLE_PutMessage(inst, "RX[1](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[1]), ((volatile SPIM_T *) (0x40007000UL))->RX[1]);	
	CONSOLE_PutMessage(inst, "RX[2](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[2]), ((volatile SPIM_T *) (0x40007000UL))->RX[2]);	
	CONSOLE_PutMessage(inst, "RX[3](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->RX[3]), ((volatile SPIM_T *) (0x40007000UL))->RX[3]);		
	CONSOLE_PutMessage(inst, "TX[0](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[0]), ((volatile SPIM_T *) (0x40007000UL))->TX[0]);	
	CONSOLE_PutMessage(inst, "TX[1](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[1]), ((volatile SPIM_T *) (0x40007000UL))->TX[1]);	
	CONSOLE_PutMessage(inst, "TX[2](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[2]), ((volatile SPIM_T *) (0x40007000UL))->TX[2]);	
	CONSOLE_PutMessage(inst, "TX[3](0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->TX[3]), ((volatile SPIM_T *) (0x40007000UL))->TX[3]);			
	CONSOLE_PutMessage(inst, "SRAMADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->SRAMADDR), ((volatile SPIM_T *) (0x40007000UL))->SRAMADDR);
	CONSOLE_PutMessage(inst, "DMACNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->DMACNT), ((volatile SPIM_T *) (0x40007000UL))->DMACNT);	
	CONSOLE_PutMessage(inst, "FADDR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->FADDR), ((volatile SPIM_T *) (0x40007000UL))->FADDR);
	CONSOLE_PutMessage(inst, "KEY1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->KEY1), ((volatile SPIM_T *) (0x40007000UL))->KEY1);
	CONSOLE_PutMessage(inst, "KEY2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->KEY2), ((volatile SPIM_T *) (0x40007000UL))->KEY2);
	CONSOLE_PutMessage(inst, "DMMCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SpimOffset->DMMCTL), ((volatile SPIM_T *) (0x40007000UL))->DMMCTL);
	CONSOLE_PutMessage(inst, "CTL2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(((volatile SPIM_T *) (0x40007000UL))->CTL2), ((volatile SPIM_T *) (0x40007000UL))->CTL2);

	return 0;
}









 
static int clicmd_SpimRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t i, j, HexOffset, HexLength, HexValue;
	
	if (argc == 1)
	{
		clicmd_SpimRegisterUsage(inst);
		clicmd_SpimRegisterDisplay(inst);
		return 1;		
	}
	
	for (i=1; i<argc; i++)
	{
		if (!strcmp(argv[i], "-rd") && strlen(argv[i])==3)
		{
			 			
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}			
			
			 
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexLength, strlen(argv[i])) != 0)
			{
				return -1;
			}			
			
			CONSOLE_PutMessage(inst, "Read extFlash, offset: 0x%08lx, length: %u\r\n", HexOffset, HexLength);
			
			SPIM_IO_Read(HexOffset, 0, HexLength, StrBuf, 0x0bU, 1, 1, 1, 1);				
			clicmd_ShowMemoryInHex8b(inst, StrBuf, HexLength);
			return 1;
		}
		else if (!strcmp(argv[i], "-wr") && strlen(argv[i])==3)
		{
			 
			i++;
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}
			
			 
			i++;
			if (i>=argc)
			{
				return -1;
			}
			for (j=0;(i+j)<argc;j++)
			{
				if (clicmd_HexText2Char((uint8_t*)argv[i+j], &StrBuf[j], strlen(argv[i+j])) != 0)
				{
					break;
				}
			}
			i+=j;
			
			 
			CONSOLE_PutMessage(inst, "Erase SPI flash block 0x%x...", HexOffset);
			SPIM_EraseBlock(HexOffset, 0, 0xd8U, 1, 1);
			CONSOLE_PutMessage(inst, "done.\r\n");
			
			 
			CONSOLE_PutMessage(inst, "Program 0x%x bytes to SPI flash from 0x%x...", j, HexOffset);			
      SPIM_IO_Write(HexOffset, 0, j, StrBuf, 0x02U, 1, 1, 1);
			CONSOLE_PutMessage(inst, "done.\r\n");
			
			return 1;
		}
		else
		{
			 
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexOffset, strlen(argv[i])) != 0)
			{
				return -1;
			}
			i++;
			
			 	
			if (clicmd_HexText2Long((uint8_t*)argv[i], &HexValue, strlen(argv[i])) != 0)
			{
				return -1;
			}
			i++;
			
			 
			*((uint32_t *)(((uint32_t)((volatile SPIM_T *) (0x40007000UL))) + HexOffset)) = HexValue;

			CONSOLE_PutMessage(inst, "Write SPIM Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
			return 1;
		}
	}
	
	return -1;
}  









 
static int clicmd_SpimRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          spim: Access MCU SPIM registers\r\n");
	return 1;
}  









 
static int clicmd_SpimRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SpimRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: spim [<HexOffset> <HexValue>] [-rd HexOffset HexLength]  [-wr HexOffset HexData0...HexDataN]\r\n");
	CONSOLE_PutMessage(inst, "       -rd: Read data from external SPI Flash.\r\n");
	CONSOLE_PutMessage(inst, "       -wr: Write data to external SPI Flash.\r\n");	
	return 1;
}  









 
static int clicmd_UARTxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, UART_T* UARTx)
{
	UART_T *UARTxOffset = 0;

	 	
	CONSOLE_PutMessage(inst, "Current UART%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "DAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->DAT), UARTx->DAT);
	CONSOLE_PutMessage(inst, "INTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->INTEN), UARTx->INTEN);
	CONSOLE_PutMessage(inst, "FIFO(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FIFO), UARTx->FIFO);
	CONSOLE_PutMessage(inst, "LINE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINE), UARTx->LINE);
	CONSOLE_PutMessage(inst, "MODEM(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->MODEM), UARTx->MODEM);
	CONSOLE_PutMessage(inst, "MODEMSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->MODEMSTS), UARTx->MODEMSTS);
	CONSOLE_PutMessage(inst, "FIFOSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FIFOSTS), UARTx->FIFOSTS);
	CONSOLE_PutMessage(inst, "INTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->INTSTS), UARTx->INTSTS);
	CONSOLE_PutMessage(inst, "TOUT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->TOUT), UARTx->TOUT);
	CONSOLE_PutMessage(inst, "BAUD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->BAUD), UARTx->BAUD);	
	CONSOLE_PutMessage(inst, "IRDA(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->IRDA), UARTx->IRDA);	
	CONSOLE_PutMessage(inst, "ALTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->ALTCTL), UARTx->ALTCTL);	
	CONSOLE_PutMessage(inst, "FUNCSEL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->FUNCSEL), UARTx->FUNCSEL);	
	CONSOLE_PutMessage(inst, "LINCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINCTL), UARTx->LINCTL);	
	CONSOLE_PutMessage(inst, "LINSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->LINSTS), UARTx->LINSTS);	
	CONSOLE_PutMessage(inst, "BRCOMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->BRCOMP), UARTx->BRCOMP);	
	CONSOLE_PutMessage(inst, "WKCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->WKCTL), UARTx->WKCTL);		
	CONSOLE_PutMessage(inst, "WKSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->WKSTS), UARTx->WKSTS);		
	CONSOLE_PutMessage(inst, "DWKCOMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(UARTxOffset->DWKCOMP), UARTx->DWKCOMP);			

	return 0;
}









 
static int clicmd_UARTxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	UART_T *UARTx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x30000UL));
			break;
		case 1:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x31000UL));
			break;
		case 2:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x32000UL));
			break;
		case 3:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x33000UL));
			break;
		case 4:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x34000UL));
			break;
		case 5:
			UARTx = ((UART_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x35000UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_UARTxRegisterUsage(inst);
			clicmd_UARTxRegisterDisplay(inst, HexIndex, UARTx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)UARTx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write UART%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_UARTxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         uartx: Access MCU UARTx reigisters\r\n");
	return 1;
}  









 
static int clicmd_UARTxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_UARTxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: uartx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_TMRxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, TIMER_T* TMRx)
{
	TIMER_T *TMRxOffset = 0;
	
	 
	CONSOLE_PutMessage(inst, "Current TMR%u clock = %uHz, registers...\r\n", index, TIMER_GetModuleClock(TMRx));
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CTL), TMRx->CTL);
	CONSOLE_PutMessage(inst, "CMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CMP), TMRx->CMP);
	CONSOLE_PutMessage(inst, "INTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->INTSTS), TMRx->INTSTS);
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CNT), TMRx->CNT);	
	CONSOLE_PutMessage(inst, "CAP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->CAP), TMRx->CAP);	
	CONSOLE_PutMessage(inst, "EXTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->EXTCTL), TMRx->EXTCTL);	
	CONSOLE_PutMessage(inst, "EINTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->EINTSTS), TMRx->EINTSTS);	
	CONSOLE_PutMessage(inst, "TRGCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->TRGCTL), TMRx->TRGCTL);	
	CONSOLE_PutMessage(inst, "ALTCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(TMRxOffset->ALTCTL), TMRx->ALTCTL);	
	return 0;
}









 
static int clicmd_TMRxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	TIMER_T *TMRx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			TMRx = ((TIMER_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x10000UL));
			break;
		case 1:
			TMRx = ((TIMER_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x10100UL));
			break;
		case 2:
			TMRx = ((TIMER_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x11000UL));
			break;
		case 3:
			TMRx = ((TIMER_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x11100UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_TMRxRegisterUsage(inst);
			clicmd_TMRxRegisterDisplay(inst, HexIndex, TMRx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)TMRx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write TMR%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_TMRxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          tmrx: Access MCU TMRx registers\r\n");
	return 1;
}  









 
static int clicmd_TMRxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_TMRxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: tmrx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_SPIxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, SPI_T* SPIx)
{
	SPI_T *SPIxOffset = 0;
	
	 
	CONSOLE_PutMessage(inst, "Current SPI%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->CTL), SPIx->CTL);	
	CONSOLE_PutMessage(inst, "CLKDIV(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->CLKDIV), SPIx->CLKDIV);		
	CONSOLE_PutMessage(inst, "SSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->SSCTL), SPIx->SSCTL);			
	CONSOLE_PutMessage(inst, "PDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->PDMACTL), SPIx->PDMACTL);				
	CONSOLE_PutMessage(inst, "FIFOCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->FIFOCTL), SPIx->FIFOCTL);				
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->STATUS), SPIx->STATUS);
	CONSOLE_PutMessage(inst, "TX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->TX), SPIx->TX);
	CONSOLE_PutMessage(inst, "RX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->RX), SPIx->RX);				
	CONSOLE_PutMessage(inst, "I2SCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SCTL), SPIx->I2SCTL);				
	CONSOLE_PutMessage(inst, "I2SCLK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SCLK), SPIx->I2SCLK);				
	CONSOLE_PutMessage(inst, "I2SSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(SPIxOffset->I2SSTS), SPIx->I2SSTS);				
	return 0;
}









 
static int clicmd_SPIxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	SPI_T *SPIx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			SPIx = ((SPI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x21000UL));
			break;
		case 1:
			SPIx = ((SPI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x22000UL));
			break;
		case 2:
			SPIx = ((SPI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x23000UL));
			break;
		case 3:
			SPIx = ((SPI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x24000UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_SPIxRegisterUsage(inst);
			clicmd_SPIxRegisterDisplay(inst, HexIndex, SPIx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)SPIx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write SPI%01d Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_SPIxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          spix: Access MCU SPIx registers\r\n");
	return 1;
}  









 
static int clicmd_SPIxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_SPIxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: spix <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_GPIOxRegisterDisplay(CONSOLE_Inst *inst, uint8_t Port, GPIO_T *GPIOx)
{
	GPIO_T *GPIOxOffet = 0;
	
	CONSOLE_PutMessage(inst, "Current GPIO%c(0x%08lx) registers...\r\n", Port, (uint32_t)GPIOx);			
	CONSOLE_PutMessage(inst, "MODE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->MODE), GPIOx->MODE);
	CONSOLE_PutMessage(inst, "DINOFF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DINOFF), GPIOx->DINOFF);
	CONSOLE_PutMessage(inst, "DOUT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DOUT), GPIOx->DOUT);
	CONSOLE_PutMessage(inst, "DATMSK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DATMSK), GPIOx->DATMSK);
	CONSOLE_PutMessage(inst, "PIN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->PIN), GPIOx->PIN);
	CONSOLE_PutMessage(inst, "DBEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->DBEN), GPIOx->DBEN);
	CONSOLE_PutMessage(inst, "INTTYPE(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTTYPE), GPIOx->INTTYPE);
	CONSOLE_PutMessage(inst, "INTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTEN), GPIOx->INTEN);
	CONSOLE_PutMessage(inst, "INTSRC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->INTSRC), GPIOx->INTSRC);
	CONSOLE_PutMessage(inst, "SMTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->SMTEN), GPIOx->SMTEN);
	CONSOLE_PutMessage(inst, "SLEWCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->SLEWCTL), GPIOx->SLEWCTL);
	CONSOLE_PutMessage(inst, "PUSEL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(GPIOxOffet->PUSEL), GPIOx->PUSEL);

	return 0;
}









 
static int clicmd_GPIOxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexPort;
	GPIO_T *GPIOx;
	
	if (argc == 1)
	{
		clicmd_GPIOxRegisterUsage(inst);
		return 1;		
	}
	else if (argc == 2)
	{
		HexPort = argv[1][0] & 0xdf;
	
		switch (HexPort)
		{
		case 'A': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04000UL); break;
		case 'B': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04040UL); break;
		case 'C': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04080UL); break;
		case 'D': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x040C0UL); break;
		case 'E': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04100UL); break;
		case 'F': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04140UL); break;
		case 'G': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04180UL); break;
		case 'H': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x041C0UL); break;
		default: return -1;
		}
		clicmd_GPIOxRegisterDisplay(inst, HexPort, (GPIO_T*)GPIOx);
		return 1;
	}
	else if (argc != 4)
	{
		return -1;
	}
	
	 
	if (strlen(argv[1]) != 1)
	{
		return -1;
	}

	HexPort = argv[1][0] & 0xdf;
	
	switch (HexPort)
	{
	case 'A': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04000UL); break;
	case 'B': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04040UL); break;
	case 'C': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04080UL); break;
	case 'D': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x040C0UL); break;
	case 'E': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04100UL); break;
	case 'F': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04140UL); break;
	case 'G': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x04180UL); break;
	case 'H': GPIOx = (GPIO_T*)(((uint32_t)0x40000000) + 0x041C0UL); break;
	default: return -1;		
	}
	
	 
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	 	
	if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
	{
		return -1;
	}
	
	 
	*((uint32_t *)(((uint32_t)GPIOx) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write GPIO%c Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexPort, HexOffset, HexValue);	
	return 1;
}  









 
static int clicmd_GPIOxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         gpiox: Access MCU GPIOx registers\r\n");
	return 1;
}  









 
static int clicmd_GPIOxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_GPIOxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: gpiox [<HexPort> <HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_NvicRegisterDisplay(CONSOLE_Inst *inst)
{
	NVIC_Type *NvicOffset = 0;
	uint32_t i;
	
	CONSOLE_PutMessage(inst, "Current NVIC(0x%08lx) registers...\r\n", (uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) ));		
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ISER[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ISER[i]), ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[i]);
	}
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ICER[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ICER[i]), ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ISPR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ISPR[i]), ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "ICPR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->ICPR[i]), ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[i]);
	}	
	for (i=0; i<8; i++)
	{
		CONSOLE_PutMessage(inst, "IABR[%d](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(NvicOffset->IABR[i]), ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[i]);
	}	
	CONSOLE_PutMessage(inst, "\r\n");				

	
	return 0;
}









 
static int clicmd_NvicRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
		
	if (argc == 1)
	{
		clicmd_NvicRegisterUsage(inst);
		clicmd_NvicRegisterDisplay(inst);		
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	 
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	 	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	 
	*((uint32_t *)(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write MVIC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
}  









 
static int clicmd_NvicRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          nvic: Access MCU NVIC registers\r\n");
	return 1;
}  









 
static int clicmd_NvicRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_NvicRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: nvic [<HexOffset> <HexValue>]\r\n");
	return 1;
}  










 
static int clicmd_EadcRegisterDisplay(CONSOLE_Inst *inst)
{
	EADC_T *EadcOffset = 0;
	uint32_t i=0;
	
	 
	CONSOLE_PutMessage(inst, "Current EADC(0x%08lx) registers...\r\n", (uint32_t)((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL)));			
	for (i=0; i<19; i++)
	{
		CONSOLE_PutMessage(inst, "DAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->DAT[i]), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->DAT[i]);
	}
	CONSOLE_PutMessage(inst, "CURDAT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CURDAT), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->CURDAT);
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CTL), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->CTL);
	CONSOLE_PutMessage(inst, "SWTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->SWTRG), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->SWTRG);
	CONSOLE_PutMessage(inst, "PENDSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->PENDSTS), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->PENDSTS);
	CONSOLE_PutMessage(inst, "OVSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->OVSTS), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->OVSTS);	
	for (i=0; i<19; i++)
	{
		CONSOLE_PutMessage(inst, "SCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->SCTL[i]), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->SCTL[i]);
	}
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "INTSRC[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->INTSRC[i]), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->INTSRC[i]);
	}
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "CMP[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->CMP[i]), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->CMP[i]);
	}
	CONSOLE_PutMessage(inst, "STATUS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS0), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->STATUS0);
	CONSOLE_PutMessage(inst, "STATUS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS1), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->STATUS1);
	CONSOLE_PutMessage(inst, "STATUS2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS2), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->STATUS2);
	CONSOLE_PutMessage(inst, "STATUS3(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->STATUS3), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->STATUS3);
	for (i=0; i<4; i++)
	{
		CONSOLE_PutMessage(inst, "DDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EadcOffset->DDAT[i]), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->DDAT[i]);
	}
	CONSOLE_PutMessage(inst, "PWRM(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->PWRM), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->PWRM);
	CONSOLE_PutMessage(inst, "CALCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CALCTL), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->CALCTL);
	CONSOLE_PutMessage(inst, "CALDWRD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EadcOffset->CALDWRD), ((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))->CALDWRD);
	return 0;
}









 
static int clicmd_EadcRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
		
	if (argc == 1)
	{
		clicmd_EadcRegisterUsage(inst);
		clicmd_EadcRegisterDisplay(inst);		
		return 1;		
	}
	else if (argc != 3)
	{
		return -1;
	}
	
	 
	if (clicmd_HexText2Long((uint8_t*)argv[1], &HexOffset, strlen(argv[1])) != 0)
	{
		return -1;
	}
	
	 	
	if (clicmd_HexText2Long((uint8_t*)argv[2], &HexValue, strlen(argv[2])) != 0)
	{
		return -1;
	}
	
	 
	*((uint32_t *)(((uint32_t)((EADC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x03000UL))) + HexOffset)) = HexValue;

	CONSOLE_PutMessage(inst, "Write EADC Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexOffset, HexValue);	
	return 1;
}  









 
static int clicmd_EadcRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          eadc: Access MCU EADC registers\r\n");
	return 1;
}  









 
static int clicmd_EadcRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EadcRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: eadc [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_QEIxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, QEI_T* QEIx)
{
	QEI_T *QEIxOffset = 0;
	
	 
	CONSOLE_PutMessage(inst, "Current QEI%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNT), QEIx->CNT);	
	CONSOLE_PutMessage(inst, "CNTHOLD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTHOLD), QEIx->CNTHOLD);
	CONSOLE_PutMessage(inst, "CNTLATCH(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTLATCH), QEIx->CNTLATCH);	
	CONSOLE_PutMessage(inst, "CNTCMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTCMP), QEIx->CNTCMP);	
	CONSOLE_PutMessage(inst, "CNTMAX(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CNTMAX), QEIx->CNTMAX);
	CONSOLE_PutMessage(inst, "CTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->CTL), QEIx->CTL);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(QEIxOffset->STATUS), QEIx->STATUS);	
	
	return 0;
}









 
static int clicmd_QEIxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	QEI_T *QEIx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			QEIx = ((QEI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x70000UL));
			break;
		case 1:
			QEIx = ((QEI_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x71000UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_QEIxRegisterUsage(inst);
			clicmd_QEIxRegisterDisplay(inst, HexIndex, QEIx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)QEIx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write QEI%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_QEIxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "          qeix: Access MCU QEIx registers\r\n");
	return 1;
}  









 
static int clicmd_QEIxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_QEIxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: qeix <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  









 
static int clicmd_ECAPxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, ECAP_T* ECAPx)
{
	ECAP_T *ECAPxOffset = 0;
	
	 
	CONSOLE_PutMessage(inst, "Current ECAP%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CNT), ECAPx->CNT);
	CONSOLE_PutMessage(inst, "HLD0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD0), ECAPx->HLD0);
	CONSOLE_PutMessage(inst, "HLD1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD1), ECAPx->HLD1);
	CONSOLE_PutMessage(inst, "HLD2(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->HLD2), ECAPx->HLD2);
	CONSOLE_PutMessage(inst, "CNTCMP(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CNTCMP), ECAPx->CNTCMP);
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CTL0), ECAPx->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->CTL1), ECAPx->CTL1);
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(ECAPxOffset->STATUS), ECAPx->STATUS);	

	return 0;
}









 
static int clicmd_ECAPxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	ECAP_T *ECAPx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			ECAPx = ((ECAP_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x74000UL));
			break;
		case 1:
			ECAPx = ((ECAP_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x75000UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_ECAPxRegisterUsage(inst);
			clicmd_ECAPxRegisterDisplay(inst, HexIndex, ECAPx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)ECAPx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write ECAP%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_ECAPxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         ecapx: Access MCU ECAPx registers\r\n");
	return 1;
}  









 
static int clicmd_ECAPxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_ECAPxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: ecapx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  










 
static int clicmd_EPWMxRegisterDisplay(CONSOLE_Inst *inst, uint8_t index, EPWM_T* EPWMx)
{
	uint32_t i;
	EPWM_T *EPWMxOffset = 0;
	
	 
	CONSOLE_PutMessage(inst, "Current EPWM%u registers...\r\n", index);	
	CONSOLE_PutMessage(inst, "CTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CTL0), EPWMx->CTL0);
	CONSOLE_PutMessage(inst, "CTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CTL1), EPWMx->CTL1);
	CONSOLE_PutMessage(inst, "SYNC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SYNC), EPWMx->SYNC);
	CONSOLE_PutMessage(inst, "SWSYNC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SWSYNC), EPWMx->SWSYNC);
	CONSOLE_PutMessage(inst, "CLKSRC(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CLKSRC), EPWMx->CLKSRC);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "CLKPSC[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CLKPSC[i]), EPWMx->CLKPSC[i]);
	}
	CONSOLE_PutMessage(inst, "CNTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CNTEN), EPWMx->CNTEN);
	CONSOLE_PutMessage(inst, "CNTCLR(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CNTCLR), EPWMx->CNTCLR);
	CONSOLE_PutMessage(inst, "LOAD(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LOAD), EPWMx->LOAD);
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "PERIOD[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PERIOD[i]), EPWMx->PERIOD[i]);
	}
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CMPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CMPDAT[i]), EPWMx->CMPDAT[i]);
	}
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "DTCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->DTCTL[i]), EPWMx->DTCTL[i]);
	}	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "PHS[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PHS[i]), EPWMx->PHS[i]);
	}		
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CNT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CNT[i]), EPWMx->CNT[i]);
	}
	CONSOLE_PutMessage(inst, "WGCTL0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL0), EPWMx->WGCTL0);
	CONSOLE_PutMessage(inst, "WGCTL1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL1), EPWMx->WGCTL1);
	CONSOLE_PutMessage(inst, "MSKEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->MSKEN), EPWMx->MSKEN);
	CONSOLE_PutMessage(inst, "MSK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->MSK), EPWMx->MSK);	
	CONSOLE_PutMessage(inst, "BNF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->BNF), EPWMx->BNF);
	CONSOLE_PutMessage(inst, "FAILBRK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->FAILBRK), EPWMx->FAILBRK);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "BRKCTL[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->BRKCTL[i]), EPWMx->BRKCTL[i]);
	}	
	CONSOLE_PutMessage(inst, "POLCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->POLCTL), EPWMx->POLCTL);
	CONSOLE_PutMessage(inst, "POEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->POEN), EPWMx->POEN);
	CONSOLE_PutMessage(inst, "SWBRK(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SWBRK), EPWMx->SWBRK);
	CONSOLE_PutMessage(inst, "INTEN0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTEN0), EPWMx->INTEN0);
	CONSOLE_PutMessage(inst, "INTEN1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTEN1), EPWMx->INTEN1);
	CONSOLE_PutMessage(inst, "INTSTS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTSTS0), EPWMx->INTSTS0);
	CONSOLE_PutMessage(inst, "INTSTS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->INTSTS1), EPWMx->INTSTS1);
	CONSOLE_PutMessage(inst, "DACTRGEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->DACTRGEN), EPWMx->DACTRGEN);
	CONSOLE_PutMessage(inst, "EADCTS0(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->EADCTS0), EPWMx->EADCTS0);
	CONSOLE_PutMessage(inst, "EADCTS1(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->EADCTS1), EPWMx->EADCTS1);
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "FTCMPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->FTCMPDAT[i]), EPWMx->FTCMPDAT[i]);
	}		
	CONSOLE_PutMessage(inst, "SSCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SSCTL), EPWMx->SSCTL);	
	CONSOLE_PutMessage(inst, "SSTRG(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->SSTRG), EPWMx->SSTRG);	
	CONSOLE_PutMessage(inst, "LEBCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LEBCTL), EPWMx->LEBCTL);	
	CONSOLE_PutMessage(inst, "LEBCNT(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->LEBCNT), EPWMx->LEBCNT);	
	CONSOLE_PutMessage(inst, "STATUS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->WGCTL1), EPWMx->STATUS);	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "IFA[%u](0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->IFA[i]), EPWMx->IFA[i]);
	}	
	CONSOLE_PutMessage(inst, "AINTSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->AINTSTS), EPWMx->AINTSTS);	
	CONSOLE_PutMessage(inst, "AINTEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->AINTEN), EPWMx->AINTEN);	
	CONSOLE_PutMessage(inst, "APDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->APDMACTL), EPWMx->APDMACTL);	
	CONSOLE_PutMessage(inst, "CAPINEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPINEN), EPWMx->CAPINEN);	
	CONSOLE_PutMessage(inst, "CAPCTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPCTL), EPWMx->CAPCTL);	
	CONSOLE_PutMessage(inst, "CAPSTS(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPSTS), EPWMx->CAPSTS);	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CAPDAT[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CAPDAT[i]), EPWMx->CAPDAT[i]);
	}		
	CONSOLE_PutMessage(inst, "PDMACTL(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->PDMACTL), EPWMx->PDMACTL);	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "PDMACAP[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PDMACAP[i]), EPWMx->PDMACAP[i]);
	}		
	CONSOLE_PutMessage(inst, "CAPIEN(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPIEN), EPWMx->CAPIEN);	
	CONSOLE_PutMessage(inst, "CAPIF(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->CAPIF), EPWMx->CAPIF);		
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "PBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->PBUF[i]), EPWMx->PBUF[i]);
	}	
	for (i=0; i<6; i++)
	{
		CONSOLE_PutMessage(inst, "CMPBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CMPBUF[i]), EPWMx->CMPBUF[i]);
	}		
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "CPSCBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->CPSCBUF[i]), EPWMx->CPSCBUF[i]);
	}	
	for (i=0; i<3; i++)
	{
		CONSOLE_PutMessage(inst, "FTCBUF[%u](0x%08lx): 0x%08lx\r\n", i, (uint32_t)&(EPWMxOffset->FTCBUF[i]), EPWMx->FTCBUF[i]);
	}		
	CONSOLE_PutMessage(inst, "FTCI(0x%08lx): 0x%08lx\r\n", (uint32_t)&(EPWMxOffset->FTCI), EPWMx->FTCI);		

	return 0;
}









 
static int clicmd_EPWMxRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t HexOffset, HexValue;
	uint8_t HexIndex;
	EPWM_T *EPWMx = 0;
	
	if (argc != 2 && argc != 4)
	{
		return -1;
	}
	else
	{
		 	
		if (clicmd_HexText2Char((uint8_t*)argv[1], &HexIndex, strlen(argv[1])) != 0)
		{
			return -1;
		}

		switch (HexIndex)
		{
		case 0:
			EPWMx = ((EPWM_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x18000UL));
			break;
		case 1:
			EPWMx = ((EPWM_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x19000UL));
			break;
		default:
			return 1;
		}
	
		if (argc == 2)
		{
			clicmd_EPWMxRegisterUsage(inst);
			clicmd_EPWMxRegisterDisplay(inst, HexIndex, EPWMx);
			return 1;
		}
	
		 
		if (clicmd_HexText2Long((uint8_t*)argv[2], &HexOffset, strlen(argv[2])) != 0)
		{
			return -1;
		}
	
		 	
		if (clicmd_HexText2Long((uint8_t*)argv[3], &HexValue, strlen(argv[3])) != 0)
		{
			return -1;
		}
	
		 
		*((uint32_t *)(((uint32_t)EPWMx) + HexOffset)) = HexValue;

		CONSOLE_PutMessage(inst, "Write EPWM%u Register, Offset: 0x%08lx, vlaue: 0x%08lx\r\n", HexIndex, HexOffset, HexValue);	
		return 1;
	}
}  









 
static int clicmd_EPWMxRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         epwmx: Access MCU EPWMx registers\r\n");
	return 1;
}  









 
static int clicmd_EPWMxRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EPWMxRegisterHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: epwmx <HexIndex> [<HexOffset> <HexValue>]\r\n");
	return 1;
}  













 
static int clicmd_EscRegister(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint32_t i;
	
	if (argc == 1)
	{
		clicmd_EscRegisterUsage(inst);
		
		clicmd_ShowEscFeatures(inst);
		clicmd_ShowEscAlInfo(inst);
		clicmd_ShowEscWdInfo(inst);
		clicmd_ShowEscFmmuSmInfo(inst);
		clicmd_ShowEscDcInfo(inst);
	}
	else
	{
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-ft") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscFeatures(inst);
			}
			else if (!strcmp(argv[i], "-al") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscAlInfo(inst);
			}		
			else if (!strcmp(argv[i], "-wd") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscWdInfo(inst);
			}	
			else if (!strcmp(argv[i], "-sm") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscFmmuSmInfo(inst);
			}			
			else if (!strcmp(argv[i], "-dc") && strlen(argv[i])==3)
			{
				i++;			
				clicmd_ShowEscDcInfo(inst);
			}			
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}
		
	}
	
	return 1;
}  









 
static int clicmd_EscRegisterHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        escreg: Show ESC registers\r\n");

	return 1;
}  









 
static int clicmd_EscRegisterUsage(CONSOLE_Inst *inst)
{
	clicmd_EscRegisterHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: escreg [-ft] [-al] [-wd] [-sm] [-dc]\r\n");
	return 1;
}  









 
static int clicmd_ShowTestBuffer(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "Current transfer buffer content...\r\n");
	CONSOLE_PutMessage(inst, "StartAddress=0x%x, EndAddress=0x%x\r\n"
								, pTestCtrl->Parameter.StartAddress
								, pTestCtrl->Parameter.EndAddress
								);

	CONSOLE_PutMessage(inst, "TxLen=0x%x, TxBuf[]=\r\n", pTemp->TxLen);
	clicmd_ShowMemoryInHex8b(inst, pTemp->TxBuf, pTemp->TxLen);		
	
	CONSOLE_PutMessage(inst, "RxLen=0x%x, RxBuf[]=\r\n", pTemp->RxLen);
	clicmd_ShowMemoryInHex8b(inst, pTemp->RxBuf, pTemp->RxLen);		
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}









 
static int clicmd_ShowTestTemporaryStatus(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "Current temporary status...\r\n");
	CONSOLE_PutMessage(inst, "  SPI DataSizeInWrite = %d\r\n", pTemp->Spis.DataSizeInWrite);
	CONSOLE_PutMessage(inst, "  SPI DataSizeInRead = %d\r\n", pTemp->Spis.DataSizeInRead);	
	CONSOLE_PutMessage(inst, "  SPI AddrSizeInWrite = %d\r\n", pTemp->Spis.AddrSizeInWrite);
	CONSOLE_PutMessage(inst, "  SPI AddrSizeInRead = %d\r\n", pTemp->Spis.AddrSizeInRead);
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}









 
static int clicmd_ShowPdiMemoryTestLog(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	int lineNum;
	TEST_TIME *pTime;
	TEST_TEMP *pTemp = &pTestCtrl->Temp;
	
	CONSOLE_PutMessage(inst, "PDI echo test log...\r\n");
	CONSOLE_PutMessage(inst, "DD:HH:MM:SS Round    Ok       Err      SpiWr/RdAddrSize SpiWr/RdDataSize HSTSR\r\n");
	lineNum	= 2;

	pTime = TEST_GetTime();
	CONSOLE_PutMessage(inst, "%02d:%02d:%02d:%02d %08lx %08lx %08lx %01d/%01d              %01d/%01d              %02x\r\n"
								, pTime->Day
								, pTime->Hour
								, pTime->Minute
								, pTime->Second
								, pTestCtrl->Record.RoundCnt
								, pTestCtrl->Record.OkCnt
								, pTestCtrl->Record.ErrorCnt
								, pTemp->Spis.AddrSizeInWrite
								, pTemp->Spis.AddrSizeInRead
								, pTemp->Spis.DataSizeInWrite
								, pTemp->Spis.DataSizeInRead
								, pTemp->HostInterfaceErrorStatus
								);								
	lineNum++;
	return lineNum;
}









 
static int clicmd_ShowPdiMemoryTestParameter(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	TEST_PARAMETER *pParameter = &pTestCtrl->Parameter;
	
	CONSOLE_PutMessage(inst, "Your test parameter...\r\n");
	CONSOLE_PutMessage(inst, "-loop=0x%x, -sta=0x%x, -end=0x%x, -ptt=0x%x, -lut=%d, -dfm=0x%x, -dfmr=0x%x, -sam=0x%x\r\n"
								, pParameter->LoopCount
								, pParameter->StartAddress
								, pParameter->EndAddress
								, pParameter->PatternType	
								, pParameter->LogUpdateTime		
								, pParameter->Spis.DataSizeInWrite			
								, pParameter->Spis.DataSizeInRead				
								, pParameter->Spis.AddrSize
								);
	CONSOLE_PutMessage(inst, "-ptn(%dBytes)=", pParameter->InitDataLen);
	clicmd_ShowMemoryInHex8b(inst, pParameter->InitData, pParameter->InitDataLen);		
	
	CONSOLE_PutMessage(inst, "\r\n");	
	return 0;
}









 
static int clicmd_PdiMemoryTest(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t lineNum;
	uint32_t i;
	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == 0x03)
		{
			clicmd_testCtrl.Terminated = 1;
		}		
			
		TEST_PdiMemoryTest(&clicmd_testCtrl);			

		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowPdiMemoryTestLog(inst, &clicmd_testCtrl);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					clicmd_ShowTestBuffer(inst, &clicmd_testCtrl);				
					clicmd_ShowTestTemporaryStatus(inst, &clicmd_testCtrl);					
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_PdiMemoryTestUsage(inst);
		clicmd_ShowTestBuffer(inst, &clicmd_testCtrl);
		clicmd_ShowTestTemporaryStatus(inst, &clicmd_testCtrl);
		return 1;
	}
	else
	{
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		 
		pParameter->LoopCount = 0;
		pParameter->LogUpdateTime = 500;
		pParameter->StartAddress = 0x1000;
		pParameter->EndAddress = 0x1000 + 0x0FFF;		
		pParameter->PatternType = TEST_PATTERN_RANDOM;				
		pParameter->InitDataLen = 2;
		pParameter->InitData[0] = 0x55;		
		pParameter->InitData[1] = 0xAA;				
		pParameter->Spis.AddrSize = TEST_SPI_ADDR_SIZE_AUTO;
		pParameter->Spis.DataSizeInWrite = TEST_SPI_DATA_SIZE_AUTO;				
		pParameter->Spis.DataSizeInRead = TEST_SPI_4BYTE_DATA;		
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &pParameter->LogUpdateTime, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}			
			else if (!strcmp(argv[i], "-sta") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->StartAddress, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}		
			else if (!strcmp(argv[i], "-end") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->EndAddress, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-ptt") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->PatternType, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-ptn") && strlen(argv[i])==4)
			{
				i++;
				
					uint8_t *pbuf8 = pParameter->InitData;	
					pParameter->InitDataLen = 0;
					for (; i<argc; i++)
					{					
						if ((argv[i][0] == '-') ||
								(pParameter->InitDataLen >= 128) || 
								(clicmd_HexText2Char((uint8_t*)argv[i], pbuf8, strlen(argv[i])) != 0))
						{
							i--;
							break;
						}
						pbuf8++;
						pParameter->InitDataLen++;						
					}					
			}
			else if (!strcmp(argv[i], "-dfm") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.DataSizeInWrite, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-dfmr") && strlen(argv[i])==5)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.DataSizeInRead, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}			
			else if (!strcmp(argv[i], "-sam") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_HexText2Char((uint8_t*)argv[i], (uint8_t*)&pParameter->Spis.AddrSize, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-trg") && strlen(argv[i])==4)
			{
			  ;
				;
				CONSOLE_PutMessage(inst, "Generate trigger output!\r\n");
				return 1;
			}			
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}
		
		 
		if (pParameter->StartAddress > pParameter->EndAddress)
		{
			CONSOLE_PutMessage(inst, "StartAddress cannot larger than EndAddress\r\n");			
			return -1;
		}
		
		clicmd_ShowPdiMemoryTestParameter(inst, &clicmd_testCtrl);
		CONSOLE_PutMessage(inst, "Start PDI memory test use ESC chip select...\r\n");
		inst->State = CLI_STATE_COMMAND_WAIT;		
	}

	return 1;
}  









 
static int clicmd_PdiMemoryTestHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "         pdimt: PDI memory access test\r\n");	
	return 1;
}  









 
static int clicmd_PdiMemoryTestUsage(CONSOLE_Inst *inst)
{
	clicmd_PdiMemoryTestHelp(inst);
	CONSOLE_PutMessage(inst, "Usage: pdimt [-loop HexValue] [-sta HexValue] [-end HexValue] [-ptt HexValue] [-ptn <HexValue0>...<HexValueN>]\r\n");
	CONSOLE_PutMessage(inst, "             [-lut DecValue] [-dfm HexValue] [-dfmr HexValue] [-sam HexValue] [-csm HexValue] [-trg]\r\n");	
	CONSOLE_PutMessage(inst, "       -loop: Specifies the number of test rounds.\r\n");	
	CONSOLE_PutMessage(inst, "              0: Infinite test, until press \"Ctrl-C\".\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", 0);
	CONSOLE_PutMessage(inst, "       -lut: Specifies the update time of log, unit in 100ms\r\n");		
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", 500);		
	CONSOLE_PutMessage(inst, "       -sta: Specifies the start address of target memory to be test.\r\n");	
	CONSOLE_PutMessage(inst, "              Default: 0x%08lx\r\n", 0x1000);	
	CONSOLE_PutMessage(inst, "       -end: Specifies the end address of target memory to be test.\r\n");		
	CONSOLE_PutMessage(inst, "              Default: 0x%08lx\r\n", 0x1000 + 0x0FFF);
	CONSOLE_PutMessage(inst, "       -ptt: Specifies the data pattern type.\r\n");	
	CONSOLE_PutMessage(inst, "              0: Fixed, using \"-ptn\" parameter to specify pattern.\r\n");		
	CONSOLE_PutMessage(inst, "              1: Increment, using \"-ptn\" parameter to specify initial pattern.\r\n");			
	CONSOLE_PutMessage(inst, "              2: Decrement, using \"-ptn\" parameter to specify initial pattern.\r\n");		
	CONSOLE_PutMessage(inst, "              3: Random.\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_PATTERN_RANDOM);
	CONSOLE_PutMessage(inst, "       -ptn: Specifies the pattern for \"Fixed\" type\r\n");		
	CONSOLE_PutMessage(inst, "             or initial pattern for other types.\r\n");
	CONSOLE_PutMessage(inst, "              Default: 0x55 0xAA\r\n");	
	CONSOLE_PutMessage(inst, "       -dfm: Specifies data fragment mode for ASIX_SPI/BECKHOFF_SPI write operation.\r\n");		
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly change data fragment size being 1, 2 or 4Byte(s) for write operation\r\n");
	CONSOLE_PutMessage(inst, "              1: 1 byte write\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte write\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte write\r\n");				
	CONSOLE_PutMessage(inst, "              4: 4 byte write\r\n");			
	CONSOLE_PutMessage(inst, "              N: N byte write\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_SPI_DATA_SIZE_AUTO);
	CONSOLE_PutMessage(inst, "       -dfmr: Specifies data fragment mode for ASIX_SPI/BECKHOFF_SPI read operation.\r\n");		
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly change data fragment size being 1, 2 or 4Byte(s) for read operation\r\n");
	CONSOLE_PutMessage(inst, "              1: 1 byte read\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte read\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte read\r\n");				
	CONSOLE_PutMessage(inst, "              4: 4 byte read\r\n");			
	CONSOLE_PutMessage(inst, "              N: N byte read\r\n");
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_SPI_4BYTE_DATA);	
	CONSOLE_PutMessage(inst, "       -sam: Specifies ASIX_SPI/BECKHOFF_SPI address mode.\r\n");
	CONSOLE_PutMessage(inst, "             this parameter is valid only for ASIX_SPI/BECKHOFF_SPI pdi mode.\r\n");
	CONSOLE_PutMessage(inst, "              0: Randomly address byte size between 2, 3Bytes for write/read operation\r\n");
	CONSOLE_PutMessage(inst, "              2: 2 byte address\r\n");		
	CONSOLE_PutMessage(inst, "              3: 3 byte address\r\n");				
	CONSOLE_PutMessage(inst, "              Default: %d\r\n", TEST_SPI_ADDR_SIZE_AUTO);
	CONSOLE_PutMessage(inst, "       -csm: Chip Select Mode.\r\n");	
	CONSOLE_PutMessage(inst, "             0: FUN_CS, 1: ESC_CS\r\n");			
	CONSOLE_PutMessage(inst, "       -trg: Trigger output test for debugging.\r\n");	
	CONSOLE_PutMessage(inst, "             Note: please connect PA.15 of Nucleo EVB. to trigger in of your instrument\r\n");			
	CONSOLE_PutMessage(inst, "\r\n");		
	return 1;
}  









 
static int clicmd_PdiReset(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t tmp8;
	
	tmp8 = 'R';
	HW_EscWrite((unsigned long*)&tmp8, 0x41, 1);
	tmp8 = 'E';
	HW_EscWrite((unsigned long*)&tmp8, 0x41, 1);	
	tmp8 = 'S';
	HW_EscWrite((unsigned long*)&tmp8, 0x41, 1);	
	
	return 1;
}  









 
static int clicmd_PdiResetHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        pdirst: Perform PDI reset\r\n");	
	return 1;
}  











 
static int clicmd_EscStack(CONSOLE_Inst *inst, int argc, char **argv)
{
	HW_DEBUG *pDbgCnt;
	pDbgCnt = HW_GetDebugCounter();
		
	CONSOLE_PutMessage(inst, "EscIsrCnt=0x%08lx\r\n", pDbgCnt->EscIsrCnt);		
	CONSOLE_PutMessage(inst, "Sync0IsrCnt=0x%08lx\r\n", pDbgCnt->Sync0IsrCnt);	
	CONSOLE_PutMessage(inst, "Sync1IsrCnt=0x%08lx\r\n", pDbgCnt->Sync1IsrCnt);
	CONSOLE_PutMessage(inst, "TmrTskIsrCnt=0x%08lx\r\n", pDbgCnt->TmrTskIsrCnt);	
	CONSOLE_PutMessage(inst, "PdiReentryCnt=0x%08lx\r\n", HW_SpiObj.ReentryCnt);
	
	CONSOLE_PutMessage(inst, "\r\n/* ESC Stack Status */\r\n");	
	CONSOLE_PutMessage(inst, "bDcSyncActive=%d\r\n", bDcSyncActive);	


	return 1;
}  









 
static int clicmd_EscStackHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "           esc: Shows ESC stack status\r\n");	
	return 1;
}  










 
static int clicmd_ShowMcStack(CONSOLE_Inst *inst)
{


	CONSOLE_PutMessage(inst, "\r\n/* MC Stack Status */\r\n");

	CONSOLE_PutMessage(inst, "\r\n");	
	return 1;
}  









 
static int clicmd_ShowMcDashboard(CONSOLE_Inst *inst, TEST_CONTROL *pTestCtrl)
{
	int lineNum=0;
	char *stateStr, *faultStr;
	MC_T *pMC = &MotorCtrl;
	
	CONSOLE_PutMessage(inst, "MC Dashboard\r\n");
	lineNum	= 1;

	 		
	switch(MC_GetState())
	{
	case MCSTE_IDLE:            stateStr = "MCSTE_IDLE           "; break; 
	case MCSTE_TUNE:            stateStr = "MCSTE_TUNE           "; break;
	case MCSTE_FAULT:           stateStr = "MCSTE_FAULT          "; break;		
	case MCSTE_WAIT_CLR_ERR:    stateStr = "MCSTE_WAIT_CLR_ERR   "; break;				
	case MCSTE_STOP:            stateStr = "MCSTE_STOP           "; break;		
	case MCSTE_START_VEC_ALIGN: stateStr = "MCSTE_START_VEC_ALIGN"; break;
	case MCSTE_VEC_ALIGN:       stateStr = "MCSTE_VEC_ALIGN      "; break;		
	case MCSTE_START_ENC_ALIGN: stateStr = "MCSTE_START_ENC_ALIGN"; break;				
	case MCSTE_ENC_ALIGN:       stateStr = "MCSTE_ENC_ALIGN      "; break;
	case MCSTE_STARTUP:         stateStr = "MCSTE_STARTUP        "; break;	  
	case MCSTE_RUN:             stateStr = "MCSTE_RUN            "; break;
	case MCSTE_SHUTDOWN:        stateStr = "MCSTE_SHUTDOWN       "; break;
	}
	
	switch(MC_GetFaultStatus())
	{
	case MCFS_NO_ERR:      faultStr = "MCFS_NO_ERR     "; break; 
	case MCFS_OVER_VOLT:   faultStr = "MCFS_OVER_VOLT  "; break;
	case MCFS_UNDER_VOLT:  faultStr = "MCFS_UNDER_VOLT "; break;
	case MCFS_OVER_TEMP:   faultStr = "MCFS_OVER_TEMP  "; break;
	case MCFS_PHASE_ERR:   faultStr = "MCFS_PHASE_ERR  "; break;
	case MCFS_OVER_CURR:   faultStr = "MCFS_OVER_CURR  "; break;
	case MCFS_SW_ERR:      faultStr = "MCFS_SW_ERR     "; break;
	case MCFS_VEC_ALI_ERR: faultStr = "MCFS_VEC_ALI_ERR"; break;
	case MCFS_ENC_ALI_ERR: faultStr = "MCFS_ENC_ALI_ERR"; break;		
	}	
	CONSOLE_PutMessage(inst, "McState               FaultStatus\r\n");	
	CONSOLE_PutMessage(inst, "%s %s\r\n"
										, stateStr
										, faultStr
										);
	lineNum+=2;

	return lineNum;

}  









 
static int clicmd_McStackUsage(CONSOLE_Inst *inst)
{
	clicmd_McStackHelp(inst);
	
	CONSOLE_PutMessage(inst, "Usage: mc [-dsb] [-clrf] [-st DecEnb]\r\n");
	CONSOLE_PutMessage(inst, "          [-alg DecDuartion] [-rstpos]\r\n");	
	CONSOLE_PutMessage(inst, "          [-pos DecPosition DecSpeed] [-pospid DecValue DecValue] [-spdpid DecValue DecValue] [-amppid DecValue DecValue]\r\n");	
	
	 
	CONSOLE_PutMessage(inst, "      -dsb: Enabled dashboard mode.\r\n");
	CONSOLE_PutMessage(inst, "     -clrf: Clear fault status.\r\n");		
	CONSOLE_PutMessage(inst, "       -st: 1/0=Start/Stop motor control.\r\n");
	 	
	CONSOLE_PutMessage(inst, "      -alg: Perform encoder alignment with specified duration time(ms).\r\n");	
	CONSOLE_PutMessage(inst, "   -rstpos: Reset position.\r\n");	
	 		
	CONSOLE_PutMessage(inst, "      -pos: Move to target position(deg) with specified duration(ms).\r\n");		
	CONSOLE_PutMessage(inst, "    -pospid: Specify KP/KI value for position control loop.\r\n");
	 
	CONSOLE_PutMessage(inst, "    -spdpid: Specify KP/KI value for speed control loop.\r\n");	
	 
	CONSOLE_PutMessage(inst, "    -amppid: Specify KP/KI value for current control loop.\r\n");	
	return 1;
}  









 
static int clicmd_McStack(CONSOLE_Inst *inst, int argc, char **argv)
{
	uint8_t lineNum, dashBoardEnable=0, tmp8;
	uint32_t i, tmp32;
	int32_t stmp32[3];
	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	MC_T *pMC = &MotorCtrl;
	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == 0x03)
		{
			clicmd_testCtrl.Terminated = 1;
			
			clicmd_testCtrl.State = TEST_DONE;
		}		
			
		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowMcDashboard(inst, &clicmd_testCtrl);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				printdCtrl(1); 		




				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_McStackUsage(inst);
		return 1;
	}
	else
	{
	
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		 
		pParameter->LoopCount = 0;
		pParameter->LogUpdateTime = 500;
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				pParameter->LogUpdateTime = tmp32;
			}
			else if (!strcmp(argv[i], "-dsb") && strlen(argv[i])==4)
			{
				dashBoardEnable = 1;
			}		
			else if (!strcmp(argv[i], "-clrf") && strlen(argv[i])==5)
			{
				MC_FaultReset();			
			}			
			else if (!strcmp(argv[i], "-st") && strlen(argv[i])==3)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				if (tmp32)
				{
					 
					MC_StartMotor();
				}
				else
				{
					 
					MC_StopMotor();					
				}
			}		
			 
			else if (!strcmp(argv[i], "-alg") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				
				 				
				MC_StartHoming(tmp32);				
			}	
			else if (!strcmp(argv[i], "-rstpos") && strlen(argv[i])==7)
			{
				 
				MC_ClearActualPosition();				
			}			
			 			
			else if (!strcmp(argv[i], "-pos") && strlen(argv[i])==4)
			{
				 
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}

				 				
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}				
				MC_SetTargetPosition(stmp32[0], stmp32[1]);	
			}
			else if (!strcmp(argv[i], "-pospid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				(mapMemory[((300)+7)]) = stmp32[0];
				(mapMemory[((300)+8)]) = stmp32[1];
			}
			 			
			else if (!strcmp(argv[i], "-spdpid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				(mapMemory[((320)+7)]) = stmp32[0];
				(mapMemory[((320)+8)]) = stmp32[1];
			}			
			 
			else if (!strcmp(argv[i], "-amppid") && strlen(argv[i])==7)
			{
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[0]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				i++;
				if (clicmd_DecText2SignLong((uint8_t*)argv[i], &(stmp32[1]), strlen(argv[i])) != 0)
				{
					return -1;
				}
				(mapMemory[((340)+7)]) = stmp32[0];
				(mapMemory[((340)+8)]) = stmp32[1];
			}
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}
		
		 
		if (dashBoardEnable)
		{
			printdCtrl(0); 




			inst->State = CLI_STATE_COMMAND_WAIT;		
		}
	}
	return 1;
}  









 
static int clicmd_McStackHelp(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "            mc: Motor control\r\n");	
	return 1;
}  









 
static int clicmd_ShowCiA402Dashboard(CONSOLE_Inst *inst, uint8_t AxisId)
{
	int lineNum=0;

	char *stateStr;
	CiA402_AXIS_T *pAxis = &CiA402Axis;
	CiA402_OBJ_T *pCiA402 = &(pAxis->CiA402Var);	
	HW_DEBUG *pDbgCnt = HW_GetDebugCounter();	

	
	CONSOLE_PutMessage(inst, "CiA402 Dashboard\r\n");
	lineNum	= 1;

	 
	CONSOLE_PutMessage(inst, "EscIsrCnt  Sync0IsrCnt Sync1IsrCnt TmrTskIsrCnt PdiReentryCnt\r\n");	
	CONSOLE_PutMessage(inst, "0x%08lx 0x%08lx  0x%08lx  0x%08lx   0x%08lx\r\n"
										, pDbgCnt->EscIsrCnt
										, pDbgCnt->Sync0IsrCnt	
										, pDbgCnt->Sync1IsrCnt
										, pDbgCnt->TmrTskIsrCnt
										, HW_SpiObj.ReentryCnt
										);
	lineNum+=2;	
	
	 		
	switch(pAxis->i16State)
	{
	case 0x0001:       stateStr = "NOT_READY_TO_SWITCH_ON "; break; 
	case 0x0002:           stateStr = "SWITCH_ON_DISABLED     "; break;
	case 0x0004:           stateStr = "READY_TO_SWITCH_ON     "; break;
	case 0x0008:                  stateStr = "SWITCHED_ON            "; break;	  
	case 0x0010:            stateStr = "OPERATION_ENABLED      "; break;
	case 0x0020:            stateStr = "QUICK_STOP_ACTIVE      "; break;
	case 0x0040:        stateStr = "FAULT_REACTION_ACTIVE  "; break;
	case 0x0080:                        stateStr = "FAULT                  "; break;		
	}

	CONSOLE_PutMessage(inst, "CiA402State             PendCode AxisAct BrakeApplied LowPower HighPower FuncEnb CfgAllowed CycleTime\r\n");	
	CONSOLE_PutMessage(inst, "%s 0x%04x   %01u       %01u            %01u        %01u         %01u       %01u          %010u\r\n"
										, stateStr
										, pAxis->u16PendingOptionCode	
										, pAxis->bAxisIsActive
										, pAxis->bBrakeApplied
										, pAxis->bLowLevelPowerApplied
										, pAxis->bHighLevelPowerApplied
										, pAxis->bAxisFunctionEnabled
										, pAxis->bConfigurationAllowed
										, pAxis->u32CycleTime
										);
	lineNum+=2;
		
	 
	CONSOLE_PutMessage(inst, "ControlWord StatusWord ErrorCode ModesOfOp ModesOfOpDisp SuppDrivModes\r\n");	
	CONSOLE_PutMessage(inst, "0x%04x      0x%04x     0x%04x    0x%02x      0x%02x          0x%08lx\r\n"
										, *(pCiA402->pControlword0x6040)
										, *(pCiA402->pStatusword0x6041)			
										, *(pCiA402->pErrorCode0x603F)		
										, *(pCiA402->pModesOfOperation0x6060)
										, *(pCiA402->pModesOfOperationDisplay0x6061)	
										, *(pCiA402->pSupportedDriveModes0x6502)
										);
	lineNum+=2;

	 
	CONSOLE_PutMessage(inst, "QuickStopOC ShutDownOC DisableOpOC FaultReactOC\r\n");	
	CONSOLE_PutMessage(inst, "0x%04x      0x%04x     0x%04x      0x%04x\r\n"
										, *(pCiA402->pQuickStopOptionCode0x605A)
										, *(pCiA402->pShutdownOptionCode0x605B)			
										, *(pCiA402->pDisableOperationOptionCode0x605C)		
										, *(pCiA402->pFaultReactionOptionCode0x605E)
										);
	lineNum+=2;
	
	 
	CONSOLE_PutMessage(inst, "TrgPos     PosAct     DigitalInputs\r\n");	
	CONSOLE_PutMessage(inst, "%010d %010d %08lx\r\n"
										, *(pCiA402->pTargetPosition0x607A)
										, *(pCiA402->pPositionActualValue0x6064)
										, *(pCiA402->pDigitalInputs0x60FD)
										);
										
	lineNum+=2;

	 
	switch(MC_GetState())
	{
	case MCSTE_IDLE:           stateStr = "MCSTE_IDLE        "; break;
	case MCSTE_ENC_ALIGN:      stateStr = "MCSTE_ENC_ALIGN   "; break;
	case MCSTE_STARTUP :       stateStr = "MCSTE_STARTUP     "; break;	  
	case MCSTE_RUN:            stateStr = "MCSTE_RUN         "; break;
	case MCSTE_SHUTDOWN:       stateStr = "MCSTE_SHUTDOWN    "; break;
	case MCSTE_STOP:           stateStr = "MCSTE_STOP        "; break;		
	case MCSTE_FAULT:          stateStr = "MCSTE_FAULT       "; break;
	case MCSTE_WAIT_CLR_ERR:   stateStr = "MCSTE_WAIT_CLR_ERR"; break;
	}
	
	 
	CONSOLE_PutMessage(inst, "HomeOffset HomeMethod HomeAcc    McState\r\n");	
	CONSOLE_PutMessage(inst, "%010d %010u %010u %s\r\n"
										, *(pCiA402->pHomeOffset0x607C)	
										, *(pCiA402->pHomingMethod0x6098)			
										, *(pCiA402->pHomingAcceleration0x609A)			
										, stateStr										
										);
	lineNum+=2;

	return lineNum;

}  









 
static int clicmd_CiA402Usage(CONSOLE_Inst *inst)
{
	clicmd_CiA402Help(inst);
	
	CONSOLE_PutMessage(inst, "Usage: cia402 [-dsb] [-clrf] [em [DecValue]]\r\n");
	
	 
	CONSOLE_PutMessage(inst, "      -dsb: Enabled dashboard mode.\r\n");
	CONSOLE_PutMessage(inst, "     -clrf: Clear fault.\r\n");
	CONSOLE_PutMessage(inst, "       -em: Enable emulation mode.\r\n");
	CONSOLE_PutMessage(inst, "            0: Disable emulation mode.\r\n");	
	CONSOLE_PutMessage(inst, "            1: Set actual position equal to target position.\r\n");		
	CONSOLE_PutMessage(inst, "            2: Set actual position equal to trajectory output position.\r\n");
	return 1;
}  









 
static int clicmd_CiA402(CONSOLE_Inst *inst, int argc, char **argv)
{

	uint8_t lineNum, dashBoardEnable=0, tmp8;
	uint32_t i, tmp32;


	TEST_PARAMETER *pParameter = &clicmd_testCtrl.Parameter;
	CiA402_AXIS_T *pAxis = &CiA402Axis;

	
	if (inst->State == CLI_STATE_COMMAND_WAIT)
	{
		if (inst->GetChar() == 0x03)
		{
			clicmd_testCtrl.Terminated = 1;
			
			clicmd_testCtrl.State = TEST_DONE;
		}		
			
		if (inst->Timer==0)
		{
			lineNum = clicmd_ShowCiA402Dashboard(inst, 0);			
			if (clicmd_testCtrl.State == TEST_DONE)
			{
				printdCtrl(1); 
				inst->State = CLI_STATE_COMMAND;
				CONSOLE_PutMessage(inst, "\r\n");
				if (clicmd_testCtrl.Record.ErrorCnt)
				{
					return -1;
				}
				else
				{
					return 1;		
				}
			}
			else
			{			
				inst->PutChar(0x0D);
				while (lineNum--)
				{
					inst->PutChar(0x1B);
					inst->PutChar(0x5B);		
					inst->PutChar(0x41);					
				}
				inst->Timer = pParameter->LogUpdateTime;
			}
		}
		return 1;
	}
	else if (argc == 1)
	{
		clicmd_CiA402Usage(inst);
		return 1;
	}
	else
	{
	
		memset((uint8_t*)&clicmd_testCtrl, 0, sizeof(clicmd_testCtrl));
		
		 
		pParameter->LoopCount = 0;
		pParameter->LogUpdateTime = 500;
		
		for (i=1; i<argc; i++)
		{
			if (!strcmp(argv[i], "-loop") && strlen(argv[i])==5)
			{
				i++;			
				if (clicmd_HexText2Long((uint8_t*)argv[i], &pParameter->LoopCount, strlen(argv[i])) != 0)
				{
					return -1;
				}
			}
			else if (!strcmp(argv[i], "-lut") && strlen(argv[i])==4)
			{
				i++;
				if (clicmd_DecText2Long((uint8_t*)argv[i], &tmp32, strlen(argv[i])) != 0)
				{
					return -1;
				}
				pParameter->LogUpdateTime = tmp32;
			}
			else if (!strcmp(argv[i], "-dsb") && strlen(argv[i])==4)
			{
				dashBoardEnable = 1;
			}		
			else if (!strcmp(argv[i], "-clrf") && strlen(argv[i])==5)
			{
				MC_FaultReset();				
			}	
			else if (!strcmp(argv[i], "-em") && strlen(argv[i])==3)
			{
				i++;
				if (clicmd_DecText2Char((uint8_t*)argv[i], &tmp8, strlen(argv[i])) != 0)
				{
					return -1;
				}				
				pAxis->EmulationEnable = tmp8;
				CIA402_EmulationControl(tmp8);
			}						
			else
			{
				CONSOLE_PutMessage(inst, "Unsupported parameter:%s\r\n", argv[i]);				
				return -1;
			}
		}
		
		 
		if (dashBoardEnable)
		{
			printdCtrl(0); 
			inst->State = CLI_STATE_COMMAND_WAIT;		
		}
	}

	return 1;
}  









 
static int clicmd_CiA402Help(CONSOLE_Inst *inst)
{
	CONSOLE_PutMessage(inst, "        cia402: CiA402 status\r\n");	
	return 1;
}  


 
