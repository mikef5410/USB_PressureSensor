//*****************************************************************************
//
// startup.c - Boot code for Tek 4330 controller board.
//
//*****************************************************************************
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>

#define HARDFAULT_SHOULD_RESET
void selfReset(void);

#if defined (__cplusplus)
// The entry point for the C++ library startup
extern "C" {
  extern void __libc_init_array(void);
}
#endif
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))
// foo// if CMSIS is being used, then SystemInit() routine// will be called by startup code rather than in application's main()//*****************************************************************************// Forward declaration of the default fault handlers.//*****************************************************************************
    extern void SystemInit(void);

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// main() is the entry point for Newlib based applications
//*****************************************************************************
  extern int main(void);

//*****************************************************************************
// Forward declaration of the default handlers. These are aliased using the
// linker "weak" attribute:  if the application defines a handler with the 
// same name, it will take precedence over the weak definition
//*****************************************************************************
  void Reset_Handler(void);
  WEAK void NMI_Handler(void);
  WEAK void HardFault_Handler(void *args);
  WEAK void MemManage_Handler(void);
  WEAK void BusFault_Handler(void);
  WEAK void UsageFault_Handler(void);
  WEAK void DebugMon_Handler(void);
  WEAK void IntDefaultHandler(void);

// FreeRTOS vs CMSIS interrupt vector definitions
  extern void xPortPendSVHandler(void);
  extern void PendSV_Handler(void);

  extern void xPortSysTickHandler(void);
  extern void SysTick_Handler(void);

  extern void vPortSVCHandler(void);
  extern void SVC_Handler(void);

//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. Most are aliased
// to the IntDefaultHandler, which is a 'forever' loop.
//
//*****************************************************************************
  void WWDG_IRQHandler(void) ALIAS(IntDefaultHandler);
  void PVD_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TAMP_STAMP_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RTC_WKUP_IRQHandler(void) ALIAS(IntDefaultHandler);
  void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RCC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream5_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Stream6_IRQHandler(void) ALIAS(IntDefaultHandler);
  void ADC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_TX_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_RX0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_RX1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_SCE_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI9_5_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM15_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM16_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM17_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM18_DAC2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C1_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C1_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C2_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C2_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SPI2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI15_10_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM15_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM16_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM17_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RTC_ALARM_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CEC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM5_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SPI3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM6_DAC1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM7_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA2_Stream0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA2_Stream1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA2_Stream2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA2_Stream3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA2_Stream4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SDADC1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SDADC2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SDADC3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void COMP1_2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM19_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USB_HP_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USB_LP_IRQHandler(void) ALIAS(usb_lp_isr);
  void USB_WKUP_IRQHandler(void) ALIAS(IntDefaultHandler);
  void FPU_IRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
  extern void __StackTop(void);

//*****************************************************************************
#if defined (__cplusplus)
}				// extern "C"
#endif
//*****************************************************************************// The vector table. This one's in Flash. placed at offset 0x0 so it's // available right at bootup. We'll copy it to RAM and change VTOR to point// to the RAM copy in the reset handler. Then runtime code can change// interrupt vectors. //*****************************************************************************
    extern void (*const FlashVectors[]) (void);

//static FILE real_stdout;

__attribute__ ((section(".isr_vector")))
void (*const FlashVectors[]) (void) =
{
  // Core Level - CM4/CM3
  &__StackTop,			// 0x0 The initial stack pointer
      Reset_Handler,		// 0x4 The reset handler
      NMI_Handler,		// 0x8 The NMI handler
      (void (*)(void)) HardFault_Handler,	// 0xc The hard fault handler
      MemManage_Handler,	// 0x10 The MPU fault handler
      BusFault_Handler,		// 0x14 The bus fault handler
      UsageFault_Handler,	// 0x18 The usage fault handler
      0,			// 0x1c Reserved
      0,			// 0x20 Reserved
      0,			// 0x24 Reserved
      0,			// 0x28 Reserved
      SVC_Handler,		// 0x2c SVCall handler
      DebugMon_Handler,		// 0x30 Debug monitor handler
      0,			// 0x34 Reserved
      PendSV_Handler,		// 0x38 The PendSV handler
      SysTick_Handler,		// 0x3c The SysTick handler
      WWDG_IRQHandler,		// 0x40 Window Watchdog
      PVD_IRQHandler,		// 0x44 PVD through EXTI Line detection 
      TAMP_STAMP_IRQHandler,	// 0x48 Tamper and TimeStamps through the EXTI line 
      RTC_WKUP_IRQHandler,	// 0x4c RTC Wakeup through the EXTI line 
      FLASH_IRQHandler,		// 0x50 FLASH                        
      RCC_IRQHandler,		// 0x541 RCC                                                  
      EXTI0_IRQHandler,		// 0x582 EXTI Line0                                           
      EXTI1_IRQHandler,		// 0x5c3 EXTI Line1                                           
      EXTI2_IRQHandler,		// 0x604 EXTI Line2                                           
      EXTI3_IRQHandler,		// 0x645 EXTI Line3                                           
      EXTI4_IRQHandler,		// 0x686 EXTI Line4                                           
      DMA1_Stream0_IRQHandler,	// 0x6c7 DMA1 Stream 0                                        
      DMA1_Stream1_IRQHandler,	// 0x708 DMA1 Stream 1                                        
      DMA1_Stream2_IRQHandler,	// 0x749 DMA1 Stream 2                                        
      DMA1_Stream3_IRQHandler,	// 0x780 DMA1 Stream 3                                        
      DMA1_Stream4_IRQHandler,	// 0x7c1 DMA1 Stream 4                                        
      DMA1_Stream5_IRQHandler,	// 0x802 DMA1 Stream 5                                        
      DMA1_Stream6_IRQHandler,	// 0x843 DMA1 Stream 6                                        
      ADC_IRQHandler,		// 0x884 ADC1, ADC2 and ADC3s                                 
      CAN1_TX_IRQHandler,	// 0x8c5 CAN1 TX                                              
      CAN1_RX0_IRQHandler,	// 0x906 CAN1 RX0                                             
      CAN1_RX1_IRQHandler,	// 0x947 CAN1 RX1                                             
      CAN1_SCE_IRQHandler,	// 0x988 CAN1 SCE                                             
      EXTI9_5_IRQHandler,	// 0x9c9 External Line[9:5]s                                  
      TIM15_IRQHandler,	        // 0xa00 TIM15
      TIM16_IRQHandler, 	// 0xa4 TIM16          
      TIM17_IRQHandler, 	// 0xa8 TIM17
      TIM18_DAC2_IRQHandler,    // 0xac TIM18/DAC2 underrrun
      TIM2_IRQHandler,		// 0xb0 TIM2
      TIM3_IRQHandler,		// 0xb4 TIM3                         
      TIM4_IRQHandler,		// 0xb8 TIM4                         
      I2C1_EV_IRQHandler,	// 0xbc I2C1 Event                            
      I2C1_ER_IRQHandler,	// 0xc0 I2C1 Error                            
      I2C2_EV_IRQHandler,	// 0xc4 I2C2 Event                            
      I2C2_ER_IRQHandler,	// 0xc8 I2C2 Error                            
      SPI1_IRQHandler,		// 0xcc SPI1                                  
      SPI2_IRQHandler,		// 0xd0 SPI2                                  
      USART1_IRQHandler,	// 0xd4 USART1                                
      USART2_IRQHandler,	// 0xd8 USART2                                
      USART3_IRQHandler,	// 0xdc USART3                                
      EXTI15_10_IRQHandler,	// 0xe0 External Line[15:10]s                 
      RTC_Alarm_IRQHandler,	// 0xe4 RTC Alarm (A and B) through EXTI Line 
      CEC_IRQHandler,	        // 0xe8 HDMI CEC
      TIM12_IRQHandler,	        // 0xec TIM12                                         
      TIM13_IRQHandler, 	// 0xf0 TIM13                                        
      TIM14_IRQHandler,	        // 0xf4 TIM14                       
      0,    	                // 0xf8 RESERVED                                         
      0,   	                // 0xfc RESERVED 
      0,		        // 0x100 RESERVED 
      0,		        // 0x104 RESERVED 
      TIM5_IRQHandler,		// 0x108 TIM5                                                         
      SPI3_IRQHandler,		// 0x10c SPI3                                                         
      0,		        // 0x110 RESERVED
      0,		        // 0x114 RESERVED
      TIM6_DAC1_IRQHandler,	// 0x118 TIM6 and DAC1 underrun errors                              
      TIM7_IRQHandler,		// 0x11c TIM7                                                         
      DMA2_Stream0_IRQHandler,	// 0x120 DMA2 Stream 0                                                
      DMA2_Stream1_IRQHandler,	// 0x124 DMA2 Stream 1                                                
      DMA2_Stream2_IRQHandler,	// 0x128 DMA2 Stream 2                                                
      DMA2_Stream3_IRQHandler,	// 0x12c DMA2 Stream 3                                                
      DMA2_Stream4_IRQHandler,	// 0x130 DMA2 Stream 4                                                
      SDADC1_IRQHandler,	// 0x134 SDADC1
      SDADC2_IRQHandler,	// 0x138 SDADC2
      SDADC3_IRQHandler,	// 0x13c SDACD3
      COMP1_2_IRQHandler,	// 0x140 Comparator 1&2
      0,	                // 0x144 RESERVED
      0,	                // 0x148 RESERVED
      0,	                // 0x14c RESERVED
      0,	                // 0x150 RESERVED
      0,	                // 0x154 RESERVED
      0,	                // 0x158 RESERVED
      0,	                // 0x15c RESERVED
      0,	                // 0x160 RESERVED
      0,	                // 0x164 RESERVED   
      USB_HP_IRQHandler,	// 0x168 USB High Priority             
      OSB_LP_IRQHandler,	// 0x16c USB Low Priority
      USB_WKUP_IRQHandler,	// 0x170 USB wakeup
      0,	                // 0x174 RESERVED
      TIM19_IRQHandler,		// 0x178 Timer 19
      0,		        // 0x17c RESERVED
      0,	                // 0x180 RESERVED
      FPU_IRQHandler		// 0x184 Floating Point
};

//void (*RAMVectors[NVECS])(void) __attribute__ (( aligned(32) ));
#define NVECS (sizeof(FlashVectors)/sizeof(FlashVectors[0]))
//68
void (*const RAMVectors[NVECS]) (void) __attribute__ ((section("vtable")));

extern void __data_start__;
extern void __data_end__;
extern uint32_t currentClockRate;
extern int32_t ClockRateHigh;

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;
//*****************************************************************************
// Reset entry point: sets up the crt and/or the c/c++ libraries
//*****************************************************************************
void Reset_Handler(void)
{

  //  Call SystemInit() for clocking/memory setup prior to scatter load 
  SystemInit();

  //
  // Copy the data sections from flash to SRAM.
  //
  unsigned int LoadAddr, ExeAddr, SectionLen;
  unsigned int *SectionTableAddr;
  unsigned int loop;


  // Load base address of Global Section Table
  SectionTableAddr = &__data_section_table;

  // Copy the data sections from flash to SRAM.
  while (SectionTableAddr < &__data_section_table_end) {
    LoadAddr = *SectionTableAddr++;
    ExeAddr = *SectionTableAddr++;
    SectionLen = *SectionTableAddr++;

    unsigned int *Dest = (unsigned int *) ExeAddr;
    unsigned int *Src = (unsigned int *) LoadAddr;
    for (loop = 0; loop < SectionLen; loop += sizeof(unsigned int)) {
      *Dest++ = *Src++;
    }
  }
  // At this point, SectionTableAddr = &__bss_section_table;
  // Zero fill the bss segments
  while (SectionTableAddr < &__bss_section_table_end) {
    ExeAddr = *SectionTableAddr++;
    SectionLen = *SectionTableAddr++;

    unsigned int *Dest = (unsigned int *) ExeAddr;
    for (loop = 0; loop < SectionLen; loop += sizeof(unsigned int)) {
      *Dest++ = 0;
    }
  }

  // Ensure 8-byte alignment of stack pointer on interrupts 
  // Enabled by default on most Cortex-M parts, but not M3 r1 
  SCB_CCR |= SCB_CCR_STKALIGN;

  // copy the interrupt vector table to RAM and point to it
  //memcpy(&RAMVectors,&FlashVectors,NVECS*sizeof( void(*)(void) ) );
  unsigned int *Dest = (unsigned int *) &RAMVectors;
  unsigned int *Src = (unsigned int *) &FlashVectors;
  unsigned int length = NVECS * sizeof(void (*)(void));
  for (loop = 0; loop < length; loop += sizeof(unsigned int)) {
    *Dest++ = *Src++;
  }
#if defined(CORE_M3) || defined(CORE_M4)
  unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;
  *pSCB_VTOR = (unsigned int) &RAMVectors;
#endif

#if defined (__cplusplus)
  //
  // Call C++ library initialisation
  //
  __libc_init_array();
#endif

  // Constructors. 
  for (fp = &__preinit_array_start; fp < &__preinit_array_end; fp++) {
    (*fp)();
  }
  for (fp = &__init_array_start; fp < &__init_array_end; fp++) {
    (*fp)();
  }

  // re-assert static values essential to main()
  //currentClockRate = Chip_Clock_GetRate(CLK_MX_MXCORE);	// should be 204 MHz, or close thereto
  //ClockRateHigh = 1;
  // Call main()
  main();

  //
  // main() shouldn't return, but if it does, we'll just enter an infinite loop 
  //
  while (1) {
    ;
  }

  // Destructors. 
  for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
    (*fp)();
  }
  
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{
  while (1) {
  }
}

typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;			//  Link Register. 
  uint32_t pc;			//  Program Counter. 
  uint32_t psr;			//  Program Status Register. 
} hard_fault_stack_t;

__attribute__ ((naked))
void hard_fault_handler(void)
{
  __asm__("TST LR, #4");
  __asm__("ITE EQ");
  __asm__("MRSEQ R0, MSP");
  __asm__("MRSNE R0, PSP");
  __asm__("B hard_fault_handler_c");
}

volatile hard_fault_stack_t *hard_fault_stack_pt;

__attribute__ ((section(".after_vectors")))
void HardFault_Handler(void *args)
{
  volatile int j = 0;
  volatile int count = 30;
  //  hard_fault_stack_pt contains registers saved before the hard fault 
  hard_fault_stack_pt = (hard_fault_stack_t *) args;
  // see: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/Cihcfefj.html
  // see: http://www.keil.com/appnotes/files/apnt209.pdf
  // args[0-7]: r0, r1, r2, r3, r12, lr, pc, psr
  // Other interesting registers to examine:
  //    CFSR: Configurable Fault Status Register (0xE000ED28)
  //    HFSR: Hard Fault Status Register (0xE000ED2C)
  //    DFSR: Debug Fault Status Register 
  //    AFSR: Auxiliary Fault Status Register (0xE000ED3C)
  //    MMAR: MemManage Fault Address Register (0xE000ED34)
  //    BFAR: Bus Fault Address Register (0xE000ED38)

  /*
     if( SCB->HFSR & SCB_HFSR_FORCED ) {        
     if( SCB->CFSR & SCB_CFSR_BFSR_BFARVALID ) {
     SCB->BFAR;
     if( SCB->CFSR & CSCB_CFSR_BFSR_PRECISERR ) {
     }
     }
     }
   */
#ifdef HARDFAULT_SHOULD_RESET
  while (count--) {
#else
  while (count) {
#endif
    //AttnLED (0, 0, 255);
    for (j = 1000000; j > 0; j--);
    //AttnLED (0, 0, 0);
    for (j = 1000000; j > 0; j--);
  }
  selfReset();
}

__attribute__ ((section(".after_vectors")))
void MemManage_Handler(void)
{
  while (1) {
  }
}

__attribute__ ((section(".after_vectors")))
void BusFault_Handler(void)
{
  while (1) {
  }
}

__attribute__ ((section(".after_vectors")))
void UsageFault_Handler(void)
{
  while (1) {
  }
}

__attribute__ ((section(".after_vectors")))
void DebugMon_Handler(void)
{
  while (1) {
  }
}


//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{
  while (1) {
  }
}

void selfReset(void)
{
  scb_reset_system();
}



 //****************************************************************************
 // S y s t e m I n i t
 //****************************************************************************

 /**
 * SystemInit() is called prior to the application and sets up system
 * clocking, memory, and any resources needed prior to the application
 * starting.
 */
void SystemInit(void)
{
#if defined(CORE_M3) || defined(CORE_M4)
#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
  fpuInit();
#endif
#endif
  
  // Now setup the clocks ...
  // Discovery is 8MHz crystal, use 120MHz core
  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_OTGFS);

  return;
}
