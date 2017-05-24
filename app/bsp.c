/*******************************************************************************
*           Copyright (C) 2015 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     bsp.c
*
* Description: BSP "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#define GLOBAL_BSP
#include "bsp.h"
#include "OSandPlatform.h"

/*
const struct clock_scale_t clockF373_16mhz = 
  { // 16MHz Crystal, 64MHz 
    .pll = RCC_CFGR_PLLMUL_PLL_IN_CLK_X16,
    .pllsrc = RCC_CFGR_PLLSRC_HSE_PREDIV,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE1_DIV_2,
    .ppre2 = RCC_CFGR_PPRE2_DIV_NONE,
    .power_save = 1,
    .flash_config = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS,
    //.ahb_frequency	= 64000000,
    .apb1_frequency = 32000000,
    .apb2_frequency = 64000000,
  };
*/
extern const struct rcc_clock_scale rcc_hsi_8mhz[];


void greenOn(int on)
{
  if (on) {
    gpio_set(GREENLED);
  } else {
    gpio_clear(GREENLED);
  }
  return;
}


void redOn(int on)
{
  if (on) {
    gpio_set(REDLED);
  } else {
    gpio_clear(REDLED);
  }
  return;
}



void setupClocks(void)
{

  //rcc_clock_setup_hse_3v3(&clockF373_16MHz);
  rcc_clock_setup_hsi(&(rcc_hsi_8mhz[1]));
  SystemCoreClock = 48000000;
  rcc_usb_prescale_1();
  return;
}

void setupGPIOs(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);

  // Setup USBOTG Clocking and pins
  // GPIO A11 = USB_DM, A12 = USB_DP, Alternate function 0
  rcc_periph_clock_enable(RCC_USB);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF0, GPIO11 | GPIO12);
  

  //Red LED
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0);
  gpio_clear(GPIOA,GPIO0);
  
  //Grn LED
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0);
  gpio_clear(GPIOB,GPIO0);

  //USB LED
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO13);
  gpio_clear(GPIOC,GPIO13);


  eeprom9366_init();
  return;
}

void setupNVIC(void)
{
  const uint32_t interrupt_priority_group4 = (0x3 << 8); // 15 priority interrupts, no sub-priorities
  scb_set_priority_grouping(interrupt_priority_group4);
  for (int irqNum=0; irqNum<=NVIC_IRQ_COUNT ; irqNum++) {
    nvic_set_priority(irqNum, 0x6f);
  }
  nvic_set_priority(-4,0); //MMU Fault
  nvic_set_priority(-5,0); //Bus Fault
  nvic_set_priority(-6,0); //Usage Fault
  nvic_set_priority(-11,0); //SVCall
  nvic_set_priority(-14,0); //PendSV
  nvic_set_priority(-15,0); //SysTick

  return;
}

void setupTimers(void)
{
  //  xTimers[0] = xTimerCreate("HVTimer", (HVTimeout/portTICK_RATE_MS),
  //          pdFALSE, (void *) 0, vhvTimerExpired );
}
  

void Delay(volatile uint32_t nCount)
{
  while (nCount--) {
    __asm__("nop");
  }
}
