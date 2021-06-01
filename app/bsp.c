/*******************************************************************************
*           Copyright (C) 2015 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     bsp.c
*
* Description: BSP "driver" For project. The pressure sensor board is based on
*              STM32F373CCT 
*
*******************************************************************************/
//#define TRACE_PRINT 1

#define GLOBAL_BSP
#include "bsp.h"
#include "OSandPlatform.h"


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

void usbLEDon(int on)
{
  if (on) {
    gpio_set(USBLED);
  } else {
    gpio_clear(USBLED);
  }
  return; 
}

void setupClocks(void)
{
  //rcc_clock_setup_hsi(&(rcc_hsi_8mhz[1]));

  //16MHz crystal, 72MHz cpu, 48MHz USB
  const struct rcc_clock_scale rcc_16mhz_config = {
    .pllsrc=RCC_CFGR_PLLSRC_HSE_PREDIV,
    .pllmul=RCC_CFGR_PLLMUL_MUL9,   //mul 8 MHz by 9 = 72
    .plldiv=RCC_CFGR2_PREDIV_DIV2,  //divide 16MHz by 2
    .usbdiv1=false,  // 72 x 2/3
    .flash_waitstates=2,
    .hpre=RCC_CFGR_HPRE_NODIV, //AHB Prescale div by 1
    .ppre1=RCC_CFGR_PPRE_DIV2, //APB1 div by 2
    .ppre2=RCC_CFGR_PPRE_DIV2, //APB2 div by 2
    .ahb_frequency=72e6,
    .apb1_frequency=36e6,
    .apb2_frequency=36e6,
  };
  rcc_clock_setup_pll(&rcc_16mhz_config);


  FLASH_ACR |= (1<<4); //Turn on Prefetch buffer (Must be when clock is < 24MHz)
  
  SystemCoreClock = rcc_ahb_frequency;

  rcc_periph_clock_enable(RCC_SYSCFG);
  rcc_periph_reset_hold(RST_USB);
  return;
}

void setupGPIOs(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
 
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

//Setup I2C2 with SCL on PA9, and SDA on PA10
void setupI2C2(void)
{
        rcc_periph_clock_enable(RCC_I2C2);
        rcc_set_i2c_clock_hsi(I2C2);

        i2c_reset(I2C2);
        /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10 );
        gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
        i2c_peripheral_disable(I2C2);
        //configure ANFOFF DNF[3:0] in CR1
        i2c_enable_analog_filter(I2C2);
        i2c_set_digital_filter(I2C2, 0);
        /* Using HSI clock, and it's at 8Mhz */
        i2c_set_speed(I2C2, i2c_speed_sm_100k, 8);
        //configure No-Stretch CR1 (only relevant in slave mode)
        i2c_enable_stretching(I2C2);
        //addressing mode
        i2c_set_7bit_addr_mode(I2C2);
        i2c_peripheral_enable(I2C2);
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
