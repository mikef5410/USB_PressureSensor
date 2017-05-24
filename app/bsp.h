/*******************************************************************************
*           Copyright (C) 2015 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     bsp.h
*
* Description: BSP "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#ifndef _BSP_INCLUDED
#define _BSP_INCLUDED

#include "OSandPlatform.h"

#ifdef GLOBAL_BSP
#define BSPGLOBAL
#define BSPPRESET(A) = A
#else
#define BSPPRESET(A)
#ifdef __cplusplus
#define BSPGLOBAL extern "C"
#else
#define BSPGLOBAL extern
#endif	/*__cplusplus*/
#endif				/*GLOBAL_BSP */

// ----------------------------------------------------------------
// PRIVATE API AND SUBJECT TO CHANGE!
// ----------------------------------------------------------------

#define NUM_TIMERS 2
xTimerHandle xTimers[ NUM_TIMERS ];

// ----------------------------------------------------------------
// PUBLIC API definition
// ----------------------------------------------------------------

BSPGLOBAL uint32_t SystemCoreClock;

BSPGLOBAL void greenOn(int on);
BSPGLOBAL void redOn(int on);
BSPGLOBAL void usbLEDon(int on);
BSPGLOBAL void setupClocks(void);
BSPGLOBAL void setupGPIOs(void);
BSPGLOBAL void setupNVIC(void);
BSPGLOBAL void Delay(volatile uint32_t nCount);
BSPGLOBAL void setupTimers(void);

BSPGLOBAL uint32_t hvState BSPPRESET(0);

#define BSPGPIO(BANK,NUM) GPIO ## BANK,GPIO ## NUM
#define GREENLED BSPGPIO(B,0)
#define REDLED BSPGPIO(A,0)
#define USBLED BSPGPIO(C,13)


#define MSleep(x) Delay((x) * 30000UL)

#endif				//_BSP_INCLUDED
