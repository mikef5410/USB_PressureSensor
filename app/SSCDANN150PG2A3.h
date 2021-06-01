/*******************************************************************************
*           Copyright (C) 2021 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     SSCDANN150PG2A3.h
*
* Description: SSCDANN150PG2A3 "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#ifndef _SSCDANN150PG2A3_INCLUDED
#define _SSCDANN150PG2A3_INCLUDED

#include "OSandPlatform.h"

#ifdef GLOBAL_SSCDANN150PG2A3
#define SSCDANN150PG2A3GLOBAL
#define SSCDANN150PG2A3PRESET(A) = (A)
#else
#define SSCDANN150PG2A3PRESET(A)
#ifdef __cplusplus
#define SSCDANN150PG2A3GLOBAL extern "C"
#else
#define SSCDANN150PG2A3GLOBAL extern
#endif	/*__cplusplus*/
#endif				/*GLOBAL_SSCDANN150PG2A3 */

// ----------------------------------------------------------------
// PRIVATE API AND SUBJECT TO CHANGE!
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// PUBLIC API definition
// ----------------------------------------------------------------

#endif				//_SSCDANN150PG2A3_INCLUDED
