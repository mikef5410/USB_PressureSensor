/*******************************************************************************
*           Copyright (C) 2021 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     MS860702.h
*
* Description: MS860702 "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#ifndef _MS860702_INCLUDED
#define _MS860702_INCLUDED

#include "OSandPlatform.h"

#ifdef GLOBAL_MS860702
#define MS860702GLOBAL
#define MS860702PRESET(A) = (A)
#else
#define MS860702PRESET(A)
#ifdef __cplusplus
#define MS860702GLOBAL extern "C"
#else
#define MS860702GLOBAL extern
#endif	/*__cplusplus*/
#endif				/*GLOBAL_MS860702 */

// ----------------------------------------------------------------
// PRIVATE API AND SUBJECT TO CHANGE!
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// PUBLIC API definition
// ----------------------------------------------------------------

#endif				//_MS860702_INCLUDED
