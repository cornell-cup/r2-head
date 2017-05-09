/*
 * File:   config.h
 * Author: Syed Tahmid Mahbub
 * Modifed by: Bruce Land 
 * Created on October 10, 2014
 * Mod: 24Sept2015
 */

#ifndef CONFIG_H
#define	CONFIG_H
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
#ifndef _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#endif
#include <plib.h>
#include <stdio.h>

//=============================================================
// 40 MHz
#pragma config FNOSC = PRIPLL, POSCMOD = HS, FSOSCEN = OFF, OSCIOFNC = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, ICESEL = ICS_PGx3
#pragma config UPLLIDIV = DIV_2, UPLLEN = ON

//==============================================================
// Protothreads configure

// IF use_vref_debug IS defined, pin 25 is Vref output
//#define use_vref_debug

// IF use_uart_serial IS defined, pin 21 and pin 22 are used by the uart
//#define use_uart_serial
#define BAUDRATE 9600 // must match PC terminal emulator setting

/////////////////////////////////
// set up clock parameters
// system cpu clock
#define sys_clock 40000000

// sys_clock/FPBDIV
#define pb_clock sys_clock // divide by one in this case

#endif	/* CONFIG_H */

