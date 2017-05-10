#include "../includes/config.h"
#include "../includes/pt_cornell_1_2_1.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>

#include "../includes/usb/usb.h"
#include "../includes/usb_main.h"
#include "../includes/usb/usb_function_cdc.h"
#include "../includes/HardwareProfile.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"

#include "../includes/R2Protocol.h"
#ifndef _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#endif
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
#include "plib.h"

#define SERVO_MIN           2000 // 1ms
#define SERVO_REST          3750 // 1.5ms
#define SERVO_MAX           5000 // 2ms
#define SERVO_RUN_SPEED     900
#define SERVO_LEFT          (SERVO_REST + SERVO_RUN_SPEED)
#define SERVO_RIGHT         (SERVO_REST - SERVO_RUN_SPEED)
#define t_revolution        9.3 // in seconds

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_readPos, pt_usb, pt_servoT;

volatile int periodTicks = 0;
volatile int endTime = 0;
volatile int motorEn = 0;

void initEncoder(void);
void initServo(void);
void setHeadSpeed(int speed);

//static PT_
static PT_THREAD (protothread_usb(struct pt *pt))
{
    PT_BEGIN(pt);
    static int result;
    while(1){
        PT_YIELD(pt);
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif
        
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        static struct R2ProtocolPacket comm_packet;
        static uint8_t packetData[30] = {0};
        comm_packet.data_len = 30;
        comm_packet.data = packetData;
        
        result = ProcessIO(&comm_packet);
        if (result == REG_DATA){
            // Process USB transactions here:
            uint8_t c = comm_packet.data[0];
            uint32_t t = atoi(&comm_packet.data[1]);
            if (t < 0) t = 0;
            motorEn = 1;
            endTime = PT_GET_TIME() + t;
            if (c == 'L'){
                setHeadSpeed(SERVO_LEFT);
            }
            if (c == 'R'){
                setHeadSpeed(SERVO_RIGHT);
            }
//            sprintf(readBuffer, "Got %c for %d ms\n\r", c, t);
//            putsUSBUSART(readBuffer);
        }
    }
    PT_END(pt);
}

static PT_THREAD (protothread_servoT(struct pt *pt)){
    PT_BEGIN(pt);
    while (1){
        if (motorEn){
            PT_YIELD_UNTIL(pt, PT_GET_TIME() > endTime);
            motorEn = 0;
            setHeadSpeed(SERVO_REST);
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}

static PT_THREAD (protothread_readPos(struct pt *pt))
{
    PT_BEGIN(pt);
    
    char buf[60];
    PT_YIELD_TIME_msec(250); // Encoder initialization=
    
    while(1){
        PT_YIELD_UNTIL(pt, mIC5GetIntFlag());
        mIC5ClearIntFlag();
        WriteTimer2(0);
        periodTicks = mIC5ReadCapture();
    }
    PT_END(pt);
}
// === Main ======================================================
void main(void) {
    
    ANSELA = 0; ANSELB = 0; 
    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();
   
    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

	CloseADC10();	// ensure the ADC is off before setting the configuration
    
      // init the threads
    PT_INIT(&pt_servoT);
    PT_INIT(&pt_usb);
    
   
   //	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin 
//  has been mapped	to it.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
    
//    initEncoder();
    initServo();
    
    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif
    // round-robin scheduler for threads
    while (1){
        PT_SCHEDULE(protothread_servoT(&pt_servoT));
        PT_SCHEDULE(protothread_usb(&pt_usb));
        
    }
} // main

void initEncoder(void){
    
    // A, B, X input:
    mPORTBSetPinsDigitalIn(BIT_0 | BIT_1 | BIT_2);
    
    // B:
//    PPSInput(4, IC2, RPB0);
    
    // A:
    PPSInput(3, IC5, RPB2);
    
    // X:
    PPSInput(2, INT3, RPB1);
    
    // B:
//    OpenCapture2(IC_ON | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_EVERY_RISE_EDGE);
//    mIC2ClearIntFlag();
//    OpenTimer3(T3_ON | T3_PS_1_1, 0xFFFF);
    
    // A:
    OpenCapture5(IC_ON | IC_TIMER2_SRC | IC_INT_1CAPTURE | IC_EVERY_RISE_EDGE);
    mIC5ClearIntFlag();
    OpenTimer2(T2_ON | T2_PS_1_1, 0xFFFF);
    
    
    
}

void initServo(void){
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBClearBits(BIT_4);
    PPSOutput(1, RPB4, OC1);
    OpenTimer3(T3_ON | T3_PS_1_16, 50000-1);
    OpenOC1(OC_ON | OC_TIMER_MODE16 | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, SERVO_REST, SERVO_REST);
    ConfigIntTimer3(T3_INT_OFF);
    setHeadSpeed(SERVO_REST);
}

void setHeadSpeed(int speed){
    if (speed < SERVO_MIN) speed = SERVO_MIN;
    if (speed > SERVO_MAX) speed = SERVO_MAX;
    SetDCOC1PWM(speed);
}
