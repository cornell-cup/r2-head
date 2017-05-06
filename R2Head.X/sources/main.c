#include "../includes/config.h"
#include "../includes/pt_cornell_1_2_1.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>

#include "../includes/usb/usb.h"
#include "../includes/usb/usb_function_cdc.h"
#include "../includes/HardwareProfile.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"

// === SPI setup ========================================================
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 256 ; // 20 MHz max speed for this RAM

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_spiWrite, pt_spiReadPos, pt_usb;

static unsigned int cmd_NOP;
static unsigned int cmd_rdpos;
static unsigned int cmd_zeroPt;
static uint16_t encoder =0;
volatile uint8_t enc_read;

//// UART parameters
//#define BAUDRATE 9600 // must match PC end
//#define PB_DIVISOR (1 << OSCCONbits.PBDIV) // read the peripheral bus divider, FPBDIV
//#define PB_FREQ SYS_FREQ/PB_DIVISOR // periperhal bus frequency
//
//// useful ASCII/VT100 macros for PuTTY
//#define clrscr() printf( "\x1b[2J")
//#define home()   printf( "\x1b[H")
//#define pcr()    printf( '\r')
//#define crlf     putchar(0x0a); putchar(0x0d);
//#define backspace 0x08
//#define str_buffer_size 20
//#define max_chars 32 // for input buffer
//#define timer2rate 625000 //ticks per 1sec
//#define max_pwm 40000 //PWM

 //UART Initializations
//int count, items, num_char;
//static char rxchar='0'; 		//received character
//int num;
//char str_buffer[str_buffer_size];


static PT_THREAD (protothread_UART(struct pt *pt))
{
    printf("PT_THREAD1\r\n");
    printf("%x\r\n",enc_read);
    PT_BEGIN(pt);
      while(1) {
        //printf("this is working");
        PT_YIELD_TIME_msec(1000);
        //PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output) );
        }
        // never exit while
        
       // END WHILE(1)
  PT_END(pt);
  }
static PT_THREAD (protothread_spiWrite(struct pt *pt))
{
    printf("PT_THREAD2\r\n");
    printf("%x\r\n",enc_read);
    PT_BEGIN(pt);
    cmd_NOP = 0xAA;//0; //0x00
    while(1){
  
        enc_read = tft_spiwrite8(cmd_NOP);
        PT_YIELD_TIME_msec(1);
        //return enc_read;
    }
    PT_END(pt);
}

static PT_THREAD (protothread_usb(struct pt *pt))
{
    //printf("PT_THREAD3\r\n");
    //printf("%x\r\n",enc_read);
    PT_BEGIN(pt);
    cmd_zeroPt = 0x70; //0x70
    //printf("encoder is 0x01\r\n");
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
        char sourceBuffer[30] = {0};
        char transactionBuffer[30] = {0};
        char payloadBuffer[30] = {0};
        char checksumBuffer[30] = {0};
        
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        int result = ProcessIO(sourceBuffer, payloadBuffer, checksumBuffer, transactionBuffer);
        /* the buffers now contain relevant information;
         * they are updated if result == 1; otherwise, it's old info
         */
        
        char readBuffer[100];
//        if (result){
            // new data available
            
//            print out data obtained:
//            sprintf(readBuffer,
//                "S: %s\n\rT: %s\n\rP: %s\n\rK: %s\n\r",
//                    sourceBuffer, transactionBuffer,
//                        payloadBuffer, checksumBuffer);
//        sprintf(readBuffer, "hello");
//        putsUSBUSART(readBuffer);
        
//        }
    }
    PT_END(pt);
}

static PT_THREAD (protothread_spiReadPos(struct pt *pt))
{
    
//    printf("%x\r\n",enc_read);
    PT_BEGIN(pt);
    printf("PT_THREAD4\r\n");
    cmd_rdpos = 16; //0x10
    cmd_NOP = 0xAA;
    
    char buf[60];
    static unsigned int state = 1;
    while(1){
        
        
        switch (state) {
            case 1:
                enc_read = tft_spiwrite8(cmd_NOP);
                if (enc_read==0xA5) state = 2;
                sprintf(buf,"state 1, 0x%0X\r\n",enc_read);
                break;
            case 2:
                enc_read = tft_spiwrite8(cmd_rdpos);
                state = 3;
                sprintf(buf,"state 2\r\n");
                break;
                //if (enc_read==0xA5)  
            case 3:
                enc_read =0xA5;
                sprintf(buf,"state 3\r\n");
                while (enc_read==0xA5) enc_read = tft_spiwrite8(cmd_NOP);
                if (enc_read ==0x10) state =4;
                break;
            case 4:
                encoder = tft_spiwrite8(cmd_NOP);
                encoder = encoder<<8;
                encoder |= tft_spiwrite8(cmd_NOP);
                sprintf(buf,"encoder is 0x%0X\r\n", encoder);
                state = 1;
                break;
            //default:
                
        }
        putsUSBUSART(buf);
//        enc_read = tft_spiwrite(cmd_rdpos); // Step 1: Master sends rd_pos command. Encoder responds with idle character.
//        printf("ReadSPI2: %x\r\n",enc_read);
//        while (enc_read == 165){ //convert hex to int 0xa5   165
//            printf("while 165: %x\r\n",enc_read);
//            printf("encoder must be 0xa5\r\n");
//             // Step 2: Continue sending nop_a5 command while encoder response is 0xA5
//            enc_read = tft_spiwrite(cmd_NOP);;
//        }
//        if(enc_read == 16){ //0x10    16
//            printf("if 16: %x\r\n",enc_read);
//            // Step 3: If response was 0x10 (rd_pos), send nop_a5 and receive MSB position
//            // (lower 4 bits of this byte are the upper 4 of the 12-bit position)
//            printf("enc_read is finally 0x10\r\n");
//            
//            pos_MSB = tft_spiwrite(cmd_NOP);
//             // Step 4: Send second nop_a5 command and receive LSB position (lower 8 bits of 12-bit positon)
//            pos_LSB = tft_spiwrite(cmd_NOP);
////            sprintf(pos_MSB,pos_LSB);
       // }
        //printf(&enc_read);
//        PT_YIELD_TIME_msec(500);
        PT_YIELD(pt);
        //return pos_MSB, pos_LSB;
    }
    PT_END(pt);
}
// === Main ======================================================
void main(void) {
    __XC_UART = 2;

    /* 	Initialize PPS */
    // specify PPS group, signal, logical pin name
    PPSInput (2, U2RX, RPB11); // Assign U2RX to pin RPB11 -- Physical pin 22 on 28 PDIP
    PPSOutput(4, RPB10, U2TX); // Assign U2TX to pin RPB10 -- Physical pin 21 on 28 PDIP

    mPORTBSetBits(BIT_2); // RPB2 for CSB -- Physical pin 6 on 28 PDIP.
    mPORTBSetPinsDigitalOut(BIT_3); // RPB3 for SCK -- Physical pin 7 on 28 PDIP.
    
    //SYSTEMConfigPerformance(PBCLK);
    ANSELA = 0; ANSELB = 0; 
    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();
   
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    // PuTTY
    clrscr();  //clear PuTTY screen
    home();
    // By default, MPLAB XC32's libraries use UART2 for STDOUT.
    // This means that formatted output functions such as printf()
    // will send their output to UART2
    printf("UART Interface Initialized\n\r");
    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    
    mPORTBSetPinsDigitalIn(BIT_3);//pin 7
//    CNPDBSET = BIT_3; //enable internal pulldown
    mPORTBSetPinsDigitalIn(BIT_2);//pin 6
//    CNPDBSET = BIT_2; //enable internal pulldown
    mPORTBSetPinsDigitalIn(BIT_1);//pin 5
//    CNPDBSET = BIT_1; //enable internal pulldown
    
      // === set up SPI ===================
  // SCK2 is pin 26 
  // SDO2 (MOSI) is in PPS output group 2, RPB1 pin 5
   PPSOutput(2, RPB1, SDO2);
  // SDI2 (MISO) is PPS output group 3, RPA4 pin 12
   PPSInput(3,SDI2,RPA4);

  // control CS for RAM
   mPORTBSetPinsDigitalOut(BIT_9);
  //PORTSetPinsDigitalOut(IOPORT_B, BIT_0); //IOPORT_PIN_0
   mPORTBSetBits(BIT_9);
        
  // divide Fpb by 2, configure the I/O ports. Not using SS in this example
  // 8 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
   SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE8 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
   TRIS_cs = 0; //sets as output
   _cs_high(); //de-selects device
	CloseADC10();	// ensure the ADC is off before setting the configuration
    
      // init the threads
    //PT_INIT(&pt_UART);
   PT_INIT(&pt_spiReadPos);
    PT_INIT(&pt_spiWrite);
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
    
    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif
    // round-robin scheduler for threads
    while (1){

//        PT_SCHEDULE(protothread_spiWrite(&pt_spiWrite));
        PT_SCHEDULE(protothread_spiReadPos(&pt_spiReadPos));
        PT_SCHEDULE(protothread_usb(&pt_usb));
        
    }
} // main
