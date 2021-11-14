//
//  RC_PPM.c
//
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>


#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "defines.h"

//#include "spi.c"
//#include "spi_adc.c"

//#include "spi_slave.c"
//#include "soft_SPI.c"

//#include "ds18x20.c"



#include "chan_n/mmc_avr_spi.c"
#include "chan_n/ff.c"
#include "chan_n/diskio.c"

#include "wireless.c"
#include "nRF24L01.h"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 5

#define SERVOMAX  4400
#define SERVOMIN  1400


#define USB_DATENBREITE USB_PACKETSIZE



#define CODE_OFFSET  4
#define ROTARY_OFFSET  10

/*
 const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};
 */

// lookup-Tabelle KTY84

// pgm_read_word(&KTY[xy])
#define KTY_OFFSET   32             // Offset, Start bei bei -32 �C
#define ADC_OFFSET   370            // Startwert der ADC-Messung
#define KTY_FAKTOR   96             // 0x60, Multiplikator
const uint16_t KTY[] PROGMEM =
{
   0x0,	0x17E,	0x328,	0x4B6,	0x651,	0x7D2,	0x961,	0xAD8,
   0xC5C,	0xDCA,	0xF45,	0x10AB,	0x121F,	0x137E,	0x14DA,	0x1645,
   0x179B,	0x1900,	0x1A52,	0x1BB3,	0x1D01,	0x1E5D,	0x1FA8,	0x2101,
   0x2249,	0x239F,	0x24E4,	0x2638,	0x277B,	0x28CD,	0x2A0E,	0x2B5F,
   0x2C9E,	0x2DEE,	0x2F2C,	0x307B,	0x31B9,	0x3307,	0x3444,	0x3582,
   0x36CF,	0x380C,	0x395A,	0x3A98,	0x3BE5,	0x3D23,	0x3E72,	0x3FB0,
   0x4100,	0x423F,	0x4390,	0x44D0,	0x4622,	0x4764,	0x48B7,	0x49FB,
   0x4B4F,	0x4C95,	0x4DEC,	0x4F33,	0x508C,	0x51D5,	0x5331,	0x547D,
   0x55CA,	0x5729,	0x5879,	0x59DB,	0x5B2E,	0x5C93,	0x5DE9,	0x5F52,
   0x60AB,	0x6217,	0x6374,	0x64E4,	0x6644,	0x67B9,	0x691D,	0x6A96,
   0x6BFF,	0x6D7C,	0x6EE9,	0x706B,	0x71DD,	0x7365,	0x74DC,	0x7669,
   0x77E5,	0x7964,	0x7AFA,	0x7C7F,	0x7E1A,	0x7FA5,	0x8147,	0x82D9,
   0x8482,	0x861A,	0x87CA,	0x8969,	0x8B21,	0x8CC8,	0x8E88,	0x9037,
   0x91FF,	0x93B6,	0x9587,	0x9747,	0x9922,	0x9AEB,	0x9CCF,	0x9EA2,
   0xA091,	0xA26E,	0xA450,	0xA650,	0xA83D,	0xAA49,	0xAC42,	0xAE5B,
   0xB060,	0xB286,	0xB499,	0xB6CD,	0xB8EE,	0xBB31,	0xBD60,	0xBFB3,
   0xC1F2,	0xC457,	0xC6A6,	0xC91C,	0xCB7E,	0xCE07,	0xD07B,	0xD319,
   0xD5A1,	0xD855,	0xDAF3,	0xDD9B,	0xE072,	0xE333,	0xE624,	0xE8FD,
  };


uint16_t key_state;				// debounced key state:
// bit = 1: key pressed
uint16_t key_press;				// key press detect
volatile uint16_t tscounter =0;

extern volatile uint8_t isrcontrol;

volatile uint8_t do_output=0;
//static volatile uint8_t testbuffer[USB_DATENBREITE]={};


//static volatile uint8_t buffer[USB_DATENBREITE]={};


static volatile uint8_t recvbuffer[USB_DATENBREITE]={};

static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

//volatile uint8_t outbuffer[USB_DATENBREITE]={};
//volatile uint8_t inbuffer[USB_DATENBREITE]={};

//static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

//static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

#define TIMER0_STARTWERT	0x40

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];


void delay_ms(unsigned int ms);

//volatile uint8_t payload[wl_module_PAYLOAD];

// WL def
//volatile uint8_t wl_spi_status;


volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;

volatile uint16_t                   startblock = 0;

volatile uint8_t                    packetcount = 0;

volatile uint8_t                    blockanzahl = 0;

volatile uint8_t                    downloadblocknummer = 0;

// counter fuer Mess-Intervall
volatile uint16_t                    intervallcounter=0;

// mess-Intervall
volatile uint16_t                    intervall=1; // defaultwert, 1s
// counter fuer Mess-Intervall
volatile uint16_t                    messungcounter=0; // Anzahl messungen fortlaufend


//static volatile uint8_t            substatus=0x00; // Tasks fuer Sub
static volatile uint8_t             hoststatus=0x00;

static volatile uint8_t             usbstatus=0x00;
static volatile uint8_t             usbstatus1=0x00; // recvbuffer[1]

volatile uint8_t                    spistatus=0x00; // was ist zu tun infolge spi

//static volatile uint8_t             eepromstatus=0x00;
static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0;

static volatile uint8_t             tastaturstatus = 0;

volatile uint8_t status=0;




// Logger
volatile uint16_t messintervall = 1;
volatile uint16_t saveSDposition = 0;
volatile uint16_t blockcounter = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).
volatile uint16_t startminute = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).

// WL
volatile uint8_t                   wl_isr_counter = 0;

// PWM

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;


//static volatile uint8_t spi_rxbuffer[SPI_BUFSIZE];
//static volatile uint8_t spi_txbuffer[SPI_BUFSIZE];
static volatile uint8_t spi_rxdata=0;

static volatile uint8_t inindex=0;

volatile char SPI_data='0';
//volatile char SPI_dataArray[SPI_BUFSIZE];
//volatile uint16_t Pot_Array[SPI_BUFSIZE];

//volatile uint16_t Mitte_Array[8];

//volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
//volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

//volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


//volatile uint16_t RAM_Array[SPI_BUFSIZE];

//volatile uint8_t testdataarray[8]={};
volatile uint16_t teststartadresse=0xA0;


volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
//volatile uint8_t usb_readcount = 0;

volatile uint8_t  eeprom_indata=0;

#pragma mark WL def
//Variablen WL
// MARK: WL Defs
volatile uint8_t wl_status=0;

volatile uint8_t wl_recv_status=0;

volatile uint8_t wl_send_status=0;




//volatile uint8_t PTX=0;
volatile uint8_t int0counter=0;
volatile uint8_t int1counter=0;


volatile uint8_t wl_spi_status=0;
char itoabuffer[20] = {};
volatile uint8_t wl_data[wl_module_PAYLOAD] = {};
//volatile uint8_t wl_data_array[4][wl_module_PAYLOAD] = {};

volatile uint8_t pipenummer = 1; // nur pipes < 7

volatile uint8_t loop_pipenummer = 2;
volatile uint8_t loop_rt_pipenummer = 1;

volatile uint8_t akt_pipenummer = 1;

volatile uint8_t wl_blockedcounter=0;
volatile uint8_t wl_sendcounter=0;


#pragma mark mmc def
FATFS Fat_Fs;		/* FatFs work area needed for each volume */
FIL Fil;			/* File object needed for each open file */
//FATFS FatFs[2];		/* File system object for each logical drive */
FIL File[2];		/* File object */
DIR Dir;			/* Directory object */
FILINFO Finfo;
DWORD AccSize;				/* Work register for fs command */
WORD AccFiles, AccDirs;

#define WRITENEXT 1
#define WRITETAKT 0x0032
BYTE RtcOk;				/* RTC is available */
volatile UINT Timer;	/* Performance timer (100Hz increment) */
volatile uint8_t mmcbuffer[SD_DATA_SIZE] = {};
//const uint8_t rambuffer[SD_DATA_SIZE] PROGMEM = {};
const uint8_t databuffer[SD_DATA_SIZE] PROGMEM = {};
//volatile uint8_t writebuffer[512] = {};
volatile uint8_t mmcstatus = 0;
volatile uint16_t                   writecounter1=0; // Takt fuer write to SD
volatile uint16_t                   writecounter2=0; // Takt fuer write to SD

volatile uint16_t                   datapendcounter=0; // Takt fuer write to SD


//#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR 16 MHz
#define CLOCK_DIV 8 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz
#define BLINK_DIV 4 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz



volatile uint16_t                TastaturCount=0;
volatile uint16_t                manuellcounter=0; // Counter fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
volatile uint8_t                 blinkcounter=0;


uint8_t                          ServoimpulsSchrittweite=10;
uint16_t                         Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};




//volatile uint16_t                SPI_Data_counter; // Zaehler fuer Update des screens



volatile uint16_t Tastenwert=0;
volatile uint16_t Trimmtastenwert=0;
volatile uint8_t adcswitch=0;


/*
 #define TASTE1		15
 #define TASTE2		23
 #define TASTE3		34
 #define TASTE4		51
 #define TASTE5		72
 #define TASTE6		94
 #define TASTE7		120
 #define TASTE8		141
 #define TASTE9		155
 #define TASTE_L	168
 #define TASTE0		178
 #define TASTE_R	194
 
 */
//const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};

/*
 
 static inline
 uint16_t key_no( uint8_t adcval )
 {
 uint16_t num = 0x1000;
 PGM_P pointer = wertearray;
 
 
 while( adcval < pgm_read_byte(pointer))
 {
 pointer++;
 num >>= 1;
 }
 return num & ~0x1000;
 }
 
 uint16_t get_key_press( uint16_t key_mask )
 {
 cli();
 key_mask &= key_press;		// read key(s)
 key_press ^= key_mask;		// clear key(s)
 sei();
 return key_mask;
 }
 
 */


#pragma mark 1-wire

//#define MAXSENSORS 2
//static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
//static int16_t gTempdata[MAXSENSORS]; // temperature times 10
//static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
//static int8_t gNsensors=0;

uint8_t search_sensors(void)
{
   return 0;
   /*
   uint8_t i;
   uint8_t id[OW_ROMCODE_SIZE];
   uint8_t diff, nSensors;
   
   ow_reset();
   
   nSensors = 0;
   
   diff = OW_SEARCH_FIRST;
   while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS )
   {
      DS18X20_find_sensor( &diff, &id[0] );
      
      if( diff == OW_PRESENCE_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("No Sensor found\0" );
         
         delay_ms(800);
         lcd_clr_line(1);
         break;
      }
      
      if( diff == OW_DATA_ERR )
      {
         lcd_gotoxy(10,0);
         lcd_puts("BusErr\0" );
         lcd_puthex(diff);
         return OW_DATA_ERR;
         break;
      }
      //lcd_gotoxy(4,1);
      
      for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      {
         //lcd_gotoxy(15,1);
         //lcd_puthex(id[i]);
         
         gSensorIDs[nSensors][i] = id[i];
         //delay_ms(100);
      }
      
      nSensors++;
   }
   
   return nSensors;
    */
}

// start a measurement for all sensors on the bus:
/*
void start_temp_meas(void)
{
   
   gTemp_measurementstatus=0;
   if ( DS18X20_start_meas(NULL) != DS18X20_OK)
   {
      gTemp_measurementstatus=1;
   }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
   uint8_t i;
   uint8_t subzero, cel, cel_frac_bits;
   for ( i=0; i<gNsensors; i++ )
   {
      
      if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
                             &cel, &cel_frac_bits) == DS18X20_OK )
      {
         gTempdata[i]=cel*10;
         gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
         if (subzero)
         {
            gTempdata[i]=-gTempdata[i];
         }
      }
      else
      {
         gTempdata[i]=0;
      }
   }
}

*/
// Code 1_wire end



void startTimer2(void)
{
   //timer2
   TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void Master_Init(void)
{
   LOOPLEDDDR |=(1<<LOOPLED);
   LOOPLEDPORT |= (1<<LOOPLED);	// HI
   
   SPI_DDR |=(1<<SPI_MOSI);
   SPI_PORT |= (1<<SPI_MOSI);	// HI
   
   //Pin 0 von   als Ausgang fuer OSZI
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   
   OSZIPORTDDR |= (1<<PULSB);		//Pin 1 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSB);		//Pin   von   als Ausgang fuer OSZI
   
   
   //OSZIPORTDDR &= ~(1<<TEST_PIN);		//Pin 1 von  als Eingang fuer Test-Pin. active LO
   //OSZIPORT |= (1<<TEST_PIN);		//Pin   von   als Ausgang fuer OSZI

   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang f�r Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   
   // ---------------------------------------------------
   // Pin Change Interrupt enable on PCINT0 (PD7)
   // ---------------------------------------------------
   
   // PCIFR |= (1<<PCIF0);
   // PCICR |= (1<<PCIE0);
   //PCMSK0 |= (1<<PCINT7);
   
   // ---------------------------------------------------
   // USB_Attach
   // ---------------------------------------------------
   
   EICRA |= (1<<ISC01); // falling edge
   EIMSK=0;
   EIMSK |= (1<<INT0); // Interrupt en
   
   
   // ---------------------------------------------------
   //LCD
   // ---------------------------------------------------
   LCD_DDR |= (1<<LCD_RSDS_PIN);		// PIN als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   
}


void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   
   
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   //SPI_PORT &= ~(1<<SPI_MISO; // HI
   SPI_DDR |= (1<<SPI_MOSI);
   SPI_DDR |= (1<<SPI_CLK);
   //SPI_PORT &= ~(1<<SPI_CLK); // LO
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
   
   
   /*
    // Slave init
    SPI_DDR |= (1<<SPI_MISO); // Output
    //SPI_PORT &= ~(1<<SPI_MISO; // HI
    SPI_DDR &= ~(1<<SPI_MOSI); // Input
    SPI_DDR &= ~(1<<SPI_CLK); // Input
    //SPI_PORT &= ~(1<<SPI_SCK; // LO
    SPI_DDR &= ~(1<<SPI_SS); // Input
    SPI_PORT |= (1<<SPI_SS); // HI
    */
   
   
   
}

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   
   SPI_ADC_CE_DDR |= (1<<SPI_ADC_CE);
   SPI_ADC_CE_PORT |= (1<<SPI_ADC_CE); // HI
}

void SPI_Master_init (void)
{
   
   SPCR |= (1<<MSTR);// Set as Master
   
   //  SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
   /*
    SPI2X 	SPR1 	SPR0     SCK Frequency
    0       0        0     fosc/4
    0       0        1     fosc/16
    0       1        0     fosc/64
    0       1        1     fosc/128
    1       0        0     fosc/2
    1       0        1     fosc/8
    1       1        0     fosc/32
    1       1        1     fosc/64
    */
   
   //SPCR |= (1<<SPR0);               // div 16 SPI2X: div 8
   SPCR |= (1<<SPR1);               // div 64 SPI2X: div 32
   //SPCR |= (1<<SPR1) | (1<<SPR0);   // div 128 SPI2X: div 64
   //SPCR |= (1<<SPI2X0);
   
   SPCR |= (1<<SPE); // Enable SPI
   status = SPSR;								//Status loeschen
   
}


void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   SPI_PORT &= ~(1<<SPI_MISO); // LO
   
   SPI_DDR |= (1<<SPI_MOSI);
   SPI_PORT &= ~(1<<SPI_MOSI); // LO
   
   SPI_DDR |= (1<<SPI_CLK);
   SPI_PORT &= ~(1<<SPI_CLK); // LO
   
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
}

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI); // MOSI off
   SPI_DDR &= ~(1<<SPI_CLK); // SCK off
   SPI_DDR &= ~(1<<SPI_SS); // SS off
   
   //SPI_RAM_DDR &= ~(1<<SPI_RAM_CS; // RAM-CS-PIN off
   //SPI_EE_DDR &= ~(1<<SPI_EE_CS; // EE-CS-PIN off
}

/*
 void spi_slave_init()
 {
 SPCR=0;
 SPCR = (1<<SPE)|(1<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA);
 
 }
 */

void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
   // we use a calibrated macro. This is more
   // accurate and not so much compiler dependent
   // as self made code.
   while(ms){
      _delay_ms(0.96);
      ms--;
   }
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

/* nicht verwendet
 void timer1_init(void)
 {
 // Quelle http://www.mikrocontroller.net/topic/103629
 
 _HI ; // Test: data fuer SR
 _delay_us(5);
 //#define FRAME_TIME 20 // msec
 KANAL_DDR |= (1<<KANAL; // Kanal Ausgang
 
 DDRD |= (1<<PORTD5); //  Ausgang
 PORTD |= (1<<PORTD5); //  Ausgang
 
 //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
 //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
 TCCR1B |= (1<<CS11);
 TCNT1  = 0;														// reset Timer
 
 // Impulsdauer
 OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
 
 TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
 TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
 } // end timer1
 */
/*
 void timer1_stop(void)
 {
 // TCCR1A = 0;
 
 }
 */
/*
 ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
 {
 
 KANAL_HI;
 impulscounter++;
 
 if (impulscounter < ANZ_POT)
 {
 // Start Impuls
 
 TCNT1  = 0;
 //KANAL_HI;
 
 // Laenge des naechsten Impuls setzen
 
 //OCR1A  = POT_FAKTOR*Pot_Array[1]; // 18 us
 //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter]; // 18 us
 
 }
 }
 */

/*
 ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
 {
 //OSZI_A_LO ;
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 //OSZI_A_HI ;
 KANAL_LO;
 
 if (impulscounter < ANZ_POT)
 {
 }
 else
 {
 timer1_stop();
 
 }
 }
 */


void timer0 (void) // Grundtakt fuer Stoppuhren usw.
{
   // Timer fuer Exp
   //TCCR0 |= (1<<CS01);						// clock	/8
   //TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
   //TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
   //TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
   
   //TCCR0B |= (1 << CS02);//
   //TCCR0B |= (1 << CS00);
   
   // TCCR0B |= (1 << CS10); // Set up timer
   
   //OCR0A = 0x02;
   
   //TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   //TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
   //TCNT0 = 0;					//R�cksetzen des Timers
   
   
   // chan_n
   /* Start 100Hz system timer with TC0 */
   OCR0A = F_CPU / 1024 / 100 - 1;
   TCCR0A |= (1<<WGM01);
   TCCR0B |= (1 << CS02);//
   TCCR0B |= (1 << CS00);
   
   //TCCR0B = 0b101;
   TIMSK0 |= (1<<OCIE0A);
   // lcd_putc('9');
   
}


 void timer2 (uint8_t wert)
 {
 //timer2
    TCCR2B |= ( (1<<CS22) | (1<<CS20));
    TIMSK2 |= ( (1<<TOIE2));
 }


volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

/*---------------------------------------------------------*/
/* 100Hz timer interrupt generated by OC0A                 */
/*---------------------------------------------------------*/

#pragma mark TIMER0_COMPA
ISR(TIMER0_COMPA_vect)
{
   //OSZIA_LO;
   //lcd_putc('4');
   //lcd_putc('+');
   //   Timer1--;          /* Performance counter for this module */
   mmc_disk_timerproc();	/* Drive timer procedure of low level disk I/O module */
   
   
   writecounter1++;
   if (writecounter1 >= 2*WRITETAKT) // 1s
   {
      
      intervallcounter++;
      if (intervallcounter >= intervall)
      {
         intervallcounter = 0;
         if (hoststatus & (1<<DOWNLOAD_OK))// Download von SD, Messungen unterbrechen
         {
            
         }
         else
         {
            hoststatus |= (1<<MESSUNG_OK); // Messung ausloesen
         }
         
         
         
      }
      
      
      
      writecounter1=0;
      writecounter2++;
      //      lcd_gotoxy(0,2);
      //      lcd_putint12(writecounter2);
      if (writecounter2 >= 0x0002)
      {
         mmcstatus |= (1<<WRITENEXT);
         writecounter2 = 0;
      }
   }
   
   if ((wl_spi_status & (1<<WL_DATA_PENDENT)))
   {
      datapendcounter++;
   }
   //OSZIA_HI;
}


#pragma mark TIMER0_OVF

ISR (TIMER0_OVF_vect)
{
   lcd_putc('+');
   mscounter++;
   
   if (mscounter%BLINK_DIV ==0)
   {
      blinkcounter++;
   }
   
}

#pragma mark timer1
void timer1(void)
{
   
   //SERVODDR |= (1<<SERVOPIN0);
   /*
    TCCR1A = (1<<WGM10)|(1<<COM1A1)   // Set up the two Control registers of Timer1.
    |(1<<COM1B1);             // Wave Form Generation is Fast PWM 8 Bit,
    TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
    |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.
    
    OCR1A = 63;                       // Dutycycle of OC1A = 25%
    //OCR1B = 127;                      // Dutycycle of OC1B = 50%
    
    return;
    */
   // https://www.mikrocontroller.net/topic/83609
   
   
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 0x200;
   //OCR1A = Servoposition[2];
   //OCR1B = 0x0FFF;
   ICR1 = 0x6400;          // 0x6400: Pulsabstand 50 ms
   // http://www.ledstyles.de/index.php/Thread/18214-ATmega32U4-Schaltungen-PWM/
   DDRB |= (1<<DDB6)|(1<<DDB5);
   
   TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)|(1<<WGM11)|(1<<WGM12);
   
   TCCR1B |= (1<<WGM13)|(1<<CS11);
   //   TCCR1A=0xAA;
   //   TCCR1B=0x19;
   // TCCR1B |= (1<<CS10);
   
   
   
   //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}


#pragma mark INT0
ISR(INT0_vect) // Interrupt bei CS, falling edge
{
   //OSZIB_LO;
   inindex=0;
   //wl_status = wl_module_get_status();
   
   wl_spi_status |= (1<<WL_ISR_RECV);
   wl_isr_counter++;
   //OSZIB_HI;
}





#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

/*
 ISR (PCINT0_vect)
 {
 
 if(INTERRUPT_PIN & (1<< MASTER_EN)// LOW to HIGH pin change, Sub ON
 {
 //OSZI_C_LO;
 
 masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
 adc_counter ++; // loest adc aus
 
 }
 else // HIGH to LOW pin change, Sub ON
 {
 displaystatus |= (1<<UHR_UPDATE);
 //masterstatus &= ~(1<<SUB_TASK_BIT);
 }
 
 }
 */

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   /*
    // Atmega168
    
    #define TASTE1		19
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		186
    #define TASTE9		212
    #define TASTE_L	234
    #define TASTE0		248
    #define TASTE_R	255
    
    
    // Atmega328
    #define TASTE1		17
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		166
    #define TASTE9		214
    #define TASTE_L	234
    #define TASTE0		252
    #define TASTE_R	255
    */
   
   //lcd_gotoxy(0,0);
   //lcd_putint(Tastaturwert);
   /*
    if (Tastaturwert < TASTE1)
    return 1;
    if (Tastaturwert < TASTE2)
    return 2;
    if (Tastaturwert < TASTE3)
    return 3;
    if (Tastaturwert < TASTE4)
    return 4;
    if (Tastaturwert < TASTE5)
    return 5;
    if (Tastaturwert < TASTE6)
    return 6;
    if (Tastaturwert < TASTE7)
    return 7;
    if (Tastaturwert < TASTE8)
    return 8;
    if (Tastaturwert < TASTE9)
    return 9;
    
    if (Tastaturwert < TASTE_L)
    return 10;
    if (Tastaturwert < TASTE0)
    return 0;
    if (Tastaturwert <= TASTE_R)
    return 12;
    */
   
   
   
   // Tastatur2 // Reihenfolge anders
   /*
    #define WERT1    11    // 1 oben  Taste 2
    #define WERT3    34    // 2 links  Taste 4
    #define WERT4    64    // 3 unten  Taste 8
    #define WERT6    103   // 4 rechts  Taste 6
    #define WERT9    174   // 5 Mitte  Taste 5
    #define WERT2 	26    //  A links oben Taste  1
    #define WERT5    72       //    B links unten Taste 7
    #define WERT7    116      //   C rechts oben Taste 3
    #define WERT8    161      // D rechts unten Taste 9
    
    */
   
   if (Tastaturwert < WERT1)
      return 2;
   if (Tastaturwert < WERT2)
      return 1;
   if (Tastaturwert < WERT3)
      return 4;
   if (Tastaturwert < WERT4)
      return 8;
   if (Tastaturwert < WERT5)
      return 7;
   if (Tastaturwert < WERT6)
      return 6;
   if (Tastaturwert < WERT7)
      return 3;
   if (Tastaturwert < WERT8)
      return 9;
   if (Tastaturwert < WERT9)
      return 5;
   
   return -1;
   
   
   
}




/*
 //#define OSZIA_LO() OSZIPORT &= ~(1<<4)
 #define OSZI_A_HI() OSZIPORT |= (1<<4)
 #define OSZI_A_TOGG() OSZIPORT ^= (1<<4)
 */

//#define OSZIA_LO( bit) PORTD &= ~(1 << (PULSA))

#define setbit(port, bit) (port) |= (1 << (bit))
#define clearbit(port, bit) (port) &= ~(1 << (bit))



uint16_t writerand(uint16_t wert)
{
   uint16_t s1 = 50*(sin(M_PI * wert / 180.0))+100;
   s1 += 20*(cos((M_PI  * wert / 130.0)*7));
   s1 += 10*(sin((M_PI  * wert / 50.0)*3));
   
   return s1;
}

uint8_t writelin(uint16_t wert)
{
   uint8_t s1 = wert%48;
   
   return s1;
}
// pi 3.14159 26535


// MARK:  - main
int main (void)
{
   uint8_t payload[wl_module_PAYLOAD];
   payload[0] = 3;
   payload[1] = 0;
   payload[2] = 1;
   payload[3] = 4;
   payload[4] = 1;
   payload[5] = 5;
   payload[6] = 9;
   payload[7] = 2;
   payload[8] = 6;

   uint16_t tempwert = 444;
   int8_t r;
   
   uint16_t spi_count=0;
   
   // set for 16 MHz clock
   CPU_PRESCALE(CPU_8MHz); // Strom sparen
   
   timer0();
   sei();
   Master_Init();
   SPI_PORT_Init();
   SPI_Master_init();
//   SPI_ADC_init();
   
//   uint16_t    ADC_Array[ADC_BUFSIZE];
   
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
   //  volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
   //   volatile    uint8_t ram_indata=0;
   
   //  volatile    uint8_t eeprom_indata=0;
   //  volatile    uint8_t eeprom_testdata =0x00;
   //  volatile    uint8_t eeprom_testaddress =0x00;
   uint8_t usb_readcount =0x00;
   
   // ---------------------------------------------------
   // initialize the LCD
   // ---------------------------------------------------
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(100);
   lcd_cls();
   //lcd_puts("READY\0");
   lcd_puts("V: \0");
   lcd_puts(VERSION);
   lcd_clr_line(1);
   
   lcd_gotoxy(0,0);
   lcd_puts("Data_Logger\0");
   delay_ms(1000);
   //lcd_cls();
   lcd_clr_line(0);
   
   uint8_t TastaturCount=0;
   
   initADC(0);
   
   // ---------------------------------------------------
   // in attach verschoben, nur wenn USB eingesteckt
   
   // Initialize the USB, and then wait for the host to set configuration.
   // If the Teensy is powered without a PC connected to the USB port,
   // this will wait forever.
   
   usb_init();
   uint16_t usbwaitcounter = 0;
   //lcd_putc('a');
   // Ueberspringen wenn kein Host eingesteckt
   hoststatus &= ~(1<< TEENSYPRESENT);
   
   
   while ((usbwaitcounter < 0xFFFA))// && (!usb_configured()))
   {
      //lcd_gotoxy(0,3);
      //lcd_putint12(usbwaitcounter);
      _delay_ms(1);
      if (usb_configured())
      {
         hoststatus |= (1<< TEENSYPRESENT);
         //hoststatus = 1;
         //lcd_gotoxy(19,3);
         //lcd_putc('$');
         break;
         
      }
      
      usbwaitcounter++;
      if (usbwaitcounter > 0x100)
      {
         //lcd_gotoxy(19,3);
         //lcd_putc('!');
         break;
      }
   }
   
   /*
    while ((!usb_configured()) )//|| (usbwaitcounter < 0xFF))
    {
    usbwaitcounter++;
    _delay_ms(1);
    if (usbwaitcounter > 0xFFFE)
    {
    continue;
    }
    
    }
    */
   /* wait */ ;
   lcd_gotoxy(19,0);
   if (hoststatus & (1<<TEENSYPRESENT))
   {
      lcd_putc('$');
   }
   else
   {
      lcd_putc('X');
   }
   /*
    lcd_putint12(usbwaitcounter);
    lcd_putc(' ');
    lcd_puthex(hoststatus);
    lcd_putc('*');
    lcd_puthex(usb_configured());
    lcd_putc('*');
    */
   // Wait an extra second for the PC's operating system to load drivers
   // and do whatever it does to actually be ready for input
   _delay_ms(100);
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   /*
    Bit 0: 1 wenn wdt ausgel�st wurde
    */
   
   PWM = 0;
   
   char* versionstring[4] = {};
   strncpy((char*)versionstring, (VERSION+9), 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi((char*)versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   
   
   timer1();
   sei();
   
   
   
   //  masterstatus |= (1<<SUB_READ_EEPROM_BIT); // sub soll EE lesen
#pragma mark MMC init
   //lcd_gotoxy(0,0);
   //lcd_putc('a');
   
   DSTATUS initerr = mmc_disk_initialize();
   //lcd_putc('b');
   
   //lcd_gotoxy(0,0);
   //lcd_putc('*');
   //lcd_puthex(initerr);
   //lcd_putc('*');
   
   if (initerr)
   {
      lcd_gotoxy(0,0);
      lcd_puts("SD-");
      lcd_puthex(initerr);
      lcd_putc('*');
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("SD+");
      //lcd_puthex(initerr);
      //lcd_putc('*');
      
   }
   
   /*
    FRESULT mounterr = f_mount((void*)FatFs,"",1);
    lcd_gotoxy(16,0);
    lcd_puthex(mounterr);
    */
   
   DRESULT readerr=0;
   DRESULT writeerr=0;
   
   //mmcbuffer[0] = 'A';
   // mmcbuffer[1] = 'B';
   // mmcbuffer[2] = 'C';
   //mmcbuffer[0] = 0;
   
   readerr = mmc_disk_read ((void*)mmcbuffer,1,	1);
   //readerr = mmc_disk_write ((void*)mmcbuffer,1,	1);
   
   //   lcd_gotoxy(0,1);
   
   //   lcd_puthex(readerr);
   
   
   //   lcd_putc('*');
   if (readerr==0)
   {
      //      lcd_putc('+');
      uint16_t i=0;
      for (i=0;i<8;i++)
      {
         if (mmcbuffer[i])
         {
            //            lcd_puthex(mmcbuffer[i]);
         }
      }
      //      lcd_putc('+');
   }
   //  lcd_putc('*');
   
   
   
   
#pragma mark DS1820 init
   
   // DS1820 init-stuff begin
   // OW_OUT |= (1<<OW;
   uint8_t i=0;
   /*
   uint8_t nSensors=0;
   uint8_t err = ow_reset();
   //   lcd_gotoxy(18,0);
   //   lcd_puthex(err);
   
   
   gNsensors = search_sensors();
   
   delay_ms(100);
   if (gNsensors>0)
   {
      lcd_gotoxy(0,0);
      lcd_puts("Sn:\0");
      lcd_puthex(gNsensors);
      
      lcd_clr_line(1);
      start_temp_meas();
      
      
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }
   */
   // DS1820 init-stuff end
   
   
   
   // ---------------------------------------------------
   // Vorgaben fuer Homescreen
   // ---------------------------------------------------
   //substatus |= (1<<SETTINGS_READ);;
   // ---------------------------------------------------
   // Settings beim Start lesen
   // ---------------------------------------------------
   
   // timer1(); PORTB5,6
   
   
   
   volatile   uint8_t old_H=0;
   volatile   uint8_t old_L=0;
   uint8_t teensy_err =0;
   uint8_t testfix=0;
   
   uint16_t mmcwritecounter=0;
   
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   
   
   //  OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer OSZI
   //  OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   //DDRD |= (1<<4);
   //PORTD |= (1<<4);
   //OSZIA_HI;
   //OSZIPORT |= (1<<OSZI_PULS_A);
   
   
   
   
  if (TEST || (!(OSZIPORT & (1<<TEST_PIN)))) // Testdaten schreiben
  {
     uint16_t ii=0;
     uint16_t wert = 0;
     for (uint8_t block = 1;block < 5;block++)
     {
        
        while (ii<0xF0)
        {
           wert += 3 ;
           mmcbuffer[2*ii] = wert & 0x00FF;
           mmcbuffer[2*ii+1] = (wert & 0xFF00)>>8;
           
           ii += 1;
           if (wert > 133)
           {
              wert = 0;
           }
           
           
        }
        //lcd_gotoxy(0,3);
        //lcd_puthex(block);
        
        writeerr = mmc_disk_write ((void*)mmcbuffer,block ,1); // Block 0 ist system
        //lcd_putc('e');
        //lcd_puthex(writeerr);
        
        ii = 0;
     }
     
     // OSZIA_HI;
     
     lcd_gotoxy(6,3);
     lcd_puts("save ");
     lcd_puthex(writeerr);
     lcd_putc(' ');
     lcd_puthex(blockcounter);
  }
   
   // MARK: WL init
   
   
   uint8_t maincounter =0;
   //Array for Payload
   
   
   wl_module_init();
   
   
   _delay_ms(10);
   
 //  wl_module_tx_config(wl_module_TX_NR_0);
   
   
   uint8_t readstatus = wl_module_get_data((void*)&wl_data);
   
   uint16_t temperatur0=0;

   // MARK:  while
   sei();
   while (1)
   {
      //OSZI_B_LO;
      //Blinkanzeige
      loopcount0+=1;
      
      // ********
      
      if (wl_spi_status & (1<<WL_ISR_RECV)) // in ISR gesetzt, ETWAS LOS AUF WL
      {
         /* reset
          https://devzone.nordicsemi.com/question/5930/is-there-any-way-to-reset-nrf24l01/
          1)use power down mode (PWR_UP = 0)
          2)clear data ready flag and data sent flag in status register
          3)flush tx/rx buffer
          4)write status register as 0x0e;
          
          */
         
         OSZIA_LO;
         lcd_gotoxy(7,0);
         //lcd_puthex(int0counter);
         lcd_puts("is");
         lcd_puthex(wl_isr_counter);
         //OSZIA_HI;
         //lcd_gotoxy(18,1);
         
         wl_status = wl_module_get_status();
         
         /*
          lcd_putc(' ');
          lcd_gotoxy(10,2);
          lcd_puthex(wl_status);
          */
         delay_ms(1);
         
         pipenummer = wl_module_get_rx_pipe_from_status(wl_status);
         delay_ms(2);
         lcd_gotoxy(13,0);
         lcd_puthex(pipenummer);
         
         wl_spi_status &= ~(1<<WL_ISR_RECV);
         
         
         if (pipenummer == 7) // ungueltige pipenummer
         {
            wl_module_get_one_byte(FLUSH_TX);
            
            // pipe vorwaertsschalten
            if (loop_pipenummer < 3)
            {
               //               loop_pipenummer++;
               
            }
            else
            {
               wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // Data angekommen, not busy
               //              loop_pipenummer=1;
               
            }
            
         }
         else
         {
            //OSZIB_LO;
            
            //lcd_gotoxy(12,2);
            //lcd_puthex(wl_status);
            
            
            //loop_pipenummer = pipenummer;
            
            
            // MARK: WL Loop
            /*
             lcd_gotoxy(4,1);
             lcd_putc(' '); // pipenummer weg
             
             lcd_gotoxy(0,1);
             lcd_puts("  "); // RX weg
             lcd_gotoxy(14,1);
             lcd_puts("  "); // TX weg
             */
            if (wl_status & (1<<RX_DR)) // IRQ: Package has been received
            {
               //  OSZIA_LO; // 130ms mit Anzeige
               
               //              lcd_gotoxy(18,1);
               //               lcd_puts("  ");
               
               lcd_gotoxy(0,1);
               lcd_puts("RX");
               
               //             pipenummer = wl_module_get_rx_pipe();
               
               
               lcd_gotoxy(3,1);
               lcd_putc('p');
               lcd_putint1(pipenummer);
               
               // Kontrolle, ob payloadlength ok
               uint8_t rec = wl_module_get_rx_pw(pipenummer); //gets the RX payload width
               //lcd_gotoxy(0,3);
               if (!(rec==0x10))
               {
                  lcd_gotoxy(14,1);
                  lcd_putc('!');
                  lcd_puthex(rec);
               }
               
               // payload lesen
               uint8_t readstatus = wl_module_get_data((void*)&wl_data); // returns status
               delay_ms(2);
               
               //lcd_putc(' ');
               // task je nach pipenummer
               switch(pipenummer)
               {
                  case 1:
                  {
                     uint16_t temperatur1 = (wl_data[13]<<8);
                     temperatur1 |= wl_data[12];
                     temperatur0 = temperatur1;
                     
                     lcd_gotoxy(0,2);
                     lcd_putc('t');
                     lcd_putc('1');
                     lcd_putc(' ');
                     lcd_putint12(temperatur0);
                     
                     sendbuffer[ADC1LO]= wl_data[12];
                     sendbuffer[ADC1HI]= wl_data[13];
                     
                     sendbuffer[ADC0LO]= wl_data[10];
                     sendbuffer[ADC0HI]= wl_data[11];
                     lcd_gotoxy(18,2);
                     lcd_puthex(wl_data[0]);// maincounter von remote module
                     
                  }break;
                  case 2:
                  {
                     uint16_t temperatur1 = (wl_data[13]<<8);
                     temperatur1 |= wl_data[12];
                     temperatur0 = temperatur1;
                     
                     lcd_gotoxy(0,3);
                     lcd_putc('t');
                     lcd_putc('2');
                     lcd_putc(' ');
                     lcd_putint12(temperatur0);
                     
                     sendbuffer[ADC1LO]= wl_data[12];
                     sendbuffer[ADC1HI]= wl_data[13];
                     
                     sendbuffer[ADC0LO]= wl_data[10];
                     sendbuffer[ADC0HI]= wl_data[11];
                     lcd_gotoxy(18,3);
                     lcd_puthex(wl_data[0]);// maincounter von remote module
                     
                     
                  }break;
                  case 3:
                  {
                     
                  }break;
                  case 4:
                  {
                     
                  }break;
                     
                     
               }// switch pipenummer
               
               
               
               /*
                lcd_gotoxy(0,1);
                lcd_puthex(wl_data[13]);
                lcd_puthex(wl_data[12]);
                */
               /*
                uint16_t temperatur1 = (wl_data[13]<<8);
                temperatur1 |= wl_data[12];
                lcd_gotoxy(0,2);
                lcd_putc('t');
                lcd_putint(temperatur1);
                */
               
               wl_module_config_register(STATUS, (1<<RX_DR)); //Clear Interrupt Bit
               
               wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // Beim Senden gesetzt. Data angekommen, not busy
               
               PTX=0;
               
               
               wl_module_get_one_byte(FLUSH_TX);
               // pipe vorwaertsschalten
               
               delay_ms(20);
               
               if (loop_pipenummer < 3)
               {
                  loop_pipenummer++;
                  wl_spi_status |= (1<<WL_SEND_REQUEST);
                  
               }
               else
               {
                  //wl_spi_status &= ~(1<<WL_SEND_REQUEST); // Auftrag an wl erfuellt
                  wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // Data angekommen, not busy
                  loop_pipenummer=1;
                  
               }
               
               lcd_gotoxy(9,1);
               lcd_puts("r");
               //lcd_putint2(datapendcounter);
               //               lcd_puthex(readstatus);
               //               datapendcounter=0;
            }  // end if RX_DR
            
            
            
            if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
            {
               wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
               
               maincounter++;
               //OSZIA_LO;
               //               lcd_gotoxy(14,1);
               //               lcd_puts("   ");
               
               lcd_gotoxy(14,1);
               lcd_puts("TX");
               PTX=0;
               //OSZIA_HI;
            }
            
            
            if (wl_status & (1<<MAX_RT)) // IRQ: Package has not been sent, send again
            {
               lcd_gotoxy(0,1);
               lcd_puts("  ");
               
               lcd_gotoxy(18,1);
               lcd_puts("RT");
               //wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // reset, not busy
               
               wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
               
               wl_module_config_register(STATUS, (1<<MAX_RT)); // Clear Interrupt Bit
               
               //             if (wl_blockedcounter>2)
               {
                  lcd_gotoxy(18,0);
                  lcd_putc('x');
                  wl_spi_status &= ~(1<<WL_DATA_PENDENT);
                  
                  wl_blockedcounter = 0;
               }
               //              wl_module_CE_hi;
               //             _delay_us(15);
               //             wl_module_CE_lo;
            } // if RT
            else
            {
               //        lcd_gotoxy(18,1);
               //        lcd_puts("--");
            }
            
            //OSZIB_HI;
         } // if pipenummer <7
         OSZIA_HI;
         //    wl_spi_status = 0;
      } // end ISR abarbeiten (wl_spi_status & (1<<WL_ISR_RECV))
      
      
      
      // ********
      
      
      /* **** spi_buffer abfragen **************** */
      // MARK:  spi_rxdata
      
      /* **** end spi_buffer abfragen **************** */
      
      // MARK:  MMC write
      //   if ((mmcstatus & (1<<WRITENEXT)) )
      if (mmcstatus & (1<<WRITENEXT) ) //
      {
         //if (usbstatus & (1<<WRITEAUTO))
         if ((usbstatus == WRITE_MMC_TEST)&& (mmcwritecounter < 512)) // Test: SD beschreiben
         {
            //uint16_t tempdata = writerand(mmcwritecounter);
            uint16_t tempdata = writerand(mmcwritecounter);
            //   OSZIA_LO;
            //     sendbuffer[16] = (tempdata & 0x00FF);
            //     sendbuffer[17] = ((tempdata & 0xFF00)>>8);
            mmcbuffer[mmcwritecounter] = (tempdata & 0x00FF);
            mmcbuffer[mmcwritecounter+1] = ((tempdata & 0xFF00)>>8);
            lcd_gotoxy(0,3);
            lcd_putint12(mmcwritecounter & 0x1FF);
            lcd_putc(' ');
            lcd_putint12(tempdata);
            /*
             lcd_putc('l');
             lcd_puthex((tempdata & 0x00FF));
             lcd_putc('h');
             lcd_puthex(((tempdata & 0xFF00)>>8));
             lcd_putc(' ');
             */
         }
         else
         {
            usbstatus = DEFAULT;
         }
         
         
         //        writeerr = mmc_disk_write ((void*)mmcbuffer,1 + (mmcwritecounter & 0x200),1);
         // OSZIA_HI;
         
         //         lcd_gotoxy(14,3);
         //         lcd_puthex(writeerr);
         
         //lcd_putc('*');
         
         mmcstatus &= ~(1<<WRITENEXT);
         //         mmcwritecounter++;
         
      }
      //OSZI_A_TOGG;
      
      // **********************************************************
#pragma mark Mess-Intervall
      // **********************************************************
      uint16_t adcwert=0;
      float adcfloat=0;
      if (hoststatus & (1<<MESSUNG_OK)) // Intervall abgelaufen. In ISR gesetzt, Messungen vornehmen
      {
         /*
          // teensy
          //ADC
          ADC 0 lo
          ADC 0 hi
          ADC 1 lo
          ADC 1 hi
          
          //MC3204 12Bit
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          
          // Digi
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          
          
          // Satellit
          //ADC
          ADC 0 lo
          ADC 0 hi
          ADC 1 lo
          ADC 1 hi
          
          //MC3204 12Bit
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          
          // Digi
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          
          
          */
         hoststatus &= ~(1<<MESSUNG_OK);
         // ADC
         /*
          spiADC_init();
          cli();
          //uint16_t tempdata =MCP3208_spiRead(SingleEnd,1);
          uint8_t i=0;
          //uint16_t tempdata=0;
          for (i=0;i<4;i++)
          {
          //tempdata = MCP3204_spiRead(i);
          ADC_Array[i] = MCP3204_spiRead(i);;
          }
          //uint16_t tempdata = MCP3204_spiRead(1);
          lcd_gotoxy(0,2);
          for (i=0;i<4;i++)
          {
          lcd_putint12(ADC_Array[i]);
          lcd_putc(' ');
          }
          
          sei();
          */
         //lcd_clr_line(2);
         
         lcd_gotoxy(4,0);
         lcd_putint2(messungcounter&0x07);
         lcd_putc(' ');
         
         
         adcwert = adc_read(0);
         
         //_delay_ms(100);
         
         
         //lcd_gotoxy(8,0);
         
         //lcd_putint12(adcwert);
         //lcd_putc(' ');
         
         
         // adcwert *=10;
         // vor Korrektur
         
         // code fuer
         
         sendbuffer[0]= MESSUNG_DATA;
         
         sendbuffer[2] = 27;
         sendbuffer[5] = 28;//recvbuffer[STARTMINUTELO_BYTE];;
         sendbuffer[6] = 29;//recvbuffer[STARTMINUTEHI_BYTE];;
         
         sendbuffer[15] = 31; // Grenze zu DATA markieren
         
         // Data von ADC laden
         //    sendbuffer[ADC0LO]= (adcwert & 0x00FF);
         //    sendbuffer[ADC0HI]= ((adcwert & 0xFF00)>>8);
         
         
         
         //zaehler laden
         sendbuffer[DATACOUNT_LO] = (messungcounter & 0x00FF);
         sendbuffer[DATACOUNT_HI] = ((messungcounter & 0xFF00)>>8);
         
         /*
          usbstatus1
          #define SAVE_SD_BIT             0
          #define SAVE_SD_RUN_BIT         1
          #define SAVE_SD_STOP_BIT        2
          */
         
         if (usbstatus1 & (1<<SAVE_SD_RUN_BIT)) // Daten in mmcbuffer speichern, immer 2 bytes
         {
            //sendbuffer[2] = 23;
            //lcd_gotoxy(0,1);
            //lcd_putint12(saveSDposition); // 0 .. 255, pos im mmcbuffer, immer 2 byte pro messung
            
            mmcbuffer[2*saveSDposition] = (adcwert & 0x00FF);
            mmcbuffer[2*saveSDposition+1] = ((adcwert & 0xFF00)>>8);
            
            // nummer in SD sichern
            mmcbuffer[2*saveSDposition+2] = (messungcounter & 0x00FF);
            mmcbuffer[2*saveSDposition+3] = ((messungcounter & 0xFF00)>>8);
            
            /*
             adcwert += 4;
             mmcbuffer[2*saveSDposition+2] = (adcwert & 0x00FF);
             mmcbuffer[2*saveSDposition+3] = ((adcwert & 0xFF00)>>8);
             adcwert += 4;
             mmcbuffer[2*saveSDposition+4] = (adcwert & 0x00FF);
             mmcbuffer[2*saveSDposition+5] = ((adcwert & 0xFF00)>>8);
             adcwert += 4;
             mmcbuffer[2*saveSDposition+6] = (adcwert & 0x00FF);
             mmcbuffer[2*saveSDposition+7] = ((adcwert & 0xFF00)>>8);
             
             // restliche Positionen fuellen
             
             for (uint8_t pos = 0;pos<4;pos++)
             {
             mmcbuffer[2*saveSDposition + 8 + 2*pos] = pos;
             mmcbuffer[2*saveSDposition + 8 + 2*pos+1] = 0;
             }
             */
            for (uint8_t pos = 0;pos<6;pos++)
            {
               mmcbuffer[2*saveSDposition + 4 + 2*pos] = 0;
               mmcbuffer[2*saveSDposition + 4 + 2*pos+1] = 0;
            }
            
            //lcd_putc(' ');
            //lcd_putint12(mmcwritecounter % 0x200);
            //lcd_putc('b');
            //lcd_puthex(blockcounter);
            
            /*
             if (saveSDposition == 8) // erstes Data an 16
             {
             lcd_gotoxy(0,3);
             lcd_puthex(mmcbuffer[2*saveSDposition]);
             lcd_puthex(mmcbuffer[2*saveSDposition+1]);
             }
             */
            saveSDposition += 8;
            mmcwritecounter += 16; // Zaehlung write-Prozesse, immer 2 bytes pro messung
            
            
            
            if ((saveSDposition ) >= 0xF0) // Block voll, 2*255 Bytes = 512
            {
               writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
               // OSZIA_HI;
               
               lcd_gotoxy(8,1);
               lcd_puts("save ");
               lcd_puthex(writeerr);
               lcd_putc(' ');
               lcd_puthex(blockcounter);
               saveSDposition = 0;
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
               sendbuffer[2] = 37;
               blockcounter++;
            }
         }
         else if (usbstatus1 & (1<<SAVE_SD_STOP_BIT)) // Schreiben beenden, letzten Block noch schreiben
         {
            usbstatus1 &= ~(1<<SAVE_SD_STOP_BIT);
            usbstatus1 &= ~(1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben beenden
            // Daten sichern
            // ** DIFF writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
            writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
            // OSZIA_HI;
            
            lcd_gotoxy(8,1);
            lcd_puts("resc ");
            lcd_puthex(writeerr);
            lcd_putc(' ');
            lcd_puthex(blockcounter);
            saveSDposition = 0;
            sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
            sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
            
         } //end if usbstatus1 & (1<<SAVE_SD_STOP_BIT)
         else if (! usb_configured()) //b kein USB
         {
            lcd_gotoxy(8,1);
            lcd_puts("esc  ");
            
            // usbstatus1 &= ~(1<<SAVE_SD_RUN_BIT);   //  Schreiben so oder so beenden
         }
         
         //adcwert /= 2;
         //lcd_gotoxy(0,1);
         //lcd_putint12(adcwert/4); // *256/1024
         // lcd_putc(' ');
         //OSZIA_LO;
         
         
         adcfloat = adcfloat *2490/1024; // kalibrierung VREF, 1V > 0.999, Faktor 10, 45 us
         
         adcwert = (((uint16_t)adcfloat)&0xFFFF);
         //OSZIA_HI;
         
         uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         //         lcd_gotoxy(10,0);
         //lcd_putint(adcwert & 0x00FF);
         //lcd_putc(' ');
         //lcd_putint2((adcwert & 0xFF00)>>8);
         //        lcd_putc('t');
         //         lcd_putint12(adcwert);
         //messungcounter++;
         messungcounter ++; // 8 Werte geschrieben, naechste zeile
         
         wl_spi_status |= (1<<WL_SEND_REQUEST); // Auftrag an wl, Daten lesen
         loop_pipenummer = 1;
         temperatur0=0;
         // Messreihe auf wl starten
         //loop_pipenummer=1;
         
         if (akt_pipenummer < 3)
         {
            akt_pipenummer++;
            
         }
         else
         {
            //wl_spi_status &= ~(1<<WL_SEND_REQUEST); // Auftrag an wl erfuellt
            akt_pipenummer=1;
            
         }
         
      } // end if (hoststatus & (1<<MESSUNG_OK))
      
      
      
      
      // Messung abgeschlossen, wl_devices aufrufen, Daten lesen
      
      
      if (wl_spi_status & (1<<WL_SEND_REQUEST))
      {
         lcd_gotoxy(9,1);
         lcd_puts(" "); // senden markieren, wird in WL_ISR_RECV-Routine mit r ueberschrieben
         
         // WL write start
         
         // ************** SEND **********************************
#pragma mark WL send
         // **********************************************************
         
         //      continue;
         
         // paket start
         /*
          if ((wl_spi_status & (1<<WL_DATA_PENDENT))) //  netz busy oder remote nicht da
          {
          
          wl_blockedcounter++;
          //if (wl_blockedcounter>1)
          {
          lcd_gotoxy(18,0);
          lcd_putc('z');
          wl_module_get_one_byte(FLUSH_TX);
          wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
          wl_module_config_register(STATUS, (1<<RX_DR)); //Clear Interrupt Bit
          wl_spi_status &= ~(1<<WL_DATA_PENDENT);
          wl_spi_status &= ~(1<<WL_SEND_REQUEST);
          wl_blockedcounter = 0;
          }
          }
          */
         //         else
         
         lcd_gotoxy(14,1);
         lcd_puts("   ");
         
         
         lcd_gotoxy(6,1);
         lcd_putc('l');
         lcd_putint1(loop_pipenummer);
         //         lcd_gotoxy(6,2);
         //        lcd_putc('l');
         //         lcd_putint1(akt_pipenummer);
         
         wl_blockedcounter = 0;
         
         wl_module_get_one_byte(FLUSH_TX);
         wl_module_get_one_byte(FLUSH_RX);
         
         
         
         delay_ms(30);
         //wl_module_tx_config(2);
         
         // ***** PIPE *********************************************
         
         if (OSZIPORTPIN & (1<<TEST_PIN))
         {
            wl_module_tx_config(2);
            
         }
         else
         {
            
            wl_module_tx_config(loop_pipenummer);
         }
         
         
         
         OSZIB_LO;
         delay_ms(10); // etwas warten
         // WL
         
         payload[9] = maincounter;
         //       payload[10] = sendbuffer[ADC0LO];
         //       payload[11] = sendbuffer[ADC0HI];
         payload[10] = adcwert & 0x00FF;
         payload[11] = (adcwert & 0xFF00)>>8;
         
         
         // ***** SENDEN *****************************************************
         
         wl_module_send(payload,wl_module_PAYLOAD);
         
         wl_sendcounter++;
         
         datapendcounter=0;
         
         wl_spi_status &= ~(1<<WL_SEND_REQUEST); // Auftrag an wl erfuellt
         //            wl_spi_status |= (1<<WL_DATA_PENDENT); // warten auf antwort vom remote
         
         //OSZIA_LO; // 30 ms bis lesen
         
         // **********************************************************
         
         //wl_spi_status |= (1<<WL_DATA_PENDENT);    // busy
         
         
         lcd_gotoxy(18,0);
         lcd_putc(' ');
         
         
         lcd_gotoxy(9,1);
         lcd_puts("s"); // senden markieren, wird in WL_ISR_RECV-Routine mit r ueberschrieben
         
         delay_ms(20); // etwas warten, wichtig, sonst wird rt nicht immer erkannt
         wl_status = wl_module_get_status();
         //     lcd_gotoxy(16,2);
         //    lcd_puthex(wl_status);
         //delay_ms(20);
         
         if (wl_status & (1<<MAX_RT))
         {
            wl_module_config_register(STATUS, (1<<MAX_RT));	// Clear Interrupt Bits
            //           lcd_gotoxy(14,1);
            //           lcd_puts("   ");
            //wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            //wl_module_config_register(STATUS, (1<<RX_DR)); //Clear Interrupt Bit
            //           wl_module_get_one_byte(FLUSH_TX);
            
            lcd_gotoxy(14,1);
            lcd_puts("rt");
            
            delay_ms(10);
            
            
            if (loop_pipenummer < 3)
            {
               loop_pipenummer++;
               wl_spi_status |= (1<<WL_SEND_REQUEST);
               
            }
            else
            {
               loop_pipenummer=1;
               //wl_spi_status |= (1<<WL_SEND_REQUEST);
            }
            
            
         }
         
         /*
          // verhinderte data lesen
          if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
          {
          //OSZIA_LO;
          lcd_gotoxy(14,1);
          lcd_puts("   ");
          
          lcd_gotoxy(14,1);
          lcd_puts("*tx");
          wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
          PTX=0;
          //OSZIA_HI;
          }
          */
         
         
         wl_module_rx_config();
         
         
      } // if (wl_spi_status & (1<<WL_SEND_REQUEST))
      
      
      
      // if hoststatus & (1<<MESSUNG_OK)
      
      if (hoststatus & (1<<DOWNLOAD_OK))
      {
         
      }// if (hoststatus & (1<<DOWNLOAD_OK))
      
      
      if (loopcount0==0x8FFF)
      {
         
         loopcount0=0;
         loopcount1+=1;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //      continue;
         lcd_gotoxy(16,0);
         lcd_puthex(wl_sendcounter);
         //         lcd_putc(' ');
         //         lcd_putc('p');
         //        lcd_puthex(pipenummer);
         //         lcd_putc(' ');
         
         
         if (usbstatus & (1<<WRITEAUTO))
         {
            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         }
         //lcd_puthex(usberfolg);
         
         if ((loopcount1%2 == 0) && (usbstatus & (1<<WRITEAUTO)))
         {
#pragma mark ADC
            /*
             _delay_ms(10);
             uint16_t adcwert = adc_read(0);
             _delay_ms(100);
             
             lcd_gotoxy(10,0);
             lcd_putint(adcwert & 0x00FF);
             lcd_putc(' ');
             lcd_putint2((adcwert & 0xFF00)>>8);
             
             // adcwert *=10;
             // vor Korrektur
             sendbuffer[ADC0LO]= (adcwert & 0x00FF);
             sendbuffer[ADC0HI]= ((adcwert & 0xFF00)>>8);
             
             //adcwert /= 2;
             lcd_gotoxy(0,1);
             //       lcd_putint12(adcwert/4); // *256/1024
             //       lcd_putc(' ');
             //OSZIA_LO;
             double adcfloat = adcwert;
             adcfloat = adcfloat *2490/1024; // kalibrierung VREF, 1V > 0.999, Faktor 10, 45 us
             
             adcwert = (((uint16_t)adcfloat)&0xFFFF);
             //OSZIA_HI;
             lcd_putint12(adcwert);
             
             */
            //           sendbuffer[ADC0LO+2]= (adcwert & 0x00FF);
            //           sendbuffer[ADC0HI+2]= ((adcwert & 0xFF00)>>8);
            /*
             lcd_putc(' ');
             lcd_gotoxy(10,1);
             lcd_putint(adcwert & 0x00FF);
             lcd_putc(' ');
             lcd_putint2((adcwert & 0xFF00)>>8);
             lcd_putc('*');
             lcd_puthex(usb_configured());
             */
         }
         
         if(loopcount1%32 == 0)
         {
            
#pragma mark Sensors
            // Temperatur messen mit DS18S20
            /*
             if (gNsensors) // Sensor eingesteckt
             {
             start_temp_meas();
             delay_ms(800);
             read_temp_meas();
             uint8_t line=0;
             //Sensor 1
             lcd_gotoxy(14,line);
             lcd_puts("T:     \0");
             if (gTempdata[0]/10>=100)
             {
             lcd_gotoxy(13,line);
             lcd_putint((gTempdata[0]/10));
             }
             else
             {
             lcd_gotoxy(12,line);
             lcd_putint2((gTempdata[0]/10));
             }
             
             lcd_putc('.');
             lcd_putint1(gTempdata[0]%10);
             
             sendbuffer[DSLO]=((gTempdata[0])& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
             sendbuffer[DSHI]=((gTempdata[0]& 0xFF00)>>8);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
             //lcd_gotoxy(10,0);
             //lcd_putint(sendbuffer[DSLO]);
             //lcd_putc(' ');
             //lcd_putint(sendbuffer[DSHI]);
             // Halbgrad addieren
             if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
             {
             // txbuffer[INNEN] +=1;
             }
             }
             */
         } //
         
         
         
         // MARK:  USB send
         // neue Daten abschicken
         //         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         //OSZI_C_LO;
         
         //   uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         //OSZI_C_HI;
         //OSZI_A_HI;
         
         // OSZI_B_HI;
      } // if loopcount0
      
      /**	ADC	***********************/
      
      /**	END ADC	***********************/
      
      /**	Begin USB-routinen	***********************/
      // MARK USB read
      // Start USB
      //OSZI_D_LO;
      r=0;
      recvbuffer[0] = 0;
      //     if (usbstatus & (1<<READAUTO))
      {
         
         r = usb_rawhid_recv((void*)recvbuffer, 0); // 5us
      }
      // MARK: USB_READ
      
      if (r > 0)
      {
         //OSZI_A_LO ;
         cli();
         usb_readcount++;
         uint8_t code = recvbuffer[0];
         //lcd_gotoxy(18,2);
         //lcd_puthex(code);
         
         if (!(usbstatus == code))
         {
            usbstatus = code;
            lcd_clr_line(1);
            lcd_gotoxy(18,1);
            lcd_puthex(code);
         }
         
         switch (code)
         {
               
            case DEFAULT: // cont read
            {
               //lcd_clr_line(2);
               sendbuffer[0] = DEFAULT;
               sendbuffer[3] = 0;
               mmcwritecounter = 0;
               sendbuffer[0] = DEFAULT;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               usbstatus1 = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(usbstatus1); // code
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
               // PWM fuer Channel A
               OCR1A = (recvbuffer[10] | (recvbuffer[11]<<8));
               // lcd_putc('*');
               // lcd_gotoxy(12,0);
               // lcd_putint12(OCR1A);
               
            }break;
               
               // MARK: LOGGER_START
               
            case LOGGER_START:
            {
               hoststatus &= ~(1<<MESSUNG_OK);
               hoststatus |= (1<<DOWNLOAD_OK); // Download von SD, Messungen unterbrechen
               
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('l');
               lcd_putc(':');
               lcd_puthex(code);
               
               sendbuffer[0] = LOGGER_START;
               
               
               //code = LOGGER_START; // read block starten
               //lcd_putc('c');
               //lcd_puthex(code); // packetcount
               
               // Block lesen
               /*
                lcd_putc('l');
                lcd_puthex(recvbuffer[BLOCKOFFSETLO_BYTE]); // startblock lo
                lcd_putc('h');
                lcd_puthex(recvbuffer[BLOCKOFFSETHI_BYTE]); // startblock hi
                lcd_putc('*');
                lcd_puthex(recvbuffer[PACKETCOUNT_BYTE]); // packetcount
                */
               // old
               
               startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               uint8_t paketindex = 0;
               
               // old
               //packetcount = recvbuffer[3] ;// laufender Index Paket, beim Start 0
               
               packetcount = recvbuffer[PACKETCOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               //               packetcount=0;
               // lcd_gotoxy(12,1);
               // lcd_puts(">mmc");
               //lcd_gotoxy(0,3);
               //lcd_puthex(startblock);
               
               blockanzahl = recvbuffer[BLOCK_ANZAHL_BYTE] ;// laufender Index Paket, beim Start 0
               //lcd_gotoxy(3,3);
               //lcd_puthex(blockanzahl);
               
               
               // Beim Start Block aus SD lesen
               readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock,1);
               
               
               if (readerr == 0)
               {
                  lcd_gotoxy(15,1);
                  lcd_puts(">OK ");
                  sendbuffer[DATA_START_BYTE -1] = 111;
                  
                  // Header uebertragen, erste HEADER_SIZE bytes im Block
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < DATA_START_BYTE-2;headerindex++)
                  {
                     // sendbuffer[headerindex+2] = mmcbuffer[headerindex];
                     //sendbuffer[headerindex+2+1] = headerindex;
                  }
               } // if readerr==0
               else
               {
                  lcd_gotoxy(14,1);
                  lcd_puts(">err");
               }
               
               // old
               //sendbuffer[3] = 0;
               
               sendbuffer[PACKETCOUNT_BYTE] = 0; //
               sendbuffer[1] = readerr;
               sendbuffer[16] = 57;
               sendbuffer[17] = startblock;
               sendbuffer[18] = 58;
               sendbuffer[19] = blockanzahl;
               sendbuffer[20] = 59;
               
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               //lcd_gotoxy(18,1);
               //lcd_puthex(usberfolg);
               
            }break;
               
               
               // MARK: LOGGER_CONT
            case LOGGER_CONT:
            {
               // lcd_clr_line(1);
               // lcd_gotoxy(0,1);
               // lcd_putc('c');
               // lcd_putc(':');
               sendbuffer[0] = LOGGER_CONT;
               usbstatus1 = recvbuffer[1];
               //code = LOGGER_CONT;
               //lcd_putc('c');
               // lcd_puthex(code); // code
               
               // Block lesen
               //lcd_puthex(recvbuffer[STARTBLOC]); // startblock lo
               //lcd_puthex(recvbuffer[2]); // startblock hi
               //lcd_putc(' ');
               //lcd_putc(' ');
               //lcd_puthex(recvbuffer[PACKETCOUNT_BYTE]); // packetcount
               
               packetcount = recvbuffer[PACKETCOUNT_BYTE];
               
               uint8_t paketindex = 0;
               
               if (downloadblocknummer == 0 && packetcount == 0)
               {
                  for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 48 bytes fuer sendbuffer
                  {
                     lcd_gotoxy(18,1);
                     lcd_putc(65);
                     // Header ist schon geladen, Breite ist HEADER_SIZE (16)
                     //sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[HEADER_SIZE + (packetcount*PACKET_SIZE)+paketindex];
                     sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[ (packetcount*PACKET_SIZE)+paketindex];
                  }
               }
               else
               {
                  lcd_gotoxy(19,1);
                  lcd_putc(97);
                  for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 48 bytes fuer sendbuffer
                  {
                     // Breite ist HEADER_SIZE (16)
                     sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                  }
               }
               
               sendbuffer[PACKETCOUNT_BYTE] = ++packetcount; //
               
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
               
            }break;
               
               // MARK: LOGGER_NEXT
            case LOGGER_NEXT: // Block an startblock + downloadblocknummer lesen
            {
               //lcd_clr_line(1);
               //lcd_gotoxy(0,1);
               //lcd_putc('n');
               //lcd_putc(':');
               //lcd_puthex(code); // code
               sendbuffer[0] = LOGGER_NEXT;
               
               //startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               uint8_t paketindex = 0;
               //lcd_gotoxy(0,3);
               //lcd_puthex(startblock);
               
               downloadblocknummer  = recvbuffer[DOWNLOADBLOCKNUMMER_BYTE] ;// nummer des next blocks
               
               //lcd_gotoxy(6,1);
               //lcd_puthex(downloadblocknummer);
               
               
               // Beim Start Block aus SD lesen
               readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock + downloadblocknummer,1);
               if (readerr == 0)
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('+');
               } // if readerr==0
               else
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('-');
               }
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
            }break; // LOGGER_NEXT
               
               // MARK: LOGGER_STOP
            case LOGGER_STOP: // 0xAF
            {
               hoststatus &= ~(1<<DOWNLOAD_OK); // Download von SD beendet, Messungen fortsetzen
               //lcd_clr_line(1);
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(code); // code
               
               mmcwritecounter = 0;
               usbstatus = 0;
               sendbuffer[0] = LOGGER_STOP;
               
               sendbuffer[PACKETCOUNT_BYTE] = 0; // packetcount
               
            }break;
               
               // MARK: WRITE_MMC_TEST
            case WRITE_MMC_TEST:
            {
               //lcd_clr_line(2);
               //lcd_gotoxy(0,2);
               //lcd_putc('t');
               //lcd_putc(':');
               sendbuffer[0] = WRITE_MMC_TEST;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               usbstatus1 = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(usbstatus1); // code
               
            }break;
               
               // MARK: USB_STOP
            case USB_STOP: // Host ist ausgeschaltet
            {
               //hoststatus &= ~(1<<MESSUNG_OK);
               //hoststatus &= ~(1<<DOWNLOAD_OK);
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('h');
               lcd_putc(':');
               lcd_puthex(code); // code
               //sendbuffer[0] = USB_STOP;
               
            }break;
               
               // MARK: LOGGER_SETTING
            case LOGGER_SETTING:
            {
               sendbuffer[0] = LOGGER_SETTING;
               usbstatus1 = recvbuffer[1];
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(code); // code
               lcd_putc(' ');
               lcd_putint12(intervall);
               lcd_gotoxy(6,3);
               lcd_puts("set  ");
            }break;
               
               
               // MARK: MESSUNG_START
            case MESSUNG_START:
            {
               cli();
               messungcounter = 0;
               sendbuffer[0] = MESSUNG_START;
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('m');
               lcd_putc(':');
               lcd_puthex(code); // code
               usbstatus = code;
               usbstatus1 = recvbuffer[1];
               
               mmcwritecounter = 0;
               saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
               
               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               startminute  = recvbuffer[STARTMINUTELO_BYTE] | (recvbuffer[STARTMINUTEHI_BYTE]<<8);
               
               lcd_putc(' ');
               lcd_puthex(blockcounter);
               
               lcd_putc(' ');
               lcd_puthex(usbstatus1);
               lcd_putc(' ');
               lcd_puthex(saveSDposition);
               
               lcd_gotoxy(8,1);
               lcd_puts("start ");
               sendbuffer[1] = usbstatus1;
               sendbuffer[2] = 17;
               sendbuffer[5] = 18;//recvbuffer[STARTMINUTELO_BYTE];;
               sendbuffer[6] = 19;//recvbuffer[STARTMINUTEHI_BYTE];;
               sendbuffer[15] = 21;
               
               saveSDposition = 0; // erste Messung sind header
               sei();
               
               // _delay_ms(1000);
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
            }break;
               
               // MARK: MESSUNG_STOP
            case MESSUNG_STOP:
            {
               sendbuffer[0] = MESSUNG_STOP;
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('h');
               lcd_putc(':');
               lcd_puthex(code); // code
               lcd_putc('*');
               usbstatus = code;
               usbstatus1 = recvbuffer[1];
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0x00FF)>>8;
               
               lcd_gotoxy(19,1);
               lcd_putc('+');
               lcd_gotoxy(8,1);
               lcd_puts("m stop");
               
               
               
            }break;
               
            default:
            {
               break;
            }
               
         }
         
         code=0;
         sei();
         
         //OSZI_D_HI;
         //OSZI_A_HI ;
      } // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
      
      
      
      //OSZI_B_HI;
      
   }//while
   //free (sendbuffer);
   
   // return 0;
}
