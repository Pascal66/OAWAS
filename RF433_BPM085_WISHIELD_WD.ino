
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define FASTADC 1
//07012013
//#define ISR 
//#define DEBUG

#include <avr/sleep.h>
//#include <avr/wdt.h>

#include <WiServer.h>
extern "C" {
#include <g2100.h>
} 
extern "C" {
#include <spi.h>
} 
#define WIRELESS_MODE_INFRA  1
#define WIRELESS_MODE_ADHOC	2
// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = {
  192,168,1,65};	// IP address of WiShield
unsigned char gateway_ip[] = {
  192,168,1,1};	// router or gateway IP address
unsigned char subnet_mask[] = {
  255,255,255,0};	// subnet mask for the local network
const prog_char ssid[] PROGMEM = {
  "Freebox-550CD1"};		// max 32 bytes
unsigned char security_type = 5;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = {
  "pulle@.-astantes!-speraveris-enuntiand#@"};
// Pre-calc
// 31 b1 e5 97 9d d6 72 dc 6f 55 a1 ca 9a 5d 33 5b 04 ef 0d b4 56 2e 99 3d 3e 1a ee a2 21 8f 82 6f
// 4, 5 - WPA/WPA2 Precalc
// The 32 byte precalculate WPA/WPA2 key. This can be calculated in advance to save boot time
// http://jorisvr.nl/wpapsk.html

//PP Voir g2100.c
const prog_char security_data[] PROGMEM = {
  0x31, 0xB1, 0xE5, 0x97, 0x9D, 0xD6, 0x72, 0xDC, 0x6F, 0x55, 0xA1, 0xCA, 0x9A, 0x5D, 0x33, 0x5B, 
  0x04, 0xEF, 0x0D, 0xB4, 0x56, 0x2E, 0x99, 0x3D, 0x3E, 0x1A, 0xEE, 0xA2, 0x21, 0x8F, 0x82, 0x6F,
};  

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = { 
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,	// Key 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Key 1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Key 2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	// Key 3
};

// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = WIRELESS_MODE_INFRA;

unsigned char ssid_len;
unsigned char security_passphrase_len;
//  The address of the server you want to connect to:
byte server[] = {
  192,168,1,50}; 
// A request that 
GETrequest sendWEE(server, 80, "Pascal.org", "");
// End of wireless configuration parameters ----------------------------------------

char path[] = "/Npds/myWEE.php?TEMP=%s&HUM=%s&Vcc=%ld&A7=%ld&BARO=%s&RSSI=%s&RF=%s";

// Function that prints data from the server
void printData(char* data, int len) {
  // Print the data returned by the server
  // Note that the data is not null-terminated, may be broken up into smaller packets, and 
  // includes the HTTP header. 
  while (len-- > 0) { 
    Serial.print(*(data++));
  }   
}

int chksum_2;
long Vcc=3293;
char rf_data[128]; 

volatile int SHORT=550, X=1900, Y=4450, Z=9450;
static float chklast = 0;

void sendstatus(void)
{ 
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());

  //  Vcc += readVcc(); 
  //  Vcc /=2;
  Vcc = readVcc(); 

  long AD7 = analogRead(A7); // Lit PC1 ?? (3eme a gauche en partant du bas

  U16 rssi = zg_get_rssi();
  float strengh = ( rssi / 2.0); // 100*rssi/200

    //  float h = dht.readHumidity();   
  //  float t = (dht.readTemperature()/1.0/* + temperature) / 2))*/);
  float h=0.00;
  float t=temperature;

  if (h+t == chklast) return;
  chklast=h+t;

  char tBuffer[16]; 
  char hBuffer[16];
  char bBuffer[16];
  char rBuffer[16]; 
  dtostrf(t,8,2,tBuffer); 
  dtostrf(h,8,2,hBuffer);
  dtostrf(pressure,8,2,bBuffer);
  dtostrf(strengh,8,2,rBuffer); 
  replaceAll(tBuffer, " ","");
  replaceAll(hBuffer, " ","");
  replaceAll(bBuffer, " ","");
  replaceAll(rBuffer, " ","");
  replaceAll(rf_data, " ","");

  char newURL[255];
  sprintf(newURL, path, tBuffer, hBuffer, Vcc, AD7, bBuffer, rBuffer,rf_data);

  sendWEE.setURL(newURL);
  sendWEE.submit();  

#ifdef DEBUG
  Serial.println( newURL ); 
  Serial.print( chksum_2 ); 
  Serial.print(char(9));

  Serial.println(rf_data); 

  Serial.print(" SHORT ");
  Serial.print(SHORT);
  Serial.print(" X ");
  Serial.print(X);
  Serial.print(" Y ");
  Serial.print(Y);
  Serial.print(" Z ");
  Serial.println(Z); 
  delay(50); 
#endif 
}

void replaceAll(char *buf_,const char *find_,const char *replace_) {
  char *pos;
  int replen,findlen;

  findlen=strlen(find_);
  replen=strlen(replace_);

  while((pos=strstr(buf_,find_))) {
    strncpy(pos,replace_,replen);
    strcpy(pos+replen,pos+findlen);
  }
} 
long readVcc() {
  long result;

  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle

  // Set ADEN in ADCSRA to enable the ADC.
  ADCSRA |= _BV(ADEN);

  ADCSRA |= _BV(ADSC); // Convert take 25 clock puis 13 clock
  while (bit_is_set(ADCSRA,ADSC)) ;
  result = 1126400L / ADCW ; 
  return result;
}

#include <Wire.h>
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
// oversampling setting
// 0 = ultra low power
// 1 = standard
// 2 = high
// 3 = ultra high resolution
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

// This variable is made volatile because it is changed inside an interrupt function
volatile int ISRCount = 0; 
volatile boolean f_timer=true;  //used to cound number of sleep cycles
//07012013 on va essayer de dormir >21s au lieu de >7s sans deconnection
int sleep_count = 21; // Keep track of how many sleep (x+1) * 0.5s
// Mieux vaut dormir souvent que de dormir longtemps

/* Receiver pinmap */
const byte RF_RX_SIG = 3; // don't forget the wire between this pin and D2 !

/* -------------------------------------------------------- */
/* ----                 Sniffer API                    ---- */
/* -------------------------------------------------------- */

/* ISR routines variables */
unsigned long current_time, relative_time, last_time = 0;

int packet_counter, __A_counter;

boolean last = false;
volatile boolean trame_triggered;

/** 
 * ISR RF frame decoding routine
 */
//07012013 
//#ifdef ISR
//ISR(INT1_vect, ISR_NAKED) { 
//#else
void isr_decoding_routine(void) {
//#endif
  current_time = micros();
  relative_time = current_time - last_time;
  last_time = current_time;

  trame_triggered = false; // Serial.println(relative_time);

  if((relative_time>=250 && relative_time<=700) && last) {
    // Nous avons un front bas très court et la base de ABC      
    last = false; 
    SHORT+=relative_time; 
    SHORT/=2;    
    return ;
  } 
  if((relative_time>=1450 && relative_time<=2500) && !last) {
    // Nous avons un front haut court et la base de BC
    __A_counter++;
    last = 1; 
    X+=relative_time; 
    X/=2;
    return ;
  } 
  if((relative_time>=4000 && relative_time<=6000) && !last) {
    // Nous avons le front des données,
    // Le nombre de A précédants donne ces données

    rf_data[packet_counter++] = '0' + __A_counter;
    rf_data[packet_counter] ='\0';

    chksum_2 += __A_counter; 
    __A_counter=1;
    last = 1; 
    Y+=relative_time; 
    Y/=2;
    return ;
  } 
  if((relative_time>=8000 && relative_time<=9900) && !last) {
    // Nous avons le front des trames (long) = 2A suivi de C
    // Il y a 9 trames 7 completes, un départ, une finale
    if (__A_counter>=1 && chksum_2==27) { 
#ifdef DEBUG     
      //Serial.println("TRAME ISR");
#endif

      trame_triggered = true; 
    } 

    __A_counter=1;
    last = 1; 
    Z+=relative_time; 
    Z/=2;
    return ;
  } 
  else 
  { 
    rf_data[0]='\0';
    packet_counter=0;
    __A_counter=1;
    last = 1;
    chksum_2 = 0;
    return ;
  } 

}

/* -------------------------------------------------------- */
/* ----            Sniffing program            ---- */
/* -------------------------------------------------------- */

void setup() {
  //07012013
  delay( 50 );   // allow some time (50 ms) after powerup and sketch start, 
  // for the Wishield Reset IC to release and come out of reset.

  //pinMode(4, OUTPUT);
  //PORTD &= ~_BV(4);//digitalWrite(4, LOW);

  //07012013 
#ifndef sleep_bod_disable
#define sleep_bod_disable() \
do { \
unsigned char tempreg; \
__asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
"ori %[tempreg], %[bods_bodse]" "\n\t" \
"out %[mcucr], %[tempreg]" "\n\t" \
"andi %[tempreg], %[not_bodse]" "\n\t" \
"out %[mcucr], %[tempreg]" \
: [tempreg] "=&d" (tempreg) \
: [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
[bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
[not_bodse] "i" (~_BV(BODSE))); \
} while (0)
#endif

#if FASTADC
  // Define various ADC prescaler
  const unsigned char PS_16 = (1 << ADPS2);
  const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
  const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
  const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_16;    // set our own prescaler to 64 

#endif

#ifdef DEBUG
  // Serial port initialization (for output) 
  Serial.begin(115200);
  Serial.println("PP 433 sniffer v10_NEWISR");
  // Initialize WiServer and have it use the sendMyPage function to serve pages
  //WiServer.init(sendMyPage);
  WiServer.init(NULL);
  Serial.println("sendMyPage NULL OK");
  WiServer.enableVerboseMode(true);
  Serial.println("Server Ready");
  // Have the processData function called when data is returned by the server
  sendWEE.setReturnFunc(printData);
  Serial.println("sendWEE.setReturnFunc(printData) OK");

#else
  WiServer.init(NULL);
#endif

  Wire.begin();
  bmp085Calibration();

  watchdogOn(); // Turn on the watch dog timer.
  pinMode(RF_RX_SIG, INPUT);
 
//07012013 
//#ifdef ISR
//EICRA &= ~(_BV(ISC10) | _BV (ISC11));  // clear existing flags
//EICRA |= _BV (ISC10);    // set wanted flags (change level interrupt)
//EIMSK |= _BV (INT1);     // enable it

//EICRA (External Interrupt Control Register A) would be set according to this table from the Atmega328 datasheet (page 71). 
//That defines the exact type of interrupt you want:

//0: The low level of INT0 generates an interrupt request (LOW interrupt).
//1: Any logical change on INT0 generates an interrupt request (CHANGE interrupt).
//2: The falling edge of INT0 generates an interrupt request (FALLING interrupt).
//3: The rising edge of INT0 generates an interrupt request (RISING interrupt).

//EIMSK (External Interrupt Mask Register) actually enables the interrupt.

//EIFR = _BV (INTF1);  // clear flag for interrupt 1
//#else
  /* low-level ISR decoding routine setup */
  attachInterrupt(1, isr_decoding_routine, CHANGE);
//#endif  

}

void loop() {
  // static char old_data[128];
  // WiServer.server_task(); 
  //  delay(500); // Wait for 9 trames

  while(!trame_triggered) { 
    WiServer.server_task();  
    ISRCount=sleep_count-1; // Juste 0.5s

    //sleepNow();
    dodo('N');
    //delay(10);
  }
  EIMSK&=~(1<<INT1);  // mask Interrupt 1

#ifdef DEBUG
    Serial.println("TRAME LOOP"); 
#endif   


  rf_data[packet_counter] ='\0';

  sendstatus(); 
  delay(500);
  rf_data[0] ='\0'; 
  // WiServer.server_task();

  f_timer=false;
  ISRCount = 0;
  packet_counter=0;
  trame_triggered=false;

  dodo('K'); 

  EIMSK|=(1<<INT1);  // unmask Interrupt 1
}

void dodo(char etat) { 

  //ZG2100_CSoff(); 
  LEDConn_off();
  //PIND |= _BV(4);//PORTD |= _BV(4); //digitalWrite(4, HIGH);

  EIMSK&=~(1<<INT0);  // mask Interrupt 0
  EIMSK&=~(1<<INT1);  // mask Interrupt 1

    //disable timer2
  // Remove the clock source to shutdown Timer2
  TCCR2B &= ~(1 << CS22);
  TCCR2B &= ~(1 << CS21);
  TCCR2B &= ~(1 << CS20);

  while(!f_timer) {
#ifdef DEBUG
    Serial.print(etat); 
    delay(10);
#endif    
    sleepNow(); // ATmega328 goes to sleep for about 8 seconds and continues to execute code when it wakes up

  } 
  // enable Timer_2
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);

  EIMSK|=(1<<INT0);  // unmask Interrupt 0
  EIMSK|=(1<<INT1);  // unmask Interrupt 1

    //PIND |= _BV(4); //PORTD &= ~_BV(4);//digitalWrite(4, LOW);
  //ZG2100_CSon(); 
  LEDConn_on();
}


void sleepNow()   
{  
  byte spi_save = SPCR;
  //  SPCR = 0;

  //  clock_prescale_set(value);
  cbi(ADCSRA,ADEN); //  ADCSRA |= (0<<ADEN); // Switch Analog to Digital converter OFF

  //  enable_low_power(true);
 // Important note! You must use the PRR after setting ADCSRA to zero, 
 // otherwise the ADC is "frozen" in an active state.
  // turn off various modules
  //  PRR = 0xFF; 
  PRR = (1<<PRTWI) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRTIM2) | (1<<PRSPI) | (1<<PRADC) | (1<<PRUSART0);
  //Bit 7 - PRTWI: Power Reduction TWI
  //Bit 6 - PRTIM2: Power Reduction Timer/Counter2
  //Bit 5 - PRTIM0: Power Reduction Timer/Counter0
  //Bit 4 - Res: Reserved bit
  //Bit 3 - PRTIM1: Power Reduction Timer/Counter1
  //Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
  //Bit 1 - PRUSART0: Power Reduction USART0
  //Bit 0 - PRADC: Power Reduction ADC


  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
  sleep_enable(); // Enable sleep mode.

  //07012013 
  sleep_bod_disable();
  // ou
  // turn off brown-out enable in software
  // MCUCR = _BV (BODS) | _BV (BODSE);
  // MCUCR = _BV (BODS); 

  sleep_cpu(); // Enter sleep mode.
  // After waking from watchdog interrupt the code continues
  // to execute from this point

  sleep_disable(); // Disable sleep mode after waking.

  // turn on various modules
  PRR = 0x00;

  sbi(ADCSRA,ADEN); //  ADCSRA |= (1<<ADEN); // Switch Analog to Digital converter ON

    //  SPCR = spi_save;
}

void watchdogOn() {
  //Watchdog Timer Prescale Select
  //   WDP3   WDP2   WDP1     WDP0         Number of WDT     Typical Time-out at

  //   0         0      0        0             2K (2048) cycles       16 ms
  //   0         0      0        1             4K (4096) cycles       32 ms
  //   0         0      1        0             8K (8192) cycles       64 ms
  //   0         0      1        1            16K (16384) cycles    0.125 s
  //   0         1      0        0            32K (32768) cycles    0.25 s
  //   0         1      0        1            64K (65536) cycles    0.5 s
  //   0         1      1        0            128K (131072) cycles 1.0 s
  //   0         1      1        1            256K (262144) cycles 2.0 s
  //   1         0      0        0            512K (524288) cycles 4.0 s
  //   1         0      0        1            1024K (1048576) cycles 8.0 s

  MCUSR &= ~(1<<WDRF);
  WDTCSR  = (1<<WDCE | 1<<WDE);     // watchdog change enable
  //  WDTCSR  = (1<<WDIE) | (1<<WDP3) | (1<<WDP0); // set  prescaler to 8 second
  // WDTCSR  = _BV(WDIE) | _BV(WDP0) | _BV(WDP1) | _BV(WDP2);     // enable WD 2 sec
  WDTCSR  = _BV(WDIE) | _BV(WDP0) | _BV(WDP2);     // enable WD 0.5 sec
}

ISR(WDT_vect) { 
  (ISRCount++ >= sleep_count)?f_timer=true:f_timer=false; 
  // keep track of how many sleep cycles have been completed.
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  //Serial.println("test0");
  ac1 = bmp085ReadInt(0xAA);
  //Serial.println("test1");
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  //Serial.println("test2");
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6>>12))>>11;
  x2 = ac2 * b6>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = ac3 * b6>>13;
  x2 = (b1 * (b6 * b6>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  //Serial.println("test3");
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  //Serial.println("test4");
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();
  //Serial.println("test5");
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

