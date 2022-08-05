//LCD 128x64
#include <Wire.h>
#include <SPI.h>
#include <I2C_graphical_LCD_display.h>

I2C_graphical_LCD_display lcd;
const byte picture [] PROGMEM = {
 0xc, 0x12, 0x12, 0xc, 0x0, 0x0, 0x0,0x0,
};

//Relay & Pompa
int pbuttonPin = A0;// connect output to push button
int relayPin1 = 10;// Connected to relay 2 (PUMP)
int relayPin2 = 9;// Connected to relay 2 (PUMP)

int val = 0; // push value from pin 2
int pumpON = 0;//pump status
int pushed = 0;//push status


//Sensor DHT22
#include "DHT.h"
#define DHTPIN 12     // DHT PIN 12
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);


//Frequency Counter
//Input: Pin D5

volatile unsigned long timerCounts;
volatile boolean counterReady;

// internal to counting routine
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;

void startCounting (unsigned int ms) 
  {
  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet

  // reset Timer 1 and Timer 2
  TCCR1A = 0;             
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer 1 - counts events on pin D5
  TIMSK1 = bit (TOIE1);   // interrupt on Timer 1 overflow

  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs. 
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)

  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt

  TCNT1 = 0;      // Both counters to zero
  TCNT2 = 0;     

  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
  }  // end of startCounting

ISR (TIMER1_OVF_vect)
  {
  ++overflowCount;               // count number of Counter1 overflows  
  }  // end of TIMER1_OVF_vect


//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect) 
  {
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;

  // see if we have reached timing period
  if (++timerTicks < timerPeriod) 
    return;  // not yet

  // if just missed an overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256)
    overflowCopy++;

  // end of gate time, measurement ready
  TCCR1A = 0;    // stop timer 1
  TCCR1B = 0;    

  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    

  TIMSK1 = 0;    // disable Timer1 Interrupt
  TIMSK2 = 0;    // disable Timer2 Interrupt
    
  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
  }  // end of TIMER2_COMPA_vect

void setup () 
  {
Serial.begin(9600);       
dht.begin();
pinMode(pbuttonPin, INPUT_PULLUP); 
pinMode(relayPin1, OUTPUT);
pinMode(relayPin2, OUTPUT);
digitalWrite(relayPin1, HIGH);
digitalWrite(relayPin2, HIGH);

lcd.begin ();  
TWBR = 12;
lcd.gotoxy(0,0);
lcd.string("Frekuensi :");
lcd.gotoxy(108,0);
lcd.string("Hz");
lcd.gotoxy(0,8);
lcd.string("Suhu      :");
lcd.gotoxy(78,8);
lcd.blit(picture, sizeof picture);
lcd.gotoxy(83,8);
lcd.string("C");
lcd.gotoxy(0,16);
lcd.string("Kelembaban:");
lcd.gotoxy(78,16);
lcd.string("%");
lcd.gotoxy(0,40);
lcd.string("_____________________");
lcd.gotoxy(12,50);
lcd.string("Frequency Counter");
lcd.gotoxy(8,60);
lcd.string("Laboratory of AQAI");
} // end of setup

void loop () 
{
val = digitalRead(pbuttonPin);// read the AO push button value

if(val == HIGH && pumpON == LOW){
pushed = 1-pushed;
}    

pumpON = val;

if(pushed == HIGH){
digitalWrite(relayPin1, LOW);
digitalWrite(relayPin2, LOW); 
lcd.gotoxy(0,34);
lcd.string("PUMP OFF/ON");
lcd.gotoxy(29,34);
lcd.string("OFF", true);
}
else{
digitalWrite(relayPin1, HIGH);
digitalWrite(relayPin2, HIGH);
lcd.gotoxy(0,34);
lcd.string("PUMP OFF/ON");
lcd.gotoxy(53,34);
lcd.string("ON", true);
}

int h = dht.readHumidity();
int t = dht.readTemperature();
  
// Check if any reads failed and exit early (to try again).
if (isnan(h) || isnan(t) ) {
Serial.println("Failed to read from DHT sensor!");
return;
}

byte oldTCCR0A = TCCR0A;
byte oldTCCR0B = TCCR0B;
TCCR0A = 0;
TCCR0B = 0;  
  
startCounting (500);

while (!counterReady) 
{ }
float frq = (timerCounts *  1000.0) / timerPeriod;
Serial.print ((unsigned long) frq);
Serial.print("Hz");   
Serial.print("\t");
Serial.print(t);
Serial.print("\t");
Serial.print(h);
Serial.print("\n");

lcd.gotoxy(65,0);
lcd.print((unsigned long) frq);
lcd.gotoxy(65,8);
lcd.print(t);
lcd.gotoxy(65,16);
lcd.print(h);


TCCR0A = oldTCCR0A;
TCCR0B = oldTCCR0B;

delay(600);
}
