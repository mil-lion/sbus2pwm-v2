/*
 * File:    sbus2pwm.ino
 * Project: sbus2pwm
 * Author:  Igor Morenko <imorenko@yandex.ru>
 * Date:    18.01.2019
 */

//#include <Arduino.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>

#include "sbus.h"
#include "config.h"

// Use to enable output of PPM values to serial
//#define SERIALOUT
//#define DEBUG_SERVO


// Bit Fubctions
#define bitRead(value, bit)  (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)   ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


// ********* Servo ************
volatile int curr_chA = 0;
volatile int curr_chB = 0;

// Channels value for test
//int channels[CHANNELS_COUNT] = {
//  //////// BANK A ////////
////  0, 256, 512, 768, 1024, 1536, 1792, 2048, 
//  2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
//  //////// BANK B ////////
////  0, 256, 512, 768, 1024, 1536, 1792, 2048
//  2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
//};

// ********** SBUS ************

// public variables
// 16 channel (11 bit) + 2 digital channel
int16_t channels[SBUS_MAX_CHANNELS] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t failsafeStatus = SBUS_SIGNAL_FAILSAFE;
int toChannels = 0;

// private variables
uint8_t inBuffer[SBUS_FRAME_SIZE];
int bufferIndex = 0;
uint8_t inData;
int feedState = 0;
static sbusFrame_t sbusFrame;


// SETUP
void setup() {
  cli(); // disable interrupts
  
  // put your setup code here, to run once:
#ifdef SERIALOUT
  Serial.begin(115200);
  Serial.println("Setup!");
#endif

  // Set Port on Output
#if defined(__AVR_ATmega32U4__)
  DDRB = DDRB | B01111111; // x,D10,D9,D8,MISO,MOSI,SCK,RX LED
  DDRC = DDRC | B01000000; // x,D5,x,x,x,x,x,x
  DDRD = DDRD | B10110011; // D6,x,TX LED,D4,x,x,D2,D3
  DDRE = DDRE | B01000000; // x,D7,x,x,x,x,x,x
  DDRF = DDRF | B11110000; // A0,A1,A2,A3,x,x,x,x
#else
  DDRB = DDRB | B00111111; // -,-,D13,D12,D11,D10,D9,D8
  DDRC = DDRC | B00001111; // x,x,x,x,A3,A2,A1,A0
  DDRD = DDRD | B11111100; // D7,D6,D5,D4,D3,D2,x,x
#endif

  // SBUS Initialize
  sbusBegin();

  // Timer Initialize
  servoInitTimer();
  
  sei(); // enable interrupts
}

// LOOP
void loop() {
  // put your main code here, to run repeatedly:
#if defined(__AVR_ATmega32U4__)
  RXLED_off; //RXLED0;
#endif

  sbusFeedLine();
  if (toChannels == 1) {
#if defined(__AVR_ATmega32U4__)
    RXLED_on; //RXLED1;
#endif
    sbusUpdateChannels();
    toChannels = 0;

#if defined(__AVR_ATmega32U4__)
    if (failsafeStatus == SBUS_SIGNAL_OK) {
      TXLED_off; //TXLED0;
    } else {
      // signal lost or failsafe
      TXLED_on; //TXLED1;
    }
#endif

#ifdef SERIALOUT
    for (int i = 0; i < CHANNELS_COUNT; i++) {
      Serial.print(channels[i]);
      Serial.print("\t");
    }
    Serial.println();
#endif
  }
}

// ********** Servo ************

void servoInitTimer() {
  // Timer 1 (16-bit, triple output compare, single input capture)
  // Stop timer
  TCCR1A = 0;
  TCCR1B = 0;   // stop timer: Clock Source: OFF
  
  // Clear Counter
  TCNT1  = 0;   // setup: timer/counter 1 value
  
  // Set Mode
  TCCR1A = 0;   // mode 0: Normal: Timer UP to 0xFFFF value, reset to 0x0000 
  TCCR1B = 0;
  
  // Set Clock (set prescaler)
  TCCR1B |= (0<<CS12) | (1<<CS11) | (0<<CS10); // Clock Source: /8 (Step 0.5us)
//  TCCR1B |= (1<<CS12) | (0<<CS11) | (1<<CS10); // Clock Source: /1024 (Step 64us)
  
  // Enable interrupt on compare match
//  TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B) | (1<<TOIE1); // register A,B и сброса счетчика (overflow)
  TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B); // register A,B

  // Set compare register A & B
  OCR1A = REFRESH_INTERVAL_TICKETS;  // timer/counter 1 output compare register A
  OCR1B = REFRESH_INTERVAL_TICKETS; // timer/counter 1 output compare register B

  StartFrame();
}

void StartFrame() {
  //////// Start Frame ////////
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
  Serial.println("Start Frame!");
#endif
  // Clear timer 1 value
  TCNT1 = 0;

  //////// BANK A ////////
  // Set first channel 1
  curr_chA = 0; 
  // Set End Time Pulse
  OCR1A = TCNT1 + SET_TIME(curr_chA);
  // Start Pulse A
  ServoOn(curr_chA);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
  RXLED_on;
  Serial.print(TCNT1);
  Serial.print(" : Start Pulse A Channel ");
  Serial.print(curr_chA);
  Serial.print(" -> ");
  Serial.println(OCR1A);
#endif
  
  //////// BANK B ////////
  // Set first channel 1
  curr_chB = 8; 
  // Set End Time Pulse
  OCR1B = TCNT1 + SET_TIME(curr_chB);
  // Start Pulse B
  ServoOn(curr_chB);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
  TXLED_on;
  Serial.print(TCNT1);
  Serial.print(" : Start Pulse B Channel ");
  Serial.print(curr_chB);
  Serial.print(" -> ");
  Serial.println(OCR1B);
#endif
}

ISR (TIMER1_COMPA_vect)
{
#if defined(SERIALOUT) && defined(DEBUG_SERVO)
  Serial.print(TCNT1);
  Serial.println(" : BANK A! (TIMER1_COMPA)");
#endif

  if (curr_chA < 0) {
    StartFrame();
  } else {
    //////// Update BANK A ////////
    // Stop Pulse A
    ServoOff(curr_chA);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)
    RXLED_off;
    Serial.print(TCNT1);
    Serial.print(" : Stop Pulse A Channel ");
    Serial.println(curr_chA);
#endif
    // Next channel
    curr_chA++;
    if (curr_chA < 8) {
      // Set End Time Pulse
      OCR1A = TCNT1 + SET_TIME(curr_chA);
      // Start Pulse A
      ServoOn(curr_chA);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)
      RXLED_on;
      Serial.print(TCNT1);
      Serial.print(" : Start Pulse A Channel ");       
      Serial.print(curr_chA);
      Serial.print(" -> ");
      Serial.println(OCR1A);
#endif
    } else {
      //////// End Frame ////////
      if ((TCNT1 + 4) > REFRESH_INTERVAL_TICKETS) {
        OCR1A = TCNT1 + 4;
      } else {
        OCR1A = REFRESH_INTERVAL_TICKETS;
      }
      curr_chA = -1;
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
      Serial.print(TCNT1);
      Serial.print(" : Set End Frame -> ");       
      Serial.println(OCR1A);
#endif      
    }
  }
}

ISR (TIMER1_COMPB_vect)
{
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
  Serial.print(TCNT1);
  Serial.println(" : BANK B! (TIMER1_COMPB)");
#endif
  
  //////// Update BANK B ////////
  // Stop Pulse B
  ServoOff(curr_chB);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
  TXLED_off;
  Serial.print(TCNT1);
  Serial.print(" : Stop Pulse B Channel ");
  Serial.println(curr_chB);
#endif
  // Next channel
  curr_chB++;
  if (curr_chB < 16) {
    // Set End Time Pulse
    OCR1B = TCNT1 + SET_TIME(curr_chB);
    // Start Pulse A
    ServoOn(curr_chB);
#if defined(SERIALOUT) && defined(DEBUG_SERVO)      
    TXLED_on;
    Serial.print(TCNT1);
    Serial.print(" : Start Pulse B Channel ");       
    Serial.print(curr_chB);
    Serial.print(" -> ");
    Serial.println(OCR1B);
#endif
  }
}

//ISR (TIMER1_OVF_vect)
//{
//    Serial.println("TIMER1_OVF");
//}

void ServoOn(int ch) {
  switch(ch) {
    case 0:  CH0_on;  break;  
    case 1:  CH1_on;  break;  
    case 2:  CH2_on;  break;  
    case 3:  CH3_on;  break;  
    case 4:  CH4_on;  break;  
    case 5:  CH5_on;  break;  
    case 6:  CH6_on;  break;  
    case 7:  CH7_on;  break;  
    case 8:  CH8_on;  break;  
    case 9:  CH9_on;  break;  
    case 10: CH10_on; break;  
    case 11: CH11_on; break;  
    case 12: CH12_on; break;  
    case 13: CH13_on; break;  
    case 14: CH14_on; break;  
    case 15: CH15_on; break;  
  }
}

void ServoOff(int ch) {
  switch(ch) {
    case 0:  CH0_off;  break;  
    case 1:  CH1_off;  break;  
    case 2:  CH2_off;  break;  
    case 3:  CH3_off;  break;  
    case 4:  CH4_off;  break;  
    case 5:  CH5_off;  break;  
    case 6:  CH6_off;  break;  
    case 7:  CH7_off;  break;  
    case 8:  CH8_off;  break;  
    case 9:  CH9_off;  break;  
    case 10: CH10_off; break;  
    case 11: CH11_off; break;  
    case 12: CH12_off; break;  
    case 13: CH13_off; break;  
    case 14: CH14_off; break;  
    case 15: CH15_off; break;  
  }
}

// ********** SBUS ************

void sbusBegin() {
  /*, SP_2_STOP_BIT | SP_EVEN_PARITY | SP_8_BIT_CHAR */
  port.begin(SBUS_BAUDRATE, SBUS_PORT_OPTIONS);
  failsafeStatus = SBUS_SIGNAL_FAILSAFE;
  toChannels = 0;
  bufferIndex = 0;
  feedState = 0;
#if defined(__AVR_ATmega32U4__)
  // signal lost or failsafe
  TXLED_on; //TXLED1;
#endif
}

// Read channel data
//int16_t channel(uint8_t ch) {
//  if ((ch > 0) && (ch <= 16)) {
//    return channels[ch-1];
//  } else {
//    return 1023;
//  }
//}

// Read digital channel data
//uint8_t digiChannel(uint8_t ch) {
//  if ((ch > 0) && (ch <= 2)) {
//    return channels[15+ch];
//  } else {
//    return 0;
//  }
//}

//uint8_t failsafe(void) {
//  return failsafeStatus;
//}

void sbusUpdateChannels(void) {
  // using structure
  channels[0]  = sbusFrame.frame.ch0;
  channels[1]  = sbusFrame.frame.ch1;
  channels[2]  = sbusFrame.frame.ch2;
  channels[3]  = sbusFrame.frame.ch3;
  channels[4]  = sbusFrame.frame.ch4;
  channels[5]  = sbusFrame.frame.ch5;
  channels[6]  = sbusFrame.frame.ch6;
  channels[7]  = sbusFrame.frame.ch7;
#ifdef ALL_CHANNELS
  // & the other 8 + 2 channels if you need them
  channels[8]  = sbusFrame.frame.ch8;
  channels[9]  = sbusFrame.frame.ch9;
  channels[10] = sbusFrame.frame.ch10;
  channels[11] = sbusFrame.frame.ch11;
  channels[12] = sbusFrame.frame.ch12;
  channels[13] = sbusFrame.frame.ch13;
  channels[14] = sbusFrame.frame.ch14;
  channels[15] = sbusFrame.frame.ch15;

#ifdef DIGITAL_CHANNELS
  // DigiChannel 1
  if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
    channels[16] = SBUS_DIGITAL_CHANNEL_MAX;
  } else {
    channels[16] = SBUS_DIGITAL_CHANNEL_MIN;
  }

  // DigiChannel 2
  if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
    channels[17] = SBUS_DIGITAL_CHANNEL_MAX;
  } else {
    channels[17] = SBUS_DIGITAL_CHANNEL_MIN;
  }
#endif

#endif

  // Failsafe
  failsafeStatus = SBUS_SIGNAL_OK;
  if (sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {
    failsafeStatus = SBUS_SIGNAL_LOST;
  }
  if (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
    // internal failsafe enabled and rx failsafe flag set
    failsafeStatus = SBUS_SIGNAL_FAILSAFE;
  }
}

void sbusFeedLine() {
  if (port.available() >= SBUS_FRAME_SIZE) {
    while (port.available() > 0) {
      inData = port.read();
      if (0 == feedState) {
        // feedState == 0
        if (inData != SBUS_START_BYTE){
          //read the contents of in buffer this should resync the transmission
          while (port.available() > 0) {
            inData = port.read();
          }
          return;
        } else {
          bufferIndex = 0;
          inBuffer[bufferIndex] = inData;
          inBuffer[SBUS_FRAME_SIZE-1] = 0xff;
          feedState = 1;
        }
      } else {
        // feedState == 1
        bufferIndex ++;
        inBuffer[bufferIndex] = inData;
        if (bufferIndex < (SBUS_FRAME_SIZE-1)
         && port.available() == 0) {
          feedState = 0;
        }
        if (bufferIndex == (SBUS_FRAME_SIZE-1)) {
          feedState = 0;
          if (inBuffer[0] == SBUS_START_BYTE
           && inBuffer[SBUS_FRAME_SIZE-1] == SBUS_END_BYTE) {
            memcpy(sbusFrame.bytes, inBuffer, SBUS_FRAME_SIZE);
            toChannels = 1;
          }
        }
      }
    }
  }
}

