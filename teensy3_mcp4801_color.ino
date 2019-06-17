#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>

#define AUDIO       A9
#define RAMPRESET   2
#define DAC1SEL     3
#define DAC2SEL     4
#define DAC3SEL     5
#define DAC4SEL     6
//#define DAC5SEL     7
#define LDACALL     14

// no buffer on vref, gain of 2 (4.096V), shdn disabled  
// gives us the high Nybble of the high Byte below
#define 4801CONFIGBITS 0x10

#define _DEBUG_
//#undef _DEBUG_


uint8_t band1,band2,band3,band4;
uint8_t dacoutH,dacoutL;

// this setup changes AREF to internal 1.2v bandgap reference
// GUItool: begin automatically generated code
AudioInputAnalog         adc1(AUDIO);
AudioAnalyzeFFT256       fft256_1;
AudioConnection          patchCord1(adc1, fft256_1);
// GUItool: end automatically generated code



void setup() {
#ifdef _DEBUG_
    Serial.begin(115200);
    Serial.print("TEST");
#endif

    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.begin();
    AudioMemory(8);

    digitalWrite(RAMPRESET, HIGH);
    pinMode(RAMPRESET,OUTPUT);    
    digitalWrite(DAC1SEL,HIGH);
    pinMode(DAC1SEL,OUTPUT); 
    digitalWrite(DAC2SEL,HIGH);
    pinMode(DAC2SEL,OUTPUT);
    digitalWrite(DAC3SEL,HIGH);
    pinMode(DAC3SEL,OUTPUT);
    digitalWrite(DAC4SEL,HIGH);
    pinMode(DAC4SEL,OUTPUT);
    // digitalWrite(DAC5SEL,HIGH);
    // pinMode(DAC5SEL,OUTPUT);
    digitalWrite(LDACALL,HIGH);
    pinMode(LDACALL,OUTPUT);
    pinMode(AUDIO,INPUT);

}

void loop() {
    // put your main code here, to run repeatedly:
    //digitalWrite(13, !(digitalRead(13)));
    if (fft256_1.available()) {

        band1=(uint8_t) (fft256_1.read(0,7)*256);
        band2=(uint8_t) (fft256_1.read(8,23)*256);
        band3=(uint8_t) (fft256_1.read(24,55)*256);
        band4=(uint8_t) (fft256_1.read(56,127)*256);
#ifdef _DEBUG_
        Serial.print("\n\r");
        Serial.print(band1);
        Serial.print(',');
        Serial.print(band2);
        Serial.print(',');
        Serial.print(band3);
        Serial.print(',');
        Serial.print(band4);
        Serial.print("\n\r");
        Serial.print(AudioMemoryUsageMax());
        
#endif
        // band 1, DAC1
        
        dacoutH = 4801CONFIGBITS | ((band1>>4) & 0x0F);
        dacoutL = (band1<<4) & 0xF0;
        digitalWrite(DAC1SEL, LOW);
        delayMicroseconds(5);            // let the DAC get ready
        SPI.transfer(dacoutH);
        SPI.transfer(dacoutL);
        delayMicroseconds(5);
        digitalWrite(DAC1SEL, HIGH);

        // wait a little before the B dac
        delayMicroseconds(5);

        // band 2, DAC2
        dacoutH = 4801CONFIGBITS | ((band2>>4) & 0x0F);
        dacoutL = (band2<<4) & 0xF0;
        digitalWrite(DAC2SEL, LOW);
        delayMicroseconds(5);
        SPI.transfer(dacoutH);
        SPI.transfer(dacoutL);
        delayMicroseconds(5);            // let the DAC settle
        digitalWrite(DAC2SEL, HIGH);
        // done with dac1 A and B


        // band 3, DAC3
        dacoutH = 4801CONFIGBITS | ((band3>>4) & 0x0f);
        dacoutL = (band3<<4) & 0xF0;
        digitalWrite(DAC3SEL, LOW);
        delayMicroseconds(5);            // let the DAC get ready
        SPI.transfer(dacoutH);
        SPI.transfer(dacoutL);
        delayMicroseconds(5);
        digitalWrite(DAC3SEL, HIGH);

        // wait a little before the B dac
        delayMicroseconds(5);

        // band 4 DAC4
        dacoutH = 4801CONFIGBITS | ((band4>>4) & 0x0f);
        dacoutL = (band4<<4) & 0xF0;
        digitalWrite(DAC4SEL, LOW);
        delayMicroseconds(5);
        SPI.transfer(dacoutH);
        SPI.transfer(dacoutL);
        delayMicroseconds(5);            // let the DAC settle
        digitalWrite(DAC4SEL, HIGH);
        //done with dac2 A and B

        // wait a little then
        // latch all dac data to outputs
        delayMicroseconds(5);
        digitalWrite(LDACALL,LOW);
        delayMicroseconds(1);
        digitalWrite(LDACALL,HIGH);
    }
    digitalWrite(RAMPRESET,LOW);
    delayMicroseconds(100);
    digitalWrite(RAMPRESET,HIGH);
    delay(8); // about 120 hz for testing, replace with zero cross detection
}
