#include "Hardware.h"
#include "AtomicBlock.h"
#include <Arduino.h>
#include "SPI.h"
#include <wiring_private.h>
#include <assert.h>

void Pulse(int DIO)
{
  if(digitalRead(DIO)==HIGH)
  {
    digitalWrite(DIO,LOW);
    delayMicroseconds(2);
    digitalWrite(DIO,HIGH);
  }
  else
  {
    digitalWrite(DIO,HIGH);
    delayMicroseconds(2);
    digitalWrite(DIO,LOW);
  }
}

// Reads the selected ADC channel for the number of averages defined by num
int GetADCvalue(int chan, int num)
{
  int i=0,j;

  for(j=0;j<num;j++) i += analogRead(chan);
  return i/num;
}

// This function reads and returns an ADC channel value. The raw ADC
// value is read and converted to engineering units.
float ReadADCchannel(ADCchan adch, int num)
{
  int adc = GetADCvalue(adch.Chan,num);
  return Counts2Value(adc,&adch);
}

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// AD5592 IO routines. This is a analog and digitial IO chip with
// a SPI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5592
void AD5592write(int addr, uint8_t reg, uint16_t val)
{
  digitalWrite(addr,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  delayMicroseconds(1);
  digitalWrite(addr,HIGH);
}

// Read from AD5593R
// returns 16 bit value read
int AD5592readWord(int addr)
{
  uint16_t  val;

  digitalWrite(addr,LOW);
  val = SPI.transfer16(0);
  delayMicroseconds(1);
  digitalWrite(addr,HIGH);
  return val;
}

// Returns -1 on error. Error is flaged if the readback channel does not match the
// requested channel.
// chan is 0 thru 7
int AD5592readADC(int addr, int8_t chan)
{
   uint16_t  val;

   AtomicBlock< Atomic_RestoreState >    a_Block;
   // Write the channel to convert register
   AD5592write(addr, 2, 1 << chan);
   delayMicroseconds(2);
   // Dummy read
   digitalWrite(addr,LOW);
   SPI.transfer16(0);
   delayMicroseconds(1);
   digitalWrite(addr,HIGH);
   // Read the ADC data 
   delayMicroseconds(2);
   digitalWrite(addr,LOW);
   val = SPI.transfer16(0);
   delayMicroseconds(1);
   digitalWrite(addr,HIGH);
   delayMicroseconds(2);
   // Test the returned channel number
   if(((val >> 12) & 0x7) != chan) return(-1);
   // Left justify the value and return
   val <<= 4;
   return(val & 0xFFF0);
}

int AD5592readADC(int addr, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5592readADC(addr, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

void AD5592writeDAC(int addr, int8_t chan, int val)
{
   uint16_t  d;

   AtomicBlock< Atomic_RestoreState >    a_Block;
   // convert 16 bit DAC value into the DAC data data reg format
   d = ((val>>4) & 0x0FFF) | (((uint16_t)chan) << 12) | 0x8000;
   digitalWrite(addr,LOW);
   val = SPI.transfer((uint8_t)(d >> 8));
   val = SPI.transfer((uint8_t)d);
   delayMicroseconds(1);
   digitalWrite(addr,HIGH);
}

// End of AD5592 routines


void Software_Reset(void)
{
  // in globals declaration section
  #define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
  #define CPU_RESTART_VAL 0x5FA0004
  #define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

  CPU_RESTART;
}

// Vector to the boot loader
void bootLoader(void)
{
  _reboot_Teensyduino_();
}
