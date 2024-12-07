#include "Arduino.h"
#include "AD9959.h"
#include <SPI.h>

DDS::DDS(void)
{
  // Set default pin assignments
  ResetPin = 0;
  ChipEnablePin = 10;
  UpdatePin = 7;
  reference_freq = 500000000;
  mult = 1;
  profile = 0;
  // Set default register bits
  CSRreg = MSB_First | IO3Wire;
  CFRreg = DACFullScale | MatchPipeDelay | OutputSineWave;
  FR1reg = SyncClkDisable | ModLevels2 | RampUpDownOff | ChargePump3;
  FR1reg |= (mult << PllDividerBit) & PllDividerMSK;
  FR1reg |= (profile << ProfileBit) & ProfileMSK;
  if (reference_freq > 200000000) FR1reg |= VCOGain;
}

void DDS::begin(void)
{
  // Set the bit directions and init
  digitalWrite(ResetPin, 0);
  pinMode(ResetPin, OUTPUT);          // Ensure we can reset the AD9959
  digitalWrite(ChipEnablePin, 1);
  pinMode(ChipEnablePin, OUTPUT);     // This control signal applies the loaded values
  digitalWrite(UpdatePin, 0);
  pinMode(UpdatePin, OUTPUT);         // This control signal applies the loaded values
  // Reset the DDS chip and start things up!
  pulse(ResetPin);                    // (minimum 5 cycles of the 30MHz clock)
  pulse(UpdatePin);
  setChannels(ChannelAll);
  write(CFR, CFRreg);
  setChannels(ChannelNone);           // Disable all channels, set 3-wire MSB mode:
  pulse(UpdatePin);                   // Apply the changes
  write(FR1, FR1reg);
}

void DDS::begin(uint8_t Rpin, uint8_t CSpin, uint8_t Upin, unsigned long ref)
{
  ResetPin = Rpin;
  ChipEnablePin = CSpin;
  UpdatePin = Upin;
  reference_freq = ref;
  begin();
}

// To read channel registers, you must first use setChannels to select exactly one channel!
uint32_t DDS::read(Register reg)
{
  return write(0x80 | reg, 0); // The zero data is discarded, just the return value is used
}

void DDS::setChannels(ChannelNum chan)
{
  static int last_channels = -1;

  if (last_channels != chan) write(CSR, chan | CSRreg);
  last_channels = chan;
}


uint32_t DDS::write(uint8_t reg, uint32_t value, int len)
{
  uint32_t    rval = 0;
  
  chipEnable();
  SPI.transfer(reg);
  while (len-- > 0)
    rval = (rval << 8) | SPI.transfer((value >> len * 8) & 0xFF);
  //SPI.endTransaction();
  chipDisable();
  return rval;  
}

uint32_t DDS::write(uint8_t reg, uint32_t value)
{
  // The indices of this array match the values of the Register enum:
  static constexpr uint8_t register_length[8] = { 1, 3, 2, 3, 4, 2, 3, 2 };  // And 4 beyond that

  uint32_t    rval = 0;
  int         len = (reg & 0x7F) < sizeof(register_length) / sizeof(uint8_t) ? register_length[reg & 0x07] : 4;

  chipEnable();
  SPI.transfer(reg);
  while (len-- > 0)
    rval = (rval << 8) | SPI.transfer((value >> len * 8) & 0xFF);
  //SPI.endTransaction();
  chipDisable();
  return rval;
}

void DDS::update(void)
{
  pulse(UpdatePin);
}

uint32_t DDS::calculateFWT(double freq)
{
  return(freq / (double)reference_freq) * (double)MAX_U32;
}

void DDS::setFrequency(ChannelNum chan, double freq)
{
  uint32_t FWT;

  FWT = (freq / (double)reference_freq) * (double)MAX_U32;
  setChannels(chan);
  write(CFTW, FWT);
}

void DDS::setPhase(ChannelNum chan, uint16_t phase)                // Maximum phase value is 16383
{
  setChannels(chan);
  write(CPOW, phase & 0x3FFF);
}

void DDS::setAmplitude(ChannelNum chan, uint16_t amplitude)
{
  uint32_t ACRreg;

  if (amplitude > 1024) amplitude = 1024;  // Clamp to the maximum
  ACRreg = amplitude & 0x3FF;
  if (amplitude == 1024) ACRreg |= MultiplierEnable;

  setChannels(chan);
  write(ACR, ACRreg);
}

void DDS::shutdown(ChannelNum chan, bool state)
{
  setChannels(chan);
  if (state) write(CFR, CFRreg | (CFR_Bits)CFR_Bits::DACPowerDown);
  else write(CFR, CFRreg);
}

void DDS::enableModulation(ChannelNum chan, bool state)
{
  CFRreg &= ~PhaseModulation;
  if(state) CFRreg |= PhaseModulation;
  write(CFR, CFRreg);
}
