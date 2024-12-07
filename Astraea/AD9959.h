#ifndef AD9959_h
#define AD9959_h

#define PHASEOFFSETRANGE 16384
#define PHASEMARGIN      100

// AD9959 data / register types and bot definitions
typedef enum
{
  ChannelNone = 0x00,
  Channel0    = 0x10,
  Channel1    = 0x20,
  Channel2    = 0x40,
  Channel3    = 0x80,
  ChannelAll  = 0xF0,
} ChannelNum;

// See register_length[] in write() before re-ordering these.
typedef enum         // There are 334 bytes in all the registers! See why below...
{
  CSR               = 0x00,   // 1 byte, Channel Select Register
  FR1               = 0x01,   // 3 bytes, Function Register 1
  FR2               = 0x02,   // 2 bytes, Function Register 2
  // The following registers are duplicated for each channel.
  // A write goes to any and all registers enabled in channel select (CSR)
  // To read successfully you must first select one channel
  CFR               = 0x03,   // 3 bytes, Channel Function Register (one for each channel!)
  CFTW              = 0x04,   // 4 bytes, Channel Frequency Tuning Word
  CPOW              = 0x05,   // 2 bytes, Channel Phase Offset Word (aligned to LSB, top 2 bits unused)
  ACR               = 0x06,   // 3 bytes, Amplitude Control Register (rate byte, control byte, scale byte)
  LSRR              = 0x07,   // 2 bytes, Linear Sweep Rate Register (falling, rising)
  RDW               = 0x08,   // 4 bytes, Rising Delta Word
  FDW               = 0x09,   // 4 bytes, Falling Delta Word
  // The following registers (per channel) are used to provide 16 modulation values
  // This library doesn't provide modulation. Only CW1 is used, for sweep destination.
  CW1               = 0x0A,   // 4 bytes, Channel Word 1-15 (phase & amplitude MSB aligned)
  CW2               = 0x0B,
  CW3               = 0x0C,
  CW4               = 0x0D,
  CW5               = 0x0E,
  CW6               = 0x0F,
  CW7               = 0x10,
  CW8               = 0x11,
  CW9               = 0x12,
  CW10              = 0x13,
  CW11              = 0x14,
  CW12              = 0x15,
  CW13              = 0x16,
  CW14              = 0x17,
  CW15              = 0x18
} Register;

typedef enum
{
  // Bit order selection (default MSB):
  MSB_First = 0x00,
  LSB_First = 0x01,
  // Serial I/O Modes (default IO2Wire):
  IO2Wire = 0x00,
  IO3Wire = 0x02,
  IO2Bit = 0x04,
  IO4Bit = 0x06,
} CSR_Bits;

typedef enum     // Function Register 1 is 3 bytes wide.
{
  // Most significant byte:
  // Higher charge pump values decrease lock time and increase phase noise
  ChargePump0      = 0x000000,
  ChargePump1      = 0x010000,
  ChargePump2      = 0x020000,
  ChargePump3      = 0x030000,

  PllDividerMSK    = 0x7C0000,    // PLL divider mask
  PllDividerBit    = 18,          // Bit position of PLL divider LSB
  VCOGain          = 0x800000,    // Set for ref >255MHz

  // Middle byte:
  ModLevels2       = 0x0000,      // How many levels of modulation?
  ModLevels4       = 0x0100,
  ModLevels8       = 0x0200,
  ModLevels16      = 0x0300,

  RampUpDownOff    = 0x0000,      // Which pins contol amplitude ramping?
  RampUpDownP2P3   = 0x0400,      // Profile=0 means ramp-up, 1 means ramp-down
  RampUpDownP3     = 0x0800,      // Profile=0 means ramp-up, 1 means ramp-down
  RampUpDownSDIO123= 0x0C00,      // Only in 1-bit I/O mode

  ProfileMSK       = 0x7000,      // Profile bit mask
  ProfileBit       = 12,          // Profile LSB bit position

  // Least significant byte:
  SyncAuto         = 0x00,    // Master SYNC_OUT->Slave SYNC_IN, with FR2
  SyncSoft         = 0x01,    // Each time this is set, system clock slips one cycle
  SyncHard         = 0x02,    // Synchronise devices by slipping on SYNC_IN signal

  // Software can power-down individual channels (using CFR[7:6])
  DACRefPwrDown    = 0x10,    // Power-down DAC reference
  SyncClkDisable   = 0x20,    // Don't output SYNC_CLK
  ExtFullPwrDown   = 0x40,    // External power-down means full power-down (DAC&PLL)
  RefClkInPwrDown  = 0x80,    // Disable reference clock input
} FR1_Bits;

typedef enum
{
  AllChanAutoClearSweep    = 0x8000,  // Clear sweep accumulator(s) on I/O_UPDATE
  AllChanClearSweep        = 0x4000,  // Clear sweep accumulator(s) immediately
  AllChanAutoClearPhase    = 0x2000,  // Clear phase accumulator(s) on I/O_UPDATE
  AllChanClearPhase        = 0x2000,  // Clear phase accumulator(s) immediately
  AutoSyncEnable   = 0x0080,
  MasterSyncEnable = 0x0040,
  MasterSyncStatus = 0x0020,
  MasterSyncMask   = 0x0010,
  SystemClockOffset = 0x0003,         // Mask for 2-bit clock offset controls
} FR2_Bits;

// Channel Function Register
typedef enum
{
  ModulationMode   = 0xC00000,        // Mask for modulation mode
  AmplitudeModulation = 0x400000,     // Mask for modulation mode
  FrequencyModulation = 0x800000,     // Mask for modulation mode
  PhaseModulation  = 0xC00000,        // Mask for modulation mode
  SweepNoDwell     = 0x008000,        // No dwell mode
  SweepEnable      = 0x004000,        // Enable the sweep
  SweepStepTimerExt = 0x002000,       // Reset the sweep step timer on I/O_UPDATE
  DACFullScale     = 0x000300,        // 1/8, 1/4, 1/2 or full DAC current
  DigitalPowerDown = 0x000080,        // Power down the DDS core
  DACPowerDown     = 0x000040,        // Power down the DAC
  MatchPipeDelay   = 0x000020,        // Compensate for pipeline delays
  AutoclearSweep   = 0x000010,        // Clear the sweep accumulator on I/O_UPDATE
  ClearSweep       = 0x000008,        // Clear the sweep accumulator immediately
  AutoclearPhase   = 0x000004,        // Clear the phase accumulator on I/O_UPDATE
  ClearPhase       = 0x000002,        // Clear the phase accumulator immediately
  OutputSineWave   = 0x000001,        // default is cosine
} CFR_Bits;

// Amplitude Control Register
typedef enum
{
  RampRate            = 0xFF0000,     // Time between ramp steps
  StepSize            = 0x00C000,     // Amplitude step size (00=1,01=2,10=4,11=8)
  MultiplierEnable    = 0x001000,     // 0 means bypass the amplitude multiplier
  RampEnable          = 0x000800,     // 0 means amplitude control is manual
  LoadARRAtIOUpdate   = 0x000400,     // Reload Amplitude Rate Register at I/O Update
  ScaleFactor         = 0x0003FF,     // 10 bits for the amplitude target
} ACR_Bits;

// DDS macros
#define MAX_U64 ((uint64_t)~0LL)
#define MAX_U32 ((uint32_t)~0L)
#define raise(pin)     digitalWrite(pin,HIGH)
#define lower(pin)     digitalWrite(pin,LOW)
#define chipEnable()   lower(ChipEnablePin)
#define chipDisable()  raise(ChipEnablePin)
#define pulse(pin)     {raise(pin); delayMicroseconds(1); lower(pin);}

// AD9959 dds class
class DDS
{
  protected:
    uint8_t         ResetPin;         // Reset pin (active = high)
    uint8_t         ChipEnablePin;    // Chip Enable (active low)
    uint8_t         UpdatePin;        // I/O_UPDATE: Apply config changes
    unsigned long   reference_freq;   // Use your crystal or reference frequency
    uint8_t         mult; 
    uint8_t         profile; 
  public:
    DDS(void);
    void setChannels(ChannelNum chan);
    uint8_t  CSRreg;
    uint32_t FR1reg;
    uint32_t FR2reg;
    uint32_t CFRreg;
    void begin(void);
    void begin(uint8_t Rpin, uint8_t CSpin, uint8_t Upin, unsigned long ref);
    uint32_t read(Register reg);
    uint32_t write(uint8_t reg, uint32_t value);
    uint32_t write(uint8_t reg, uint32_t value, int len);
    void update(void);
    uint32_t calculateFWT(double freq);
    void setFrequency(ChannelNum chan, double freq);
    void setPhase(ChannelNum chan, uint16_t phase);
    void setAmplitude(ChannelNum chan, uint16_t amplitude);
    void shutdown(ChannelNum chan, bool state);
    void enableModulation(ChannelNum chan, bool state);
};

#endif
