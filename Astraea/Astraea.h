#ifndef ASTRAEA_h
#define ASTRAEA_h
#include "AD9959.h"
#include "Agility.h"
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

#define SCANTIMEOUT 5000

#define SPIFASTCLK  20000000
#define SPISLOWCLK  2000000

// Host interface parameter entry limits
#define MINMOD    10
#define MAXMOD    255
#define MINMZ     100
#define MAXMZ     500000
#define MINFREQ   10000
#define MAXFREQ   500000
#define MINQ      0
#define MAXQ      100
#define MINRO     1
#define MAXRO     10
#define MINVOP    1
#define MAXVOP    500
#define MINT      5
#define MAXT      95

// IO Trigger signals
#define TrigIn    6
#define Trig1     4
#define Trig2     5

// QUAD driver board signals
#define VCS       16      // Output driver AD5592 chip select
#define VENA      15      // Output driver enable, Low = enabled

#define VENABLE  digitalWrite(VENA,LOW)
#define VDISABLE digitalWrite(VENA,HIGH)

// AD5592 channel assignments
#define   VP1DAC    0
#define   VP1ADC    1 
#define   VN1DAC    2
#define   VN1ADC    3
#define   VP2DAC    4
#define   VP2ADC    5
#define   VN2ADC    6
#define   VN2DAC    7

// DDS interface signals
#define CS        10      // DDS SPI Chip Select
#define PDC       8
#define REST      0
#define SDIO_1    18
#define SDIO_3    17
#define P0        19
#define P1        20
#define P2        21
#define P3        22

// FPGA signals
#define SCS       9       // FPGA SPI Chip Select, jumper wire, no trace on PCB
#define INTRP     3       // Interrupt input from FPGA
#define SIP       23      // Scan in progress input from FPGA
#define RST       1       // Reset signal to FPGA
#define UPDATEDDS 7       // Update DDS (DDS load) routed through FPGA
#define LOAD      2       // Load the FPGA counters

typedef struct QuadState QuadState;
typedef struct PWMchannel PWMchannel;

typedef struct
{
  bool  update;
  float VP[2];
  float VN[2];
} State;

// FPGA module addresses
#define PWM0add         0
#define PWM1add         4
#define SCANadd         8
#define SYSadd          14
// Configuration byte bit definitions
// 4 lsb are leds for status display
#define LoadOnPWM0zero  4     // If set then generate load in FPGA only on PWM channel 0 zero count
#define LoadOnPWM1zero  5     // If set then generate load in FPGA only on PWM channel 1 zero count
#define DDSupdateOnLoad 6     // If set DDSupdate pulse generated with load pulse
#define LoadCFGsNoload  7     // If set cfg bytes applied without strobe
typedef struct
{
  uint8_t addr;
  uint8_t cfg;  
} SystemCtrl;


// Configuration byte bit definitions
#define P1ENA   0         // Enable phase 1 output
#define P2ENA   1         // Enable phase 2 output
#define P1INV   2         // Invert phase 1 output 
#define P2INV   3         // Invert phase 2 output
#define SYNCP   4         // Set to enable generation of sync pulse on positive clock edge
#define SYNCN   5         // Set to enable generation of sync pulse on negative clock edge
#define P1M     6         // Set to enable phase modulation for phase 1 output
#define P2M     7         // Set to enable phase modulation for phase 2 output

struct PWMchannel
{
  uint8_t addr;
  uint8_t cfg;
  uint8_t mod;
  uint8_t pwmp1;
  uint8_t pwmp2;
};

typedef enum
{
  SCAN_FREQ,
  SCAN_MZ,
  SCAN_T1,
  SCAN_T3,
} SCANTYPE;

// Scan configuration byte bit definitions
#define  ScanReq      0     // Set to 1 to start a scan
#define  ScanAdv      1     // Set to 1 to advance scan to next point
#define  ScanAbrt     2     // Set to 1 to abort a scan in progress
#define  ScanChan     3     // Set to 1 to selects the PWM, 0 or 1
#define  ScanWFcount  4     // Set to 1 to select the count waveforms mode
typedef struct
{
  uint8_t addr;
  uint8_t cfg;
  uint16_t WFperPoint;
  uint32_t NumPoints;
} SCAN;

struct QuadState
{
  char        name;
  double      mz,freq,q,t1,t3;
  int         ch;
  int         NumWaveforms;
  ChannelNum  DDSchannel;
  ChannelNum  DDSmodChannel;
  uint32_t    DDSfreq;
  uint16_t    Phase1;
  uint16_t    Phase2;
  PWMchannel  PWM;
};

typedef struct
{
  int    modulus;
  int    minMod, maxMod;
  bool   fixedMod;
  // quad physical parameters
  double Vop;       // Voltage base to peak in volts
  double ro;        // Quad radius in mm
  double q;         // QUAD q factor
  // quad output parameters
  double freq;
  double mz;
  double t1;
  double t2;
  double t3;
  // DDS parameters
  ChannelNum  DDSchannel;
  ChannelNum  DDSmodChannel;
  double  DDSfreq;
  int DDSoffset1;
  int DDSoffset2;
  PWMchannel PWM;
} DQUAD;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the module name, "Astraea"
  int8_t        Rev;                    // Holds the board revision number
  // FPGA system control
  SystemCtrl    sysCtrl;
  // quad parameters
  bool          syncQuads;
  DQUAD         quads[2];
  // scan parameters
  int           scanCh;
  SCANTYPE      scanTyp;
  SCAN          scan;
  // Quad driver parameters
  bool          DriverPresent;
  bool          Venable;
  float         VOP;                    // QUAD voltage base to peak
  float         VRES;                   // QUAD resolving DC
  float         VP[2];                  // Positive quad voltages
  float         VN[2];                  // Negative quad voltages
  ADCchan       VOPMon[2];              // Positive DC voltage monitors
  ADCchan       VONMon[2];              // negative DC voltage monitors
  DACchan       VOPCtrl[2];             // Positive output DC voltage controls
  DACchan       VONCtrl[2];             // Negative output DC voltage controls
  // Agility parameters
  int           NumStates;
  QuadState     qs[MaxStates];
  char          stateTable[140];
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} ASTRAEA;

extern ASTRAEA  astraea;
extern float    scnStart;
extern float    scnStop;
extern int      scanSteps;
extern int      wfmsPerStep;
extern bool     generateScanTrig;
extern bool     waitForScanTrig;
extern bool     waitForScanTrigNeg;
extern bool     T3_enable[2];

bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter = FILTER);

void SaveSettings(void);
void RestoreSettings(void);
void SaveCal(void);
void LoadCal(void);
void FormatFLASH(void);
void Debug(int i);

void ReadAllSerial(void);
void ProcessSerial(bool scan=true);

void SetMod(char *ch, char *value);
void GetMod(int ch);
void SetMinMod(char *ch, char *value);
void GetMinMod(int ch);
void SetMaxMod(char *ch, char *value);
void GetMaxMod(int ch);

void SetV(char *ch, char*value);
void GetV(int ch);
void SetRo(char *ch, char *value);
void GetRo(int ch);
void SetQ(char *ch, char *value);
void GetQ(int ch);
void SetT1(char *ch, char *value);
void GetT1(int ch);
void GetT2(int ch);
void SetT3(char *ch, char *value);
void GetT3(int ch);

void SetFreq(char *ch, char *value);
void GetFreq(int ch);
void mzSet(char *ch, char *value);
void mzGet(int ch);

void Get_t1dps(int ch);
void Get_t1ps(int ch);
void Get_t3dps(int ch);
void Get_t3ps(int ch);

void readADC(void);

void rwDDSreg(char *reg, char *value);

void SetCFG(char *reg, char *value);
void GetCFG(char *reg);

void phaseSet(char *chan, char *phase);
void aline(void);

void SetLoad(void);
void SetUpdate(void);

void writeDrvDAC(int chan, int value);
void readDrvADC(int chan);

void SetConfigure(int ch);

void setScanCh(int ch);
void getScanCh(void);
void setScanType(char *scanT);
void getScanType(void);
void Scan(void);
void ScanAbort(bool report = true);

void setVOP(char *val);
void setVRES(char *val);
void setVP(char *ch, char *val);
void getVP(int ch);
void setVN(char *ch, char *val);
void getVN(int ch);
void getVPA(int ch);
void getVNA(int ch);

void setT2enable(char *ch, char *val);
void getT2enable(int ch);

void SetTrig(char *ch, char *cmd);
void ReadTrigIn(void);

#endif
