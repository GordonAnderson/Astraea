#ifndef Hardware_h
#define Hardware_h

typedef struct
{
  int8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
void Pulse(int DIO);

int   GetADCvalue(int chan, int num);
float ReadADCchannel(ADCchan adch, int num=20);
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);

void  AD5592write(int addr, uint8_t reg, uint16_t val);
int   AD5592readWord(int addr);
int   AD5592readADC(int addr, int8_t chan);
int   AD5592readADC(int addr, int8_t chan, int8_t num);
void  AD5592writeDAC(int addr, int8_t chan, int val);

void Software_Reset(void);
void bootLoader(void);
#endif
