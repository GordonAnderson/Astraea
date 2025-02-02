#include "Calibration.h"

void CalibrateLoop(void)
{
  ProcessSerial(false);
  //control.run();
}

int Calibrate5592point(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float *V)
{
  char   *Token;
  String sToken;

  // Set value and ask for user to enter actual value read
  if(dacchan !=NULL) AD5592writeDAC(SPIcs, dacchan->Chan, Value2Counts(*V,dacchan));
  serial->print("Enter actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  *V = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
  if(adcchan != NULL) return AD5592readADC(SPIcs, adcchan->Chan, 10); 
  return 0; 
}

// This function is used to calibrate ADC/DAC AD5592 channels. 
void Calibrate5592(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float V1, float V2)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;
  int    dacV1, dacV2;

  serial->println("Enter values when prompted.");
  // Set to first voltage and ask for user to enter actual voltage
  val1 = V1;
  adcV1 = Calibrate5592point(SPIcs, dacchan, adcchan, &val1);
  // Set to second voltage and ask for user to enter actual voltage
  val2 = V2;
  adcV2 = Calibrate5592point(SPIcs, dacchan, adcchan, &val2);
  // Calculate calibration parameters and apply
  dacV1 = Value2Counts(V1, dacchan);
  dacV2 = Value2Counts(V2, dacchan);
  m = (float)(dacV2-dacV1) / (val2-val1);
  b = (float)dacV1 - val1 * m;
  serial->println("DAC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  dacchan->m = m;
  dacchan->b = b;
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  adcchan->m = m;
  adcchan->b = b;
}

void CalibrateVP(int ch)
{
  if((ch<1) || (ch>2)) BADARG;
  serial->print("Calibrate Vpos output: "); serial->print(ch); serial->println(", monitor with a voltmeter.");
  ch--;
  SPI.beginTransaction(SPISettings(SPISLOWCLK, MSBFIRST, SPI_MODE2));
  Calibrate5592(VCS, &astraea.VOPCtrl[ch], &astraea.VOPMon[ch], 10.0, 75.0);
  AD5592writeDAC(VCS, astraea.VOPCtrl[ch].Chan, Value2Counts(astraea.VP[ch],&astraea.VOPCtrl[ch]));
  SPI.beginTransaction(SPISettings(SPIFASTCLK, MSBFIRST, SPI_MODE0));
}

void CalibrateVN(int ch)
{
  if((ch<1) || (ch>2)) BADARG;
  serial->print("Calibrate Vneg output: "); serial->print(ch); serial->println(", monitor with a voltmeter.");
  ch--;
  SPI.beginTransaction(SPISettings(SPISLOWCLK, MSBFIRST, SPI_MODE2));
  Calibrate5592(VCS, &astraea.VONCtrl[ch], &astraea.VONMon[ch], -10.0, -75.0);
  AD5592writeDAC(VCS, astraea.VONCtrl[ch].Chan, Value2Counts(astraea.VN[ch],&astraea.VONCtrl[ch]));
  SPI.beginTransaction(SPISettings(SPIFASTCLK, MSBFIRST, SPI_MODE0));
}
