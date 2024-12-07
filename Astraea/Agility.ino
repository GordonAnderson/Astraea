#include <Arduino.h>
#include "Astraea.h"
#include "AD9959.h"
#include "Hardware.h"
#include "Agility.h"

// The agility capability and the use of the state tables provides the most flexibility for the
// Astraea system. To use this capability you first define a set of states for the quads supported
// by Astraea, then you define the order you wish to execute the states using the state table.
// When scanning using the states you define a state table that represents the states for a single
// point in the scan. Lets assue you wish to perform a trap, filter, eject and detect process and you
// want to do this 10 times at each point in the scan. You would first define the states, we will
// name them t,f,e, and d. Next you build the state table:
// [tfeCd]10R
// This table will run the trap filter eject states then collect data during the detect state, repeat
// this process 10 times and then report the results.
// The scan functions in Astraea will set the m/z and advance its value through the scan.

// This design needs to change as follows:
// - m/z is a global value that is used in all states
// - Remove m/z from state and add quad channel
// - Define global scan values
//    - Start m/z
//    - End m/z
//    - Number of steps
//    - Number of scans
// - Add additional state sequence string functions:
//    - M = set m/z value
//    - C = Collect data during the next quad state
//    - R = Report data
// State commands block the advance of the sequence. If a state is not updated after its
// number of waveforms has completed it will just continue until updated.
// Add command to set all state m/z values, basically calculate the frequency from m/z for each state.
// Scanning will happen by a sequence that runs one time at each scan point. The scanning 
// script will set the m/z, run the script, collect the data, and repeat until done.

// State commands:
// - Clear all states
// - Enter state, mz,t1,t3,num or freq,t1,t3,num
// - Get number of states
// - Display state
// - Apply state
// - Edit a state
// - Name states lower case a thru j, matches index 0 thru 9
// - State sequence string
//    abca10[aca]10aD10, [] for looping, D = delay in mS, count follows command implied count of 1
//
// - Allow external trigger of sequence table
//  - Develop a state send function
//  - Update FPGA to load the DDS and FPGA after n cycles and then generate an interrupt

// Agility parameters
double      globalMZ = 1000.0;
DQUAD       Aquad;
volatile bool        executing = false;
volatile bool        collectFlag = false;
volatile bool        reportPoint = false;
volatile bool        waitForHostReply = false;
volatile bool        abortStateScan = false;

IntervalTimer delayTimer;

int        stateStackLevel;
StateStack stateStack[MAXSTACK];

void stateStackInit(void) { stateStackLevel = 0; }

bool stateLoopStart(int Index)
{
  // If stack is full exit and return false
  if(stateStackLevel >= MAXSTACK) return false;
  // Init the stack entry
  stateStackLevel++;
  stateStack[stateStackLevel - 1].StartOfLoop = Index;
  stateStack[stateStackLevel - 1].Inited = false;
  return true;
}

int stateProcessLoop(int Count)
{
  if(stateStackLevel <= 0) return -1;
  if(stateStack[stateStackLevel - 1].Inited == false) stateStack[stateStackLevel - 1].Count = Count;
  stateStack[stateStackLevel - 1].Inited = true;
  stateStack[stateStackLevel - 1].Count--;
  if(stateStack[stateStackLevel - 1].Count == 0) 
  {
    stateStackLevel--;
    return -1;
  }
  return stateStack[stateStackLevel - 1].StartOfLoop;
}

double mz2freq(double mz, QuadState *qs)
{
  double Ro = astraea.quads[qs->ch].ro/1000.0;
  double c = sqrt(((9.775e6) * astraea.quads[qs->ch].Vop)/(qs->q * Ro * Ro));
  return(c * sqrt(1.0/mz));
}

int findState(char name)
{
  for(int i=0;i<astraea.NumStates;i++) 
  {
    if(astraea.qs[i].name == name) return(i);
  }
  return -1;
}

// This function calculates the state parameters. It is assumed the following paramaters
// are defined in the QuadState when called:
// mz or freq, if non zero assumed its defined
// q
// t1
// t3
// ch
bool prepairState(QuadState *qs)
{
   if((qs->ch != 0)&&(qs->ch != 1)) return false;
   if((qs->mz == 0)&&(qs->freq == 0)) return false;
   Aquad = astraea.quads[qs->ch];
   Aquad.freq = qs->freq;
   Aquad.mz   = qs->mz;
   if(qs->mz == 0)   qs->mz   = freq2mz(&Aquad);
   if(qs->freq == 0) qs->freq = mz2freq(&Aquad);
   Aquad.q  = qs->q;
   Aquad.t1 = qs->t1;
   Aquad.t3 = qs->t3;
   Aquad.t2 = 100.0 - qs->t1 - qs->t3;
   // Calculate the hardware parameters
   findModulus(qs->freq, &Aquad);
   configureModulus(Aquad.modulus, &Aquad,true);
   mz2freq(&Aquad);
   // Move parameters to state structure
   qs->DDSchannel     = Aquad.DDSchannel;
   qs->DDSmodChannel  = Aquad.DDSmodChannel;
   qs->DDSfreq = dds->calculateFWT(Aquad.DDSfreq);
   qs->Phase1 = Aquad.DDSoffset1;
   qs->Phase2 = Aquad.DDSoffset2;
   qs->PWM = Aquad.PWM;  
   return true;
}

bool setStateMZ(char stateName, double mz)
{
  int index = findState(stateName);
  if(index <= 0) return false;
  astraea.qs[index].mz = mz;
  return true;
}

void setAllStateMZ(double mz)
{
  for(int i=0;i<astraea.NumStates;i++) astraea.qs[i].mz = mz;
}

bool calculateStateFreq(char stateName)
{
  int index = findState(stateName);
  if(index <= 0) return false;
  astraea.qs[index].freq = mz2freq(astraea.qs[index].mz, &astraea.qs[index]);
  astraea.qs[index].DDSfreq = dds->calculateFWT(astraea.qs[index].freq * astraea.qs[index].PWM.mod);
  return true;  
}

void calculateAllStateFreq(void)
{
  for(int i=0;i<astraea.NumStates;i++)
  {
    astraea.qs[i].freq = mz2freq(astraea.qs[i].mz, &astraea.qs[i]);
    astraea.qs[i].DDSfreq = dds->calculateFWT(astraea.qs[i].freq * astraea.qs[i].PWM.mod);    
  }
}

void writeState(QuadState *qs)
{
  // Write frequency
  dds->setChannels((ChannelNum)(qs->DDSchannel | qs->DDSmodChannel));
  dds->write(CFTW, qs->DDSfreq);
  // Write phase
  dds->setPhase(qs->DDSmodChannel,PHASEOFFSETRANGE - qs->Phase1);
  if(qs->Phase2 == 0) dds->write(0x0A, 0);
  else dds->write(0x0A, (PHASEOFFSETRANGE - qs->Phase2) << 18);
  // Write PWM
  PWMupdate(&qs->PWM);
  // Write waveform count
  if(qs->ch > 0) astraea.scan.cfg = (1 << ScanWFcount) | (1 << ScanChan);
  else astraea.scan.cfg = 1 << ScanWFcount;
  digitalWrite(SCS,LOW);
  SPI.transfer(SCANadd);
  SPI.transfer(astraea.scan.cfg);
  SPI.transfer(qs->NumWaveforms & 0xFF);
  SPI.transfer((qs->NumWaveforms>>8) & 0xFF);
  //SPI.endTransaction();
  digitalWrite(SCS,HIGH);  
}

// This interrupt happens at the end of the number of requested waveforms. This
// ISR will first latch the next state that was loaded into the registers on the
// previous interrupt, then the next state will be loaded into the registers and
// this ISR assigned to process.
// If the acquire data flag is set then ISR will not exit until the next interrupt
// is detected and it will acquire data during the time it is waiting for the ISR.
volatile bool collecting = false;
 
void stateChangeISR(void)
{
   char state;
   int  i;

   Pulse(LOAD);
   dds->update();
   detachInterrupt(digitalPinToInterrupt(INTRP));
   state = getNextState(false);
   if((state>='a') && (state<='z'))
   {
      i = findState(state);
      if(i != -1) writeState(&astraea.qs[i]);
      attachInterrupt(digitalPinToInterrupt(INTRP),stateChangeISR,RISING);
      if(collectFlag) collecting = true;
   }
   if(state == 0) executing = false;
   if(collectFlag) collectFlag = false;
   else collecting = false;
}

void triggerStateTable(void)
{
  dds->enableModulation(astraea.quads[0].DDSmodChannel, true);
  executing = true;
  detachInterrupt(digitalPinToInterrupt(INTRP));
  getNextState(true);
  stateChangeISR();
}

void stopStateTable(void)
{
  if(!executing) return;
  executing = false;
  detachInterrupt(digitalPinToInterrupt(INTRP));
  detachInterrupt(TrigIn);
  delayTimer.end();
}

void stateDelayISR(void)
{
  delayTimer.end();
  if(!executing) return;
  stateChangeISR();
}

void stateWaitISR(void)
{
  detachInterrupt(TrigIn);
  if(!executing) return;
  stateChangeISR();
}

// Valid state table commands:
// a thru z, define a state to be set. The states used must be defined
// D, delays in milliseconds
// W, wait for trigger input rising edge
// T, generate output trigger pulse on Trig2
// C, collect flag, collect data on next event
// R, report data and clear the accumulator
// H, set flag to wait for host reply on report
// [, start of a loop 
// ], end of loop
// All commands can be followed by a value, if no value is defined then it
// default value of 1 is used.
//
char getNextState(bool init)
{
  bool   valfound;
  float  fval;
  static int  tblindex=0;
  static char OP;
  static int  count = 0;

  if(init)
  {
    stateStackInit();
    tblindex = 0;
    count    = 0;
    return(0);
  }
  if(count > 0)
  {
    count--;
    return(OP);
  }
  while(true)
  {
    // Find a valid character
    while(true)
    {
      if(astraea.stateTable[tblindex] == 0) return(0);
      OP = astraea.stateTable[tblindex++];
      count = 1;  // Default to count of 1
      valfound = false;
      if(isDigit(astraea.stateTable[tblindex]))
      {
        // If here then get the value, number can be a float but has to start with a number, 0.1 is ok, .1 is not ok
        count = int(astraea.stateTable[tblindex++] - '0');
        while(isDigit(astraea.stateTable[tblindex])) count = count * 10 + int(astraea.stateTable[tblindex++] - '0');
        // If its a decimal point then its a float so process
        fval = 0;
        if(astraea.stateTable[tblindex] == '.')
        {
          tblindex++;
          for(float d = 10; isDigit(astraea.stateTable[tblindex]); d *= 10) fval += (float)(astraea.stateTable[tblindex++] - '0') / d;
        }
        fval += count;
        valfound = true;
      }
      break;
    }
    // Here with character to process and count
    if((OP >= 'a') && (OP <= 'z'))
    {
      count--;
      return(OP);
    }
    else if(OP == 'D')  // Delay in millisec
    {
      // Start interval timer that will call delay event after 
      // defined delay in millisec
      delayTimer.begin(stateDelayISR, count * 1000); 
      count = 0;
      return(OP);
    }
    else if(OP == 'T')  // Generate output trigger pulse, Trig2, count defined width in uS
    {
      digitalWrite(Trig2,LOW);
      delayMicroseconds(count);
      digitalWrite(Trig2,HIGH);
      count = 0;
    }
    else if(OP == 'W') // Wait for input trigger pulse in TrigIn
    {
      count = 0;
      // Setup interrupt on trigger input signal
      attachInterrupt(TrigIn, stateWaitISR, FALLING);
      return(OP);
    }
    else if(OP == 'C') 
    {
      collectFlag = true;
      count = 0;
    }
    else if(OP == 'R')
    {
      // The reporting and clearing of the accumulator needs to be done
      // in the scan processing loop. Here we just set a report flag
      reportPoint = true;
      count = 0;
    }
    else if(OP == 'H')
    {
      waitForHostReply = true;
      count = 0;
      return(OP);
    }
    else if(OP == '[') stateLoopStart(tblindex);
    else if(OP == ']')
    {
      int i = stateProcessLoop(count);
      if(i != -1) tblindex = i;
    }    
  }
  count = 0;
  return(0);
}

// This function is called to start a scan using the state system. 
void stateScan(void)
{
  char ch;
  uint32_t now;
  static bool busy = false;

  if(busy) return;
  busy = true;
  // Initialize the scan and set the start mz
  scnStep = (scnStop - scnStart) / (double) (scanSteps-1);
  collectFlag = false;
  scnCurrent = scnStart;
  reportPoint = false;
  waitForHostReply = false;
  abortStateScan = false;
  // Prepair all the states using the lower of scnStop and scnStart
  double pMZ = scnStart;
  if(scnStop < pMZ) pMZ = scnStop;
  for(int i=0;i<astraea.NumStates;i++)
  {
    astraea.qs[i].mz = pMZ;
    astraea.qs[i].freq = 0;
    prepairState(&astraea.qs[i]);
  }
  // Loop for all the steps in the scan, execute the table at each point in the scan.
  // If the report flag is set then report the data to the host computer. If the waitForHost
  // flag is set then do not continue until the host sends and advance command.
  for(int scn = 0; scn < scanSteps; scn++)
  {
    if(abortStateScan) break;
    SPI.beginTransaction(SPISettings(SPIFASTCLK, MSBFIRST, SPI_MODE0));
    // Set all states mz
    setAllStateMZ(scnCurrent);
    // Calculate all states freq
    calculateAllStateFreq();
    // Start the state table executing
    astraea.scan.cfg = 1 << ScanWFcount;
    SCANupdate(&astraea.scan);
    triggerStateTable();
    // While executing look for report command
    while(executing)
    {
      while(collecting)
      {
        // Read ADC and accumulate the results, count the number of readings
        ADCsum += adc->adc0->analogReadContinuous();
        ADCcount++;
      }
      // If report flag is set send the host the data and clear the registers 
      if(reportPoint)
      {
        serial->print(1 + scn);
        serial->print(",");
        serial->print(scnCurrent,3);
        serial->print(",");
        serial->println((float)ADCsum/(float)ADCcount,3);
        //serial->println(ADCcount);
        serial->flush();
        ADCsum   = 0;
        ADCcount = 0;
        reportPoint = false;
        // If the wait for host flag is set wait for a message from the host to proceed
        if(waitForHostReply)
        {
          now = millis();
          while(true)
          {
            if(serial->available() > 0)
            {
              ch = serial->read();
              if(ch == 'n') break;
              else PutCh(ch);
            }
            if((millis() - now) > SCANTIMEOUT)
            {
              // We timed out so abort the scan
              //ScanAbort(false);
              abortStateScan = true;
              break;
            }
          }
          waitForHostReply = false;
        }
        // Continue executing the table 
        stateChangeISR();
      }
      ProcessSerial();  // added 9/11/23
    }
    // Calculate the mz for next point
    scnCurrent += scnStep;
    ProcessSerial();
  }
  busy = false;
}

// Host commands

// This function will define a state. The user input arguments are read from the
// input ring buffer. The user defines a state name and it must be a thur z, lower
// case. A maximum of NumSates can be defined.
//
// Read the following from the input serial ring buffer
// name,ch,q,t1,t3,numWaveforms
void defineState(void)
{
  char      *Token,name;
  QuadState nqs;
  int       i;
  
  while(true)
  {
  // Get name
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    name = Token[0];
    // Validate the name
    if((name < 'a') || (name > 'z')) break;
    // If this name is in use we will overwrite it, if its not
    // inuse make sure we have room for a new entry
    if((i=findState(name)) != -1)
    {
      if((astraea.NumStates + 1) >= MaxStates) break;
    }
    nqs.name = name;
  // Get QUAD channel 1 or 2
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sscanf(Token,"%d",&nqs.ch);
    if(nqs.ch < 1) break;
    if(nqs.ch > 2) break;
    nqs.ch--;         // make it into an index
  // Get q
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sscanf(Token,"%lf",&nqs.q);
    if(nqs.q < MINQ) break;
    if(nqs.q > MAXQ) break;
  // Get t1
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sscanf(Token,"%lf",&nqs.t1);
    if(nqs.t1 < MINT) break;
    if(nqs.t1 > MAXT) break;
  // Get t3    
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sscanf(Token,"%lf",&nqs.t3);
    if(nqs.t3 < MINT) break;
    if(nqs.t3 > MAXT) break;
  // Get numWaveforms    
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sscanf(Token,"%d",&nqs.NumWaveforms);
    if(nqs.NumWaveforms < MINNWFMS) break;
  // Prepair the state
    nqs.freq = 0;
    nqs.mz = globalMZ;
    if(!prepairState(&nqs)) break;
  // Success, clean up and exit
    if(i!=-1) astraea.qs[i] = nqs;
    else {astraea.qs[astraea.NumStates] = nqs; astraea.NumStates++;}
    SendACK;
    return;
  }
  // Here with an error, clear the ring buffer and exit with a NAK!
  while ((Token = GetToken(true)) != NULL);
  BADARG;
}

// Val = state name
// numWVF = number of waveforms
void current2State(char *val, char *numWVF)
{
  String token;
  
  token = numWVF;
  if(token.toInt() < MINNWFMS) BADARG;
  int i = findState(val[0]);
  if(i == -1)
  {
    if((astraea.NumStates + 1) >= MaxStates) BADARG;
    i = astraea.NumStates;
    astraea.NumStates++;
  }
  SendACK;
  Aquad = astraea.quads[0];
  astraea.qs[i].name = val[0];
  astraea.qs[i].mz = Aquad.mz;
  astraea.qs[i].freq = 0;
  astraea.qs[i].q  = Aquad.q;
  astraea.qs[i].t1 = Aquad.t1;
  astraea.qs[i].t3 = Aquad.t3;
  astraea.qs[i].ch = 0;
  astraea.qs[i].NumWaveforms = token.toInt();
  astraea.qs[i].DDSchannel = Aquad.DDSchannel;
  astraea.qs[i].DDSmodChannel = Aquad.DDSmodChannel;
  astraea.qs[i].Phase1 = Aquad.DDSoffset1;
  astraea.qs[i].Phase2 = Aquad.DDSoffset2;
  astraea.qs[i].PWM = Aquad.PWM;
  prepairState(&astraea.qs[i]);
}

void clearStates(void)
{
  astraea.NumStates = 0;
  SendACK;
}

void getNumStates(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(astraea.NumStates);
}

// report: name,ch,q,t1,t3,numWaveforms
void reportState(char *val)
{
  int i = findState(val[0]);
  if(i == -1) BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->print(astraea.qs[i].name); serial->print(",");
  serial->print(astraea.qs[i].ch+1); serial->print(",");
  serial->print(astraea.qs[i].q);    serial->print(",");
  serial->print(astraea.qs[i].t1);   serial->print(",");
  serial->print(astraea.qs[i].t3);   serial->print(",");
  serial->println(astraea.qs[i].NumWaveforms);
}

void reportStates(void)
{
  SendACKonly;
  if(SerialMute) return;
  for(int i=0;i<astraea.NumStates;i++)
  {
    serial->print(astraea.qs[i].name); serial->print(",");
    serial->print(astraea.qs[i].ch+1); serial->print(",");
    serial->print(astraea.qs[i].q);    serial->print(",");
    serial->print(astraea.qs[i].t1);   serial->print(",");
    serial->print(astraea.qs[i].t3);   serial->print(",");
    serial->println(astraea.qs[i].NumWaveforms);    
  }
}

void applyState(char *val)
{
  int i = findState(val[0]);
  if(i == -1) BADARG;
  SendACK;
  writeState(&astraea.qs[i]);
  Pulse(LOAD);
  dds->update();
}

void executeStates(void)
{
  SendACK;
  if(executing) return;
  SPI.beginTransaction(SPISettings(SPIFASTCLK, MSBFIRST, SPI_MODE0));
  astraea.scan.cfg = 1 << ScanWFcount;
  SCANupdate(&astraea.scan);
  triggerStateTable();
}

void abortStates(void)
{
  abortStateScan = true;
  SendACK;
  triggerStateTable();
  stopStateTable();
}

void setStateMZcmd(char *val, char *mz)
{
   double dval;
   
   sscanf(mz,"%lf",&dval);
   if(!setStateMZ(val[0], dval)) BADARG;
   SendACK;
   int index = findState(val[0]);
   if(index < 0) return;
   astraea.qs[index].freq = 0;
   prepairState(&astraea.qs[index]);
}

void setAllStateMZcmd(char *mz)
{
  double dval;

  SendACK;
  sscanf(mz,"%lf",&dval);
  globalMZ = dval;
  for(int i=0;i<astraea.NumStates;i++)
  {
    astraea.qs[i].mz = dval;
    astraea.qs[i].freq = 0;
    prepairState(&astraea.qs[i]);
  }  
}

void returnGlobalMZ(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(globalMZ,3);
}

void startStateScan(void)
{
  SendACK;
  stateScan();
}
