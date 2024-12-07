#ifndef AGILITY_h
#define AGILITY_h
#include "Astraea.h"

#define MaxStates   10
#define MAXSTACK    5
#define MINNWFMS    2

// State transion table loop nesting stack.
typedef struct
{
  bool Inited;
  int  StartOfLoop;
  int  Count;
} StateStack;

extern char stateTable[];
extern volatile bool executing;
extern bool advanceScan;

// Prototypes
void defineState(void);
void clearStates(void);
void getNumStates(void);
void reportState(char *val);
void reportStates(void);
void applyState(char *val);
void executeStates(void);
void current2State(char *val, char *numWVF);
void abortStates(void);

void setStateMZcmd(char *val, char *mz);
void setAllStateMZcmd(char *mz);
void returnGlobalMZ(void);
void startStateScan(void);

#endif
