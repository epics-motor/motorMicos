/*
FILENAME...   SMChydraDriver.h
USAGE...      Motor driver support for the Micos SMC hydra controller.

*/

#ifndef __SMCHYDRA__
#define __SMCHYDRA__

#include "SMChydraAxis.h"

#define MAX_SMCHYDRA_AXES 2

// Controller-specific parameters
#define NUM_SMCHYDRA_PARAMS 1

/** drvInfo strings for extra parameters that the SMC Hydra controller supports */
#define SMChydraRegulatorModeString "SMCHYDRA_REGULATOR_MODE"

class epicsShareClass SMChydraController : public asynMotorController {
public:
  SMChydraController(const char *portName, const char *SMChydraPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  SMChydraAxis* getAxis(asynUser *pasynUser);
  SMChydraAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);

protected:
  int SMChydraRegulatorMode_;    /** Regulator mode parameter index */

friend class SMChydraAxis;
};

#endif

