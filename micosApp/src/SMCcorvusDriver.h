/*
FILENAME...   SMCcorvusDriver.h
USAGE...      Motor driver support for the Micos SMC corvus controller.

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_SMCCORVUS_AXES 3

// No controller-specific parameters yet
#define NUM_SMCCORVUS_PARAMS 0  

class epicsShareClass SMCcorvusAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SMCcorvusAxis(class SMCcorvusController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus changeResolution(double newResolution);

private:
  SMCcorvusController *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  int motorForm_;
  double pitch_;
  int polePairs_;
  double clPeriod_;
  double axisRes_;
  double mres_;
  int lowLimitConfig_;
  int highLimitConfig_;
  double posTravelLimit_;
  double negTravelLimit_;
  
friend class SMCcorvusController;
};

class epicsShareClass SMCcorvusController : public asynMotorController {
public:
  SMCcorvusController(const char *portName, const char *SMCcorvusPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  SMCcorvusAxis* getAxis(asynUser *pasynUser);
  SMCcorvusAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);

friend class SMCcorvusAxis;
};
