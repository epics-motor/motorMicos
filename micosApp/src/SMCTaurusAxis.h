#ifndef __SMC_TAURUS_AXIS__
#define __SMC_TAURUS_AXIS__

#include <cstdlib>
#include <cmath>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass SMCTaurusAxis : public asynMotorAxis
{
public:
    SMCTaurusAxis(class SMCTaurusController* _controller, int axis);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus setHighLimit(double highLimit);
    asynStatus setLowLimit(double lowLimit);
    asynStatus changeResolution(double newResolution);
    asynStatus setLimitSwitches(int value);
    asynStatus getLimitSwitches(int limitIndex, int* value);
    
    void report(FILE *fp, int level);

private:
    SMCTaurusController* controller_;
    asynStatus sendAccelAndVelocity(double accel, double velocity);
    int motorForm_;
    int polePairs_;
    int lowLimitConfig_;
    int highLimitConfig_;
    double pitch_;
    double clPeriod_;
    double axisRes_;
    double mres_;
    double posTravelLimit_;
    double negTravelLimit_;
  
friend class SMCTaurusController;
};

#endif

