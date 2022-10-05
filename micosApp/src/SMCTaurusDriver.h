#ifndef __SMC_TAURUS__
#define __SMC_TAURUS__

#include "SMCTaurusAxis.h"

#define NUM_SMCTAURUS_PARAMS        3
#define MAX_SMCTAURUS_AXIS          1
#define SMCTaurusLimitTypeString    "SMCTAURUS_SET_LIMIT_TYPE"
#define SMCTaurusLimit1TypeString   "SMCTAURUS_LIMIT1_TYPE"
#define SMCTaurusLimit2TypeString   "SMCTAURUS_LIMIT2_TYPE"

class epicsShareClass SMCTaurusController : public asynMotorController
{
public:
    SMCTaurusController(const char *portName, const char *asynControllerName, int numAxes, double movingPollPeriod, double idlePollPeriod);
    
    void report(FILE *fp, int level);
    SMCTaurusAxis* getAxis(asynUser *pasynUser);
    SMCTaurusAxis* getAxis(int axisNo);
    asynStatus changeResolution(int axisNo, double newResolution);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus readInt32(asynUser *pasynUser, epicsInt32* value);

protected:
    int SMCTaurusLimitType_;       /* Controller Limit Switch type */
    int SMCTaurusLimit1Type_;
    int SMCTaurusLimit2Type_;
 
friend class SMCTaurusAxis;
};

#endif

