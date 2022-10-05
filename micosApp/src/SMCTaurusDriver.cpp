#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SMCTaurusDriver.h"

SMCTaurusController::SMCTaurusController(const char *portName, const char *asynName, int numAxes, double movingPollPeriod, double idlePollPeriod) 
    : asynMotorController(
            portName,
            numAxes, 
            NUM_SMCTAURUS_PARAMS,
            0,
            0,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
            1,
            0, 0)
{
    int axis;
    asynStatus status;
    SMCTaurusAxis *pAxis;
    static const char *functionName = "SMCTaurusController::SMCTaurusController";

    createParam(SMCTaurusLimitTypeString,  asynParamInt32, &SMCTaurusLimitType_);
    createParam(SMCTaurusLimit1TypeString, asynParamInt32, &SMCTaurusLimit1Type_);
    createParam(SMCTaurusLimit2TypeString, asynParamInt32, &SMCTaurusLimit2Type_);

    status = pasynOctetSyncIO->connect(asynName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to SMC Taurus controller\n", functionName);
    }
    for (axis = 0; axis<numAxes; axis++) {
        pAxis = new SMCTaurusAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

asynStatus SMCTaurusController::changeResolution(int axisNo, double newResolution)
{
    SMCTaurusAxis* pAxis;
    asynStatus status;

    pAxis  = this->getAxis(axisNo);
    status = pAxis->changeResolution(newResolution);
    return status;
}

void SMCTaurusController::report(FILE *fp, int level)
{
    fprintf(fp, "SMC Taurus motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
    asynMotorController::report(fp, level);
}

SMCTaurusAxis* SMCTaurusController::getAxis(asynUser *pasynUser)
{
    return static_cast<SMCTaurusAxis*>(asynMotorController::getAxis(pasynUser));
}

SMCTaurusAxis* SMCTaurusController::getAxis(int axisNo)
{
    return static_cast<SMCTaurusAxis*>(asynMotorController::getAxis(axisNo));
}

/** Creates a new SMCTaurusController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCTaurusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC Taurus controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMCTaurusCreateController(const char *portName, const char *SMCTaurusPortName, int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    SMCTaurusController *pSMCTaurusController = new SMCTaurusController(portName, SMCTaurusPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    pSMCTaurusController = NULL;
    return(asynSuccess);
}

/** Specify a new resolution for an SMC Taurus axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMCTaurusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC Taurus controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMCTaurusChangeResolution(const char *SMCTaurusPortName, int axisNo, double newResolution)
{
    SMCTaurusController *pC;
    static const char *functionName = "SMCTaurusChangeResolution";
  
    pC = (SMCTaurusController*) findAsynPortDriver(SMCTaurusPortName);
    if (!pC) {
        printf("SMCTaurusDriver.cpp:%s: Error port %s not found\n", functionName, SMCTaurusPortName);
        return asynError;
    }
  
    pC->lock();
    pC->changeResolution(axisNo, newResolution);
    pC->unlock();
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SMCTaurusCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCTaurusCreateControllerArg1 = {"SMC Taurus port name", iocshArgString};
static const iocshArg SMCTaurusCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMCTaurusCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCTaurusCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMCTaurusCreateControllerArgs[] = {&SMCTaurusCreateControllerArg0,
                                                             &SMCTaurusCreateControllerArg1,
                                                             &SMCTaurusCreateControllerArg2,
                                                             &SMCTaurusCreateControllerArg3,
                                                             &SMCTaurusCreateControllerArg4};
static const iocshFuncDef SMCTaurusCreateControllerDef = {"SMCTaurusCreateController", 5, SMCTaurusCreateControllerArgs};
static void SMCTaurusCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCTaurusCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg SMCTaurusChangeResolutionArg0 = {"SMC Taurus port name", iocshArgString};
static const iocshArg SMCTaurusChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMCTaurusChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMCTaurusChangeResolutionArgs[] = {&SMCTaurusChangeResolutionArg0,
                                                             &SMCTaurusChangeResolutionArg1,
                                                             &SMCTaurusChangeResolutionArg2};
static const iocshFuncDef SMCTaurusChangeResolutionDef = {"SMCTaurusChangeResolution", 3, SMCTaurusChangeResolutionArgs};
static void SMCTaurusChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMCTaurusChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMCTaurusRegister(void)
{
  iocshRegister(&SMCTaurusCreateControllerDef, SMCTaurusCreateControllerCallFunc);
  iocshRegister(&SMCTaurusChangeResolutionDef, SMCTaurusChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMCTaurusRegister);
}

