/*
FILENAME... SMChydraDriver.cpp
USAGE...    Motor driver support for the Micos SMC hydra controller.

Note: This driver was tested with the Micos SMC hydra CM and 
      motor forms 0 (stepper) and 1 (linear).

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SMChydraDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SMChydraController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMChydraPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC hydra controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SMChydraController::SMChydraController(const char *portName, const char *SMChydraPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_SMCHYDRA_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SMChydraAxis *pAxis;
  static const char *functionName = "SMChydraController::SMChydraController";

  // Create controller-specific parameters
  createParam(SMChydraRegulatorModeString, asynParamInt32, &SMChydraRegulatorMode_);

  /* Connect to SMC hydra controller */
  status = pasynOctetSyncIO->connect(SMChydraPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SMC hydra controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new SMChydraAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Change the resolution of an axis
  * \param[in] axisNo The index of the axis
  * \param[in] newResolution The new resolution
  */
asynStatus SMChydraController::changeResolution(int axisNo, double newResolution)
{
  SMChydraAxis* pAxis;
  asynStatus status;
  
  pAxis = this->getAxis(axisNo);
  
  status = pAxis->changeResolution(newResolution);
  
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SMChydraController::report(FILE *fp, int level)
{
  fprintf(fp, "SMC hydra motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SMChydraAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SMChydraAxis* SMChydraController::getAxis(asynUser *pasynUser)
{
  return static_cast<SMChydraAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SMChydraAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SMChydraAxis* SMChydraController::getAxis(int axisNo)
{
  return static_cast<SMChydraAxis*>(asynMotorController::getAxis(axisNo));
}

/** Creates a new SMChydraController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMChydraPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC hydra controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMChydraCreateController(const char *portName, const char *SMChydraPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  SMChydraController *pSMChydraController
    = new SMChydraController(portName, SMChydraPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pSMChydraController = NULL;
  return(asynSuccess);
}

/** Specify a new resolution for an SMC hydra axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMChydraPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC hydra controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMChydraChangeResolution(const char *SMChydraPortName, int axisNo, double newResolution)
{
  SMChydraController *pC;
  static const char *functionName = "SMChydraChangeResolution";
  
  pC = (SMChydraController*) findAsynPortDriver(SMChydraPortName);
  if (!pC) {
    printf("SMChydraDriver.cpp:%s: Error port %s not found\n",
           functionName, SMChydraPortName);
    return asynError;
  }
  
  pC->lock();
  pC->changeResolution(axisNo, newResolution);
  pC->unlock();
  
  return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SMChydraCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMChydraCreateControllerArg1 = {"SMC hydra port name", iocshArgString};
static const iocshArg SMChydraCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMChydraCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMChydraCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMChydraCreateControllerArgs[] = {&SMChydraCreateControllerArg0,
                                                             &SMChydraCreateControllerArg1,
                                                             &SMChydraCreateControllerArg2,
                                                             &SMChydraCreateControllerArg3,
                                                             &SMChydraCreateControllerArg4};
static const iocshFuncDef SMChydraCreateControllerDef = {"SMChydraCreateController", 5, SMChydraCreateControllerArgs};
static void SMChydraCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMChydraCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg SMChydraChangeResolutionArg0 = {"SMC hydra port name", iocshArgString};
static const iocshArg SMChydraChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMChydraChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMChydraChangeResolutionArgs[] = {&SMChydraChangeResolutionArg0,
                                                             &SMChydraChangeResolutionArg1,
                                                             &SMChydraChangeResolutionArg2};
static const iocshFuncDef SMChydraChangeResolutionDef = {"SMChydraChangeResolution", 3, SMChydraChangeResolutionArgs};
static void SMChydraChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMChydraChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMChydraRegister(void)
{
  iocshRegister(&SMChydraCreateControllerDef, SMChydraCreateControllerCallFunc);
  iocshRegister(&SMChydraChangeResolutionDef, SMChydraChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMChydraRegister);
}
