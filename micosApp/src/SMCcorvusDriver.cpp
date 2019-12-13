/*
FILENAME... SMCcorvusDriver.cpp
USAGE...    Motor driver support for the Micos SMC corvus controller.

Note: This driver was tested with the Micos SMC corvus eco with three stepper motors.

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "SMCcorvusDriver.h"

/** Creates a new SMCcorvusController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCcorvusPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC corvus controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SMCcorvusController::SMCcorvusController(const char *portName, const char *SMCcorvusPortName, int numAxes, 
        double movingPollPeriod, double idlePollPeriod)
        : asynMotorController(portName, numAxes, NUM_SMCCORVUS_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
    int axis;
    asynStatus status;
    SMCcorvusAxis *pAxis;
    static const char *functionName = "SMCcorvusController::SMCcorvusController";
  
    /* Connect to SMC corvus controller */
    status = pasynOctetSyncIO->connect(SMCcorvusPortName, 0, &pasynUserController_, NULL);
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s: cannot connect to SMC corvus controller\n",
        functionName);
    }
    for (axis=0; axis<numAxes; axis++) {
      pAxis = new SMCcorvusAxis(this, axis);
    }
  
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new SMCcorvusController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCcorvusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC corvus controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMCcorvusCreateController(const char *portName, const char *SMCcorvusPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
    SMCcorvusController *pSMCcorvusController
        = new SMCcorvusController(portName, SMCcorvusPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    pSMCcorvusController = NULL;
    return(asynSuccess);
}


/** Specify a new resolution for an SMC corvus axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMCcorvusPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC corvus controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMCcorvusChangeResolution(const char *SMCcorvusPortName, int axisNo, double newResolution)
{
    SMCcorvusController *pC;
    static const char *functionName = "SMCcorvusChangeResolution";
    
    pC = (SMCcorvusController*) findAsynPortDriver(SMCcorvusPortName);
    if (!pC) {
      printf("SMCcorvusDriver.cpp:%s: Error port %s not found\n",
             functionName, SMCcorvusPortName);
      return asynError;
    }
    
    pC->lock();
    pC->changeResolution(axisNo, newResolution);
    pC->unlock();
    
    return(asynSuccess);
}

/** Change the resolution of an axis
  * \param[in] axisNo The index of the axis
  * \param[in] newResolution The new resolution
  */
asynStatus SMCcorvusController::changeResolution(int axisNo, double newResolution)
{
    SMCcorvusAxis* pAxis;
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
void SMCcorvusController::report(FILE *fp, int level)
{
    fprintf(fp, "SMC corvus motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
      this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
  
    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an SMCcorvusAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SMCcorvusAxis* SMCcorvusController::getAxis(asynUser *pasynUser)
{
    return static_cast<SMCcorvusAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SMCcorvusAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SMCcorvusAxis* SMCcorvusController::getAxis(int axisNo)
{
    return static_cast<SMCcorvusAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the SMCcorvusAxis methods

/** Creates a new SMCcorvusAxis object.
  * \param[in] pC Pointer to the SMCcorvusController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SMCcorvusAxis::SMCcorvusAxis(SMCcorvusController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC)
{  
    sprintf(pC_->outString_, "%i getpitch", (axisNo + 1));
    pC_->writeReadController();
    pitch_ = atof( (char *) &pC_->inString_ );
    
    sprintf(pC_->outString_, "%i getpolepairs", (axisNo + 1));
    pC_->writeReadController();
    polePairs_ = atoi( (char *) &pC_->inString_ );
  
    sprintf(pC_->outString_, "%i getclperiod", (axisNo + 1));
    pC_->writeReadController();
    clPeriod_ = atof( (char *) &pC_->inString_ );
  
    axisRes_ = pitch_ / (4.0 * polePairs_);
  
    // Determine the travel limits (will change after homing)
    sprintf(pC_->outString_, "%i getnlimit", (axisNo + 1));
    pC_->writeReadController();
    sscanf(pC_->inString_, "%lf %lf", &negTravelLimit_, &posTravelLimit_);

    callParamCallbacks();  
}
/** Change the axis resolution
  * \param[in] newResolution The new resolution
  */
asynStatus SMCcorvusAxis::changeResolution(double newResolution)
{
    axisRes_ = newResolution;
    return asynSuccess;
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SMCcorvusAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "    pitch %f\n", pitch_);
    fprintf(fp, "    polePairs %d\n", polePairs_);
    fprintf(fp, "    clPeriod %f\n", clPeriod_);
    fprintf(fp, "    axisRes %f\n", axisRes_);
    fprintf(fp, "    lowLimitConfig %d\n", lowLimitConfig_);
    fprintf(fp, "    highLimitConfig %d\n", highLimitConfig_);
    fprintf(fp, "    posTravelLimit %f\n", posTravelLimit_);
    fprintf(fp, "    negTravelLimit %f\n", negTravelLimit_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus SMCcorvusAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
    asynStatus status;
    // static const char *functionName = "SMCcorvusAxis::sendAccelAndVelocity";
  
    // The corvus does not allow setting the velocity and acceleration for individual axes.
    // These commands affect all axes. 
  
    // Send the velocity
    sprintf(pC_->outString_, "%f sv", fabs(velocity * axisRes_));
    status = pC_->writeController();
  
    // Send the acceleration
    // acceleration is in units/sec/sec
    sprintf(pC_->outString_, "%f sa", fabs(acceleration * axisRes_));
    status = pC_->writeController();
    return status;
}


asynStatus SMCcorvusAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
    // The corvus only supports a single, simultaneous move command for all axes.
    // The workaround is to:
    //   i) check the current position for all axes, 
    //   ii) set the target to the current position for the axes we don't want to move,
    //   iii) set the target to the desired position for the axis we do want to move.

    asynStatus status;
    // static const char *functionName = "SMCcorvusAxis::move";
    double pos1, pos2, pos3;
    const char* movecmd;
    
    movecmd = relative ? "rmove":"move";
  
    status = sendAccelAndVelocity(acceleration, slewVelocity);
   
    sprintf(pC_->outString_, "pos");
    status = pC_->writeReadController();
    if (pC_->numAxes_  == 1) {
        sscanf(pC_->inString_, "%lf", &pos1);
    } else if (pC_->numAxes_ == 2) {
        sscanf(pC_->inString_, "%lf %lf", &pos1, &pos2);
    } else if (pC_->numAxes_ == 3) {
        sscanf(pC_->inString_, "%lf %lf %lf", &pos1, &pos2, &pos3);
    }

    if (axisNo_ == 0) {
        sprintf(pC_->outString_, "%f %f %f %s", (position * axisRes_), pos2, pos3, movecmd);
    } else if (axisNo_ == 1) {
        sprintf(pC_->outString_, "%f %f %f %s", pos1, (position * axisRes_), pos3, movecmd);
    } else if (axisNo_ == 2) {
        sprintf(pC_->outString_, "%f %f %f %s", pos1, pos2, (position * axisRes_), movecmd);
    }
    
    status = pC_->writeController();
    return status;
}

asynStatus SMCcorvusAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
    asynStatus status;
    // static const char *functionName = "SMCcorvusAxis::home";
  
    status = sendAccelAndVelocity(acceleration, slewVelocity);
  
    if (forwards) {
      sprintf(pC_->outString_, "%i nrm", (axisNo_ + 1));
    } else {
      sprintf(pC_->outString_, "%i ncal", (axisNo_ + 1));
    }
    status = pC_->writeController();
    return status;
}

asynStatus SMCcorvusAxis::moveVelocity(double baseVelocity, double slewVelocity, double acceleration)
{
    asynStatus status;
    static const char *functionName = "SMCcorvusAxis::moveVelocity";
    double llm, hlm;

    pC_->getDoubleParam(axisNo_, pC_->motorLowLimit_, &llm);
    pC_->getDoubleParam(axisNo_, pC_->motorHighLimit_, &hlm);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
      "%s: baseVelocity=%f, slewVelocity=%f, acceleration=%f\n",
      functionName, baseVelocity, slewVelocity, acceleration);

    /* SMC corvus does not have a reliable jog command (the "speed" command crashes my controller).
     Move to a limit instead.*/
    if (slewVelocity > 0.) {
        status = move(hlm, 0, baseVelocity, slewVelocity, acceleration);
    } else {
        status = move(llm, 0, baseVelocity, slewVelocity, acceleration);
    }
    return status;

}

asynStatus SMCcorvusAxis::stop(double acceleration)
{

    // The corvus does not have a stop commnad for individual axes.
    // This command will stop all axes.

    asynStatus status;
    //static const char *functionName = "SMCcorvusAxis::stop";
  
    sprintf(pC_->outString_, "abort");
    status = pC_->writeController();
    return status;
}

asynStatus SMCcorvusAxis::setPosition(double position)
{
    asynStatus status;
    //static const char *functionName = "SMCcorvusAxis::setPosition";
    double pos1, pos2, pos3;

    sprintf(pC_->outString_, "pos");
    status = pC_->writeReadController();
    if (pC_->numAxes_ == 1) {
        sscanf(pC_->inString_, "%lf", &pos1);
    } else if (pC_->numAxes_ == 2) {
        sscanf(pC_->inString_, "%lf %lf", &pos1, &pos2);
    } else if (pC_->numAxes_ == 3) {
        sscanf(pC_->inString_, "%lf %lf %lf", &pos1, &pos2, &pos3);
    }
    
    // The argument to the setnpos command is the distance from the current position of the
    // desired origin, which is why the position needs to be multiplied by -1.0
    if (axisNo_ == 0) {
        sprintf(pC_->outString_, "%f %f %f setpos", (position * axisRes_ * -1.0), pos2, pos3);
    } else if (axisNo_ == 1) {
        sprintf(pC_->outString_, "%f %f %f setpos", pos1, (position * axisRes_ * -1.0), pos3);
    } else if (axisNo_ == 2) {
        sprintf(pC_->outString_, "%f %f %f setpos", pos1, pos2, (position * axisRes_ * -1.0));
    }
  
    status = pC_->writeController();
    return status;
}

asynStatus SMCcorvusAxis::setClosedLoop(bool closedLoop)
{
    asynStatus status = asynSuccess;
    //static const char *functionName = "SMCcorvusAxis::setClosedLoop";
  
    // enable closed-loop control
    sprintf(pC_->outString_, "%i %i setcloop", closedLoop ? 1:0, (axisNo_ + 1));
    status = pC_->writeController();
  
    return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SMCcorvusAxis::poll(bool *moving)
{ 
    int done;
    int driveOn;
    int lowLimit;
    int highLimit;
    int ignoreLowLimit;
    int ignoreHighLimit;
    int controllerStatus=-1;
    double position=0.0;
    asynStatus comStatus;
    double pos1, pos2, pos3;
  
    //static const char *functionName = "SMCcorvusAxis::poll";
  
    // Read the current motor position
    // The corvus doesn't have an individual position command for each axis, so we
    // read all axes and set the parameter for the current axis.
    sprintf(pC_->outString_, "pos");
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    
    if (pC_->numAxes_ == 1) {
        sscanf(pC_->inString_, "%lf", &pos1);
    } else if (pC_->numAxes_ == 2) {
        sscanf(pC_->inString_, "%lf %lf", &pos1, &pos2);
    } else if (pC_->numAxes_ == 3) {
        sscanf(pC_->inString_, "%lf %lf %lf", &pos1, &pos2, &pos3);
    }
    
    if (axisNo_ == 0) {
        position = pos1;
    } else if (axisNo_ == 1) {
        position = pos2;
    } else if (axisNo_ == 2) {
        position = pos3;
    }
    setDoubleParam(pC_->motorPosition_, (position / axisRes_));
    setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_));
    
    
    // Read the controller status.  The corvus also does not have an 
    // individual status command for each axis, so all axes will show moving if one is moving.
    sprintf(pC_->outString_, "st");
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    controllerStatus = atoi((char *)&pC_->inString_);
    
    //// Check the moving bit
    done = !(controllerStatus & 0x1); 
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);
    *moving = done ? false:true;
  
    // Read the limit status
    // Note: calibration switch = low limit; range measure switch = high limit
    // also need to read the switch configuration to see if limits are ignored"
    // Don't poll the limit switches while moving, or we'll block the controller's interpreter
    if (done) {  
        // Read switch configuration
        // Bit 0:	polarity (0 = NO, 1 = NC)
        // Bit 1:	mask (0 = enabled, 1 = disabled)
        sprintf(pC_->outString_, "%i getsw", (axisNo_ + 1));
        comStatus = pC_->writeReadController();
        if (comStatus) goto skip;
        sscanf(pC_->inString_, "%i %i", &lowLimitConfig_, &highLimitConfig_);
        ignoreLowLimit = lowLimitConfig_ & 0x2;
        ignoreHighLimit = highLimitConfig_ & 0x2;
        
        // Read status of switches 0=inactive 1=active
        sprintf(pC_->outString_, "%i getswst", (axisNo_ + 1));
        comStatus = pC_->writeReadController();
        if (comStatus) goto skip;
        // The response string is of the form "0 0"
        sscanf(pC_->inString_, "%i %i", &lowLimit, &highLimit);
        //
        if (ignoreLowLimit)
          setIntegerParam(pC_->motorStatusLowLimit_, 0);
        else
          setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
      
        if (ignoreHighLimit)
          setIntegerParam(pC_->motorStatusHighLimit_, 0);
        else
          setIntegerParam(pC_->motorStatusHighLimit_, highLimit);
    
        // Check if motor drive current is enabled
        sprintf(pC_->outString_, "%i getmp", (axisNo_ + 1));
        comStatus = pC_->writeReadController();
        if (comStatus) goto skip;
        driveOn = atoi( (char *) &pC_->inString_);
        setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
        setIntegerParam(pC_->motorStatusProblem_, !driveOn);
    }
  
    skip:
    setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);

    callParamCallbacks();
    return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg SMCcorvusCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCcorvusCreateControllerArg1 = {"SMC corvus port name", iocshArgString};
static const iocshArg SMCcorvusCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMCcorvusCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCcorvusCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMCcorvusCreateControllerArgs[] = {&SMCcorvusCreateControllerArg0,
                                                             &SMCcorvusCreateControllerArg1,
                                                             &SMCcorvusCreateControllerArg2,
                                                             &SMCcorvusCreateControllerArg3,
                                                             &SMCcorvusCreateControllerArg4};
static const iocshFuncDef SMCcorvusCreateControllerDef = {"SMCcorvusCreateController", 5, SMCcorvusCreateControllerArgs};
static void SMCcorvusCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCcorvusCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg SMCcorvusChangeResolutionArg0 = {"SMC corvus port name", iocshArgString};
static const iocshArg SMCcorvusChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMCcorvusChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMCcorvusChangeResolutionArgs[] = {&SMCcorvusChangeResolutionArg0,
                                                             &SMCcorvusChangeResolutionArg1,
                                                             &SMCcorvusChangeResolutionArg2};
static const iocshFuncDef SMCcorvusChangeResolutionDef = {"SMCcorvusChangeResolution", 3, SMCcorvusChangeResolutionArgs};
static void SMCcorvusChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMCcorvusChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMCcorvusRegister(void)
{
  iocshRegister(&SMCcorvusCreateControllerDef, SMCcorvusCreateControllerCallFunc);
  iocshRegister(&SMCcorvusChangeResolutionDef, SMCcorvusChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMCcorvusRegister);
}
