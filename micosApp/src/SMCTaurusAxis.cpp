
#include "SMCTaurusDriver.h"

/** Creates a new SMCTaurusAxis object.
  * \param[in] controller Pointer to the SMCTaurusController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to controller->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SMCTaurusAxis::SMCTaurusAxis(SMCTaurusController* controller, int axisNo) 
    : asynMotorAxis(controller, axisNo),
      controller_(controller)
{
    sprintf(controller->outString_, "%i getclperiod", (axisNo + 1));
    controller->writeReadController();
    clPeriod_ = atof( (char *) &controller->inString_ );
    axisRes_ = clPeriod_;

    /* Enable gain support so that the CNEN field can be used to send
    the init command to clear a motor fault for stepper motors, even
    though they lack closed-loop support. */
    setIntegerParam(controller->motorStatusGainSupport_, 1);

    // Determine the travel limits (will change after homing)
    sprintf(controller->outString_, "%i getnlimit", (axisNo + 1));
    controller->writeReadController();
    sscanf(controller->inString_, "%lf %lf", &negTravelLimit_, &posTravelLimit_);
}

asynStatus SMCTaurusAxis::changeResolution(double newResolution)
{
    axisRes_ = newResolution;
    return asynSuccess;
}

void SMCTaurusAxis::report(FILE *fp, int level)
{
    if (level > 0) {
        fprintf(fp, "  axis %d\n", axisNo_);
        fprintf(fp, "    clPeriod %f\n", clPeriod_);
        fprintf(fp, "    axisRes %f\n", axisRes_);
        fprintf(fp, "    lowLimitConfig %d\n", lowLimitConfig_);
        fprintf(fp, "    highLimitConfig %d\n", highLimitConfig_);
        fprintf(fp, "    posTravelLimit %f\n", posTravelLimit_);
        fprintf(fp, "    negTravelLimit %f\n", negTravelLimit_);
    }

    asynMotorAxis::report(fp, level);
}

asynStatus SMCTaurusAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
    asynStatus status;

    // Send the velocity
    sprintf(controller_->outString_, "%f %i snv", fabs(velocity * axisRes_), (axisNo_ + 1));
    status = controller_->writeController();

    // Send the acceleration
    // acceleration is in units/sec/sec
    sprintf(controller_->outString_, "%f %i sna", fabs(acceleration * axisRes_), (axisNo_ + 1));
    status = controller_->writeController();
    return status;
}

asynStatus SMCTaurusAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
    asynStatus status;

    sendAccelAndVelocity(acceleration, slewVelocity); 
    if (relative) {
        sprintf(controller_->outString_, "%f %i nr", (position * axisRes_), (axisNo_ + 1));
    } else {
        sprintf(controller_->outString_, "%f %i nm", (position * axisRes_), (axisNo_ + 1));
    }
    status = controller_->writeController();
    return status;
}
    
asynStatus SMCTaurusAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
    asynStatus status;

    sendAccelAndVelocity(acceleration, slewVelocity);
    if (forwards) {
        sprintf(controller_->outString_, "%i nrm", (axisNo_ + 1));
    } else {
        sprintf(controller_->outString_, "%i ncal", (axisNo_ + 1));
    }
    status = controller_->writeController();
    return status;
}

asynStatus SMCTaurusAxis::moveVelocity(double baseVelocity, double slewVelocity, double acceleration)
{
    asynStatus status;

    static const char *functionName = "SMCTaurusAxis::moveVelocity";
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s: baseVelocity=%f, slewVelocity=%f, acceleration=%f\n", functionName, baseVelocity, slewVelocity, acceleration);
  
    /* SMC Taurus does not have jog command. Move to a limit*/
    if (slewVelocity > 0.) {
        status = sendAccelAndVelocity(acceleration, slewVelocity);
        sprintf(controller_->outString_, "%f %i nm", posTravelLimit_, (axisNo_ + 1));
    } else {
        status = sendAccelAndVelocity(acceleration, (slewVelocity * -1.0));
        sprintf(controller_->outString_, "%f %i nm", negTravelLimit_, (axisNo_ + 1));
    }
    status = controller_->writeController();
    return status;
}

asynStatus SMCTaurusAxis::stop(double acceleration )
{
    asynStatus status;

    sprintf(controller_->outString_, "%i nabort", (axisNo_ + 1));
    status = controller_->writeController();
    return status;
}

asynStatus SMCTaurusAxis::setPosition(double position)
{
    asynStatus status;

    // The argument to the setnpos command is the distance from the current position of the
    // desired origin, which is why the position needs to be multiplied by -1.0
    sprintf(controller_->outString_, "%f %i setnpos", (position * axisRes_ * -1.0), (axisNo_ + 1));
    status = controller_->writeController();
    return status;
}

asynStatus SMCTaurusAxis::setClosedLoop(bool closedLoop)
{
    asynStatus status = asynSuccess;
    if (closedLoop) {
        // enable closed-loop control
        sprintf(controller_->outString_, "%i %i setcloop", closedLoop, (axisNo_ + 1));
        status = controller_->writeController();

        // reinit so the closed-loop setting takes effect (this powers on the motor)
        sprintf(controller_->outString_, "%i init", (axisNo_ + 1));
        status = controller_->writeController();

        // a delay is required after the init command is sent
        epicsThreadSleep(0.2);
    } else {
        // disable closed-loop control
        sprintf(controller_->outString_, "%i motoroff", (axisNo_ + 1));
        status = controller_->writeController();
    }

    return status;
}

asynStatus SMCTaurusAxis::setHighLimit(double highLimit)
{
    posTravelLimit_ = highLimit;
    sprintf(controller_->outString_, "%f %f %i setnlimit", negTravelLimit_, posTravelLimit_, (axisNo_ + 1));
    controller_->writeController();
    return asynSuccess;
}

asynStatus SMCTaurusAxis::setLowLimit(double lowLimit)
{
    negTravelLimit_ = lowLimit;
    sprintf(controller_->outString_, "%f %f %i setnlimit", negTravelLimit_, posTravelLimit_, (axisNo_ + 1));
    controller_->writeController();
    return asynSuccess;
}

asynStatus SMCTaurusAxis::setLimitSwitches(int value)
{
    sprintf(controller_->outString_, "%i 0 %i setsw", value, (axisNo_ + 1));
    controller_->writeController();
    sprintf(controller_->outString_, "%i 1 %i setsw", value, (axisNo_ + 1));
    controller_->writeController();
    return asynSuccess;
}

asynStatus SMCTaurusAxis::getLimitSwitches(int limitIndex, int* value)
{
    int axis_1;
    int axis_2;
    sprintf(controller_->outString_, "%i getsw", (axisNo_ + 1));
    controller_->writeReadController();
    int status = sscanf(controller_->inString_, "%i %i", &axis_1, &axis_2);
    if(status != 2)
    {
        printf("Could not parse getsw response.\n");
        return asynError;
    }

    switch(limitIndex)
    {
        case 0:
            *value = axis_1;
            break;
        case 1:
            *value = axis_2;
            break;
        default:
            return asynError;
    }

    return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SMCTaurusAxis::poll(bool *moving)
{ 
    int done;
    int driveOn;
    int lowLimit;
    int highLimit;
    int ignoreLowLimit;
    int ignoreHighLimit;
    int axisStatus = -1;
    double position = 0.0;
    asynStatus comStatus;

    static const char *functionName = "SMCTaurusAxis::poll";

    // A different implementation for setting and reading current limit switches configuration.
    // int limit_switches;
    // controller_->getIntegerParam(controller_->SMCTaurusLimitType_, &limit_switches);
    // controller_->getAxis(0)->setLimitSwitches(limit_switches);

    // limit_switches = 0;
    // controller_->getAxis(0)->getLimitSwitches(0, &limit_switches);
    // controller_->setIntegerParam(controller_->SMCTaurusLimit1Type_, limit_switches);
    // limit_switches = 0;
    // controller_->getAxis(0)->getLimitSwitches(1, &limit_switches);
    // controller_->setIntegerParam(controller_->SMCTaurusLimit2Type_, limit_switches);

    // Read the current motor position
    sprintf(controller_->outString_, "%i np", (axisNo_ + 1));
    comStatus = controller_->writeReadController();
    if (comStatus) goto skip;

    // The response string is a double
    position = atof( (char *) &controller_->inString_);
    setDoubleParam(controller_->motorPosition_, (position / axisRes_) );
    setDoubleParam(controller_->motorEncoderPosition_, (position / axisRes_) );
  
    // Read the status of this motor
    sprintf(controller_->outString_, "%i nst", (axisNo_ + 1));
    comStatus = controller_->writeReadController();
    if (comStatus) goto skip;

    // The response string is an int
    axisStatus = atoi( (char *) &controller_->inString_);
  
    // Check the moving bit
    done = !(axisStatus & 0x1);
    setIntegerParam(controller_->motorStatusDone_, done);
    setIntegerParam(controller_->motorStatusMoving_, !done);
    *moving = done ? false:true;

    // Read the commanded velocity and acceleration
    sprintf(controller_->outString_, "%i gnv", (axisNo_ + 1));
    comStatus = controller_->writeReadController();
  
    sprintf(controller_->outString_, "%i gna", (axisNo_ + 1));
    comStatus = controller_->writeReadController();

    // Check the limit bit (0x40)
    if (axisStatus & 0x40)
    {
        asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, "%s: axis %i limit indicator active.\n", functionName, (axisNo_ + 1));
        // query limits?
    }

    // Check the e-stop bit (0x80)
    if (axisStatus & 0x80)
    {
        asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, "%s: axis %i emergency stopped.\n", functionName, (axisNo_ + 1));
    }
  
    // Check the e-stop switch active bit (0x200)
    if(axisStatus & 0x200)
    {
        asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, "%s: axis %i emergency stop switch active.\n",functionName, (axisNo_ + 1));
        setIntegerParam(controller_->motorStatusProblem_, 1);
    }
    else {
        setIntegerParam(controller_->motorStatusProblem_, 0);
    }

    // Check the device busy bit (0x400)
    if (axisStatus & 0x400)
    {
        asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s: axis %i device is busy - move commands discarded.\n", functionName, (axisNo_ + 1));
    }

    // Read the limit status
    // Note: calibration switch = low limit; range measure switch = high limit
    // also need to read the switch confiruation to see if limits are ignored"
    // Read switch confiruation
    // Bit 0:	polarity (0 = NO, 1 = NC)
    // Bit 1:	mask (0 = enabled, 1 = disabled)
    sprintf(controller_->outString_, "%i getsw", (axisNo_ + 1));
    comStatus = controller_->writeReadController();
    if (comStatus) goto skip;
    sscanf(controller_->inString_, "%i %i", &lowLimitConfig_, &highLimitConfig_);
    ignoreLowLimit = lowLimitConfig_ & 0x2;
    ignoreHighLimit = highLimitConfig_ & 0x2;
  
    // Read status of switches 0=inactive 1=active
    sprintf(controller_->outString_, "%i getswst", (axisNo_ + 1));
    comStatus = controller_->writeReadController();
    if (comStatus) goto skip;

    // The response string is of the form "0 0"
    sscanf(controller_->inString_, "%i %i", &lowLimit, &highLimit);
    if (ignoreLowLimit)
        setIntegerParam(controller_->motorStatusLowLimit_, 0);
    else
        setIntegerParam(controller_->motorStatusLowLimit_, lowLimit);

        if (ignoreHighLimit)
        setIntegerParam(controller_->motorStatusHighLimit_, 0);
    else
        setIntegerParam(controller_->motorStatusHighLimit_, highLimit);

    /*setIntegerParam(controller_->motorStatusAtHome_, limit);*/

    // Check the drive power bit (0x100)
    driveOn = (axisStatus & 0x100) ? 0 : 1;
    setIntegerParam(controller_->motorStatusPowerOn_, driveOn);
    setIntegerParam(controller_->motorStatusProblem_, 0);

    skip:
    setIntegerParam(controller_->motorStatusProblem_, comStatus ? 1:0);
    callParamCallbacks();
    return comStatus ? asynError : asynSuccess;
}


