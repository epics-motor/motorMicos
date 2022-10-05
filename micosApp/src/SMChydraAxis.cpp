
#include "SMChydraDriver.h"

/** Creates a new SMChydraAxis object.
  * \param[in] pC Pointer to the SMChydraController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SMChydraAxis::SMChydraAxis(SMChydraController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{  
  sprintf(pC_->outString_, "%i getmotor", (axisNo + 1));
  pC_->writeReadController();
  motorForm_ = atoi( (char *) &pC_->inString_ );

  sprintf(pC_->outString_, "%i getpitch", (axisNo + 1));
  pC_->writeReadController();
  pitch_ = atof( (char *) &pC_->inString_ );
  
  sprintf(pC_->outString_, "%i getpolepairs", (axisNo + 1));
  pC_->writeReadController();
  polePairs_ = atoi( (char *) &pC_->inString_ );

  sprintf(pC_->outString_, "%i getclperiod", (axisNo + 1));
  pC_->writeReadController();
  clPeriod_ = atof( (char *) &pC_->inString_ );

  switch (motorForm_)
  {
    case 0:
      // Stepper motor
      axisRes_ = pitch_ / ( 4.0 * polePairs_);
      break;

    case 1:
      // Linear or torque motor
      axisRes_ = clPeriod_;
      break;

    default:
      // For now assume clPeriod_ works for other motor forms
      axisRes_ = clPeriod_;
      break;
  }

  /* Enable gain support so that the CNEN field can be used to send
     the init command to clear a motor fault for stepper motors, even
     though they lack closed-loop support. */
  setIntegerParam(pC_->motorStatusGainSupport_, 1);

  // Determine the travel limits (will change after homing)
  sprintf(pC_->outString_, "%i getnlimit", (axisNo + 1));
  pC_->writeReadController();
  sscanf(pC_->inString_, "%lf %lf", &negTravelLimit_, &posTravelLimit_);

}
/** Change the axis resolution
  * \param[in] newResolution The new resolution
  */
asynStatus SMChydraAxis::changeResolution(double newResolution)
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
void SMChydraAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "    motorForm %d\n", motorForm_);
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

asynStatus SMChydraAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "SMChydraAxis::sendAccelAndVelocity";

  // Send the velocity
  sprintf(pC_->outString_, "%f %i snv", fabs(velocity * axisRes_), (axisNo_ + 1));
  status = pC_->writeController();

  // Send the acceleration
  // acceleration is in units/sec/sec
  sprintf(pC_->outString_, "%f %i sna", fabs(acceleration * axisRes_), (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}


asynStatus SMChydraAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "SMChydraAxis::move";

  status = sendAccelAndVelocity(acceleration, slewVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%f %i nr", (position * axisRes_), (axisNo_ + 1));
  } else {
    sprintf(pC_->outString_, "%f %i nm", (position * axisRes_), (axisNo_ + 1));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMChydraAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "SMChydraAxis::home";

  status = sendAccelAndVelocity(acceleration, slewVelocity);

  if (forwards) {
    sprintf(pC_->outString_, "%i nrm", (axisNo_ + 1));
  } else {
    sprintf(pC_->outString_, "%i ncal", (axisNo_ + 1));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMChydraAxis::moveVelocity(double baseVelocity, double slewVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "SMChydraAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: baseVelocity=%f, slewVelocity=%f, acceleration=%f\n",
    functionName, baseVelocity, slewVelocity, acceleration);
  
  /* SMC hydra does not have jog command. Move to a limit*/
  if (slewVelocity > 0.) {
    status = sendAccelAndVelocity(acceleration, slewVelocity);
    sprintf(pC_->outString_, "%f %i nm", posTravelLimit_, (axisNo_ + 1));
  } else {
    status = sendAccelAndVelocity(acceleration, (slewVelocity * -1.0));
    sprintf(pC_->outString_, "%f %i nm", negTravelLimit_, (axisNo_ + 1));
  }
  status = pC_->writeController();
  return status;
}

asynStatus SMChydraAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "SMChydraAxis::stop";

  // Set stop deceleration (will be overridden by accel if accel is higher)
  sprintf(pC_->outString_, "%f %i ssd", fabs(acceleration * axisRes_), (axisNo_ + 1));
  status = pC_->writeController();

  sprintf(pC_->outString_, "%i nabort", (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}

asynStatus SMChydraAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "SMChydraAxis::setPosition";

  // The argument to the setnpos command is the distance from the current position of the
  // desired origin, which is why the position needs to be multiplied by -1.0
  sprintf(pC_->outString_, "%f %i setnpos", (position * axisRes_ * -1.0), (axisNo_ + 1));
  status = pC_->writeController();
  return status;
}

asynStatus SMChydraAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  int regulatorMode;
  //static const char *functionName = "SMChydraAxis::setClosedLoop";

  switch (motorForm_)
  {
    case 0:
      // Stepper motor with encoder
    case 1:
      // Linear or torque motor

      // Get the regulator mode (0 = open-loop, 1 = standard, 2 = adaptive)
      pC_->getIntegerParam(axisNo_, pC_->SMChydraRegulatorMode_, &regulatorMode);
      
      if (closedLoop) {
        // enable closed-loop control
        sprintf(pC_->outString_, "%i %i setcloop", regulatorMode, (axisNo_ + 1));
        status = pC_->writeController();
        
        // reinit so the closed-loop setting takes effect (this powers on the motor)
        sprintf(pC_->outString_, "%i init", (axisNo_ + 1));
        status = pC_->writeController();

        // a delay is required after the init command is sent
        epicsThreadSleep(0.2);
      } else {
        // disable closed-loop control
        sprintf(pC_->outString_, "%i motoroff", (axisNo_ + 1));
        status = pC_->writeController();
      }
      
      break;

    default:
      // Do nothing
      break;
  }
  
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SMChydraAxis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int lowLimit;
  int highLimit;
  int ignoreLowLimit;
  int ignoreHighLimit;
  int axisStatus=-1;
  double position=0.0;
  asynStatus comStatus;

  static const char *functionName = "SMChydraAxis::poll";

  // Read the current motor position
  sprintf(pC_->outString_, "%i np", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is a double
  position = atof( (char *) &pC_->inString_);
  setDoubleParam(pC_->motorPosition_, (position / axisRes_) );
  setDoubleParam(pC_->motorEncoderPosition_, (position / axisRes_) );
  
  // Read the status of this motor
  sprintf(pC_->outString_, "%i nst", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is an int
  axisStatus = atoi( (char *) &pC_->inString_);
  
  // Check the moving bit
  done = !(axisStatus & 0x1);
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_, !done);
  *moving = done ? false:true;

  // Read the commanded velocity and acceleration
  sprintf(pC_->outString_, "%i gnv", (axisNo_ + 1));
  comStatus = pC_->writeReadController();
  
  sprintf(pC_->outString_, "%i gna", (axisNo_ + 1));
  comStatus = pC_->writeReadController();

  // Check the limit bit (0x40)
  if (axisStatus & 0x40)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i limit indicator active.\n",
      functionName, (axisNo_ + 1));
      
    // query limits?
  }

  // Check the e-stop bit (0x80)
  if (axisStatus & 0x80)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stopped.\n",
      functionName, (axisNo_ + 1));
  }
  
  // Check the e-stop switch active bit (0x200)
  if (axisStatus & 0x200)
  {
    asynPrint(this->pasynUser_, ASYN_TRACEIO_DRIVER, 
      "%s: axis %i emergency stop switch active.\n",
      functionName, (axisNo_ + 1));
    setIntegerParam(pC_->motorStatusProblem_, 1);
  }
  else{
    setIntegerParam(pC_->motorStatusProblem_, 0);
  }

  // Check the device busy bit (0x400)
  if (axisStatus & 0x400)
  {
    asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, 
      "%s: axis %i device is busy - move commands discarded.\n",
      functionName, (axisNo_ + 1));
  }

  // Read the limit status
  // Note: calibration switch = low limit; range measure switch = high limit
  // also need to read the switch confiruation to see if limits are ignored"

  // Read switch confiruation
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

  /*setIntegerParam(pC_->motorStatusAtHome_, limit);*/

  // Check the drive power bit (0x100)
  driveOn = (axisStatus & 0x100) ? 0 : 1;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

