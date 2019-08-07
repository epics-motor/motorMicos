# motorMicos Releases

## __R1-1 (2019-08-08)__
R1-1 is a release based on the master branch.  

### Changes since R1-1

#### New features
* Pull request [#3](https://github.com/epics-motor/motorMicos/pull/3): Added SMChydraChangeResolution to allow the resolution of an axis to be corrected when the it differs from the value reported by getclperiod

#### Bug fixes
* Pull request [#2](https://github.com/epics-motor/motorMicos/pull/2): Send the absolute values of velocity, acceleration, and deceleration to the controller to eliminate problems caused by negative getclperiod values

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorMicos is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorMicos can be built outside of the motor directory
* motorMicos has a dedicated example IOC that can be built outside of motorMicos

#### Modifications to existing features
* None

#### Bug fixes
* Commit [64428ef](https://github.com/epics-motor/motorMicos/commit/64428efa4b2d5e202fa0d5d019e4730997e5d0ff) Registered MoCo Setup/Config functions so they can be called from iocsh
