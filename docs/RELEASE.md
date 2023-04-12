# motorMicos Releases

## __R2-1 (2023-04-11)__
R2-1 is a release based on the master branch.  

### Changes since R2-0

#### New features
* Pull request [#11](https://github.com/epics-motor/motorMicos/pull/11): [Abdalla Al-Dalleh](https://github.com/AbdallaDalleh) added support for the SMC Taurus

#### Modifications to existing features
* None

#### Bug fixes
* None

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R2-0 (2020-05-12)__
R2-0 is a release based on the master branch.  

### Changes since R1-1

#### New features
* Pull request [#7](https://github.com/epics-motor/motorMicos/pull/7): [Michael Dunning](https://github.com/mpdunning) added support for the SMC Corvus Eco
* Pull request [#8](https://github.com/epics-motor/motorMicos/pull/8): A new database, ``SMChydraAxis.db``, provides a regulator mode record that allows adaptive regulator mode to be used.

#### Modifications to existing features
* Pull request [#8](https://github.com/epics-motor/motorMicos/pull/8): The behavior of ``setClosedLoop()`` in the SMC hydra driver has changed.  The argument to the ``setcloop`` command is determined by the new regulator mode.  The ``motoroff`` command is now sent to the controller when ``closedLoop`` is false.

#### Bug fixes
* Commit [6049202](https://github.com/epics-motor/motorMicos/commit/6049202cd3a0f8c9f0450f179baeb3e9503785df): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Pull request [#5](https://github.com/epics-motor/motorMicos/pull/5): Eliminated compiler warnings

## __R1-1 (2019-08-08)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* Pull request [#3](https://github.com/epics-motor/motorMicos/pull/3): Added SMChydraChangeResolution to allow the resolution of an axis to be corrected when the it differs from the value reported by getclperiod

#### Bug fixes
* Pull request [#2](https://github.com/epics-motor/motorMicos/pull/2): Send the absolute values of velocity, acceleration, and deceleration to the SMC Hydra to eliminate problems caused by negative getclperiod values

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
