# motorMicos Releases

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
