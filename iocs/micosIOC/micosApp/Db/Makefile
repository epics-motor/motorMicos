TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#DB += xxx.db

### These databases come from motor
DB_INSTALLS += $(MOTOR)/db/asyn_motor.db
DB_INSTALLS += $(MOTOR)/db/asyn_motor_positions.req
DB_INSTALLS += $(MOTOR)/db/asyn_motor_settings.req
DB_INSTALLS += $(MOTOR)/db/motor.db
DB_INSTALLS += $(MOTOR)/db/motor_positions.req
DB_INSTALLS += $(MOTOR)/db/motor_settings.req

ifdef MOTOR_MICOS
### motorMicos was built outside motor
DB_INSTALLS += $(MOTOR_MICOS)/db/SMChydraAxis.db
else
### motorMicos was built inside motor/modules
DB_INSTALLS += $(MOTOR)/db/SMChydraAxis.db
endif

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

