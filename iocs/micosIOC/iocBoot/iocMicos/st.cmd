#!../../bin/linux-x86_64/micos

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/micos.dbd"
micos_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=micos:")

##
< MoCo.cmd
< motor.cmd.SMChydra
< motor.cmd.SMCcorvus

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("micos:")

# Boot complete
