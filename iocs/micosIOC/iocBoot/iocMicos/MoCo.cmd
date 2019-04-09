
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)

dbLoadTemplate("MoCo.substitutions")

# Micos MoCo driver setup parameters:
# Load MicosSetup once.
#     (1) max # of controller groups.  Controller groups are per serial
#         port.
#     (2) max # axes per controller group.  Maximum 16. (addr 0-15)
#     (3) motor task polling rate (min=1Hz, max=60Hz, 10Hz works well)
# Example:
#   MicosSetup(1, 2, 10) 1 group. 2 axes (controllers) in the group.
#                        10Hz poll.
MicosSetup(1, 2, 10)

# Micos MoCo driver configuration parameters:
# Load one MicosConfig for each group of Micos drivers.
#     (1) "Controller group" number
#     (2) ASYN port name
MicosConfig(0, "serial1")
#!var drvMicosDebug 4
