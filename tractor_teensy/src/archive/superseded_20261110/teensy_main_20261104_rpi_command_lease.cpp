/*********************************************************************
  teensy_main_20261104_rpi_command_lease.cpp

  Production-candidate wrapper around the commissioned v2 generic RPi
  command lease. It adds only authenticated software E-stop and guarded
  reset-to-Pause modes. Phone/network policy remains entirely on the RPi.

  This file is not selected by the normal PlatformIO configuration and is
  not field-motion authorization.
*********************************************************************/

#define TRACTOR_RPI_COMMAND_LEASE_V3 1
#define TRACTOR_FIRMWARE_ID "teensy_main_20261104_rpi_command_lease_v3"
#include "teensy_main_20261031_rpi_command_lease.cpp"
