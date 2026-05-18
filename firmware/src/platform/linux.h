#pragma once
#ifdef LINUX

#include "hal_linux.h"
#include <I2C/I2C_Mock.h>
#include <SPI/SPI_Mock.h>
#include <CAN/CAN_Linux.h>

extern CAN_Linux g_can;

#endif /* LINUX */
