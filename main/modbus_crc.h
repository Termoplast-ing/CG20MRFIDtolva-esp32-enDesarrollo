
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>

uint16_t modbus_crc16(const uint8_t *data, uint16_t length);

#endif