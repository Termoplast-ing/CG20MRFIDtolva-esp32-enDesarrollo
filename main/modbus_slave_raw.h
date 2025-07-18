#ifndef MODBUS_SLAVE_RAW_H
#define MODBUS_SLAVE_RAW_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "time.h"
#include "freertos/FreeRTOS.h"


#define UART_NUM2 UART_NUM_2
#define BUF_SIZE2 1024

/*typedef struct {
    char nombre[16];
    uint8_t tipoCurva;
    uint8_t pesoDosis;
    time_t fechaServicio;
    uint8_t indiceCorporal;
    bool agua;
    uint8_t cantDosis;
    uint16_t intervaloMin;    
} data_animal1;*/


extern uint8_t slave_addr;
extern time_t timeStampDatos;
extern time_t timeStampConfig;
extern time_t timeStampCurva;
extern time_t tiempoReal;


void modbus_slave_init(void);
void modbus_slave_poll();
void modbus_slave_process(uint8_t *request, size_t len);
//void actualizar();

void modbus_slave_process_0x03(uint8_t *request, size_t request_len);
void modbus_slave_process_0x06(uint8_t *request, size_t request_len);
void modbus_slave_process_0x10(uint8_t *request, size_t request_len);
void modbus_slave_process_0x51(uint8_t *request, size_t request_len);
void modbus_slave_process_0x40(uint8_t *request, size_t request_len);
void modbus_slave_process_0x41(uint8_t *request, size_t request_len);
//void modbus_slave_process_0x42(uint8_t *request, size_t request_len);
void modbus_slave_process_0x60(uint8_t *request, size_t request_len);
void modbus_slave_process_0x61(uint8_t *request, size_t request_len);
void modbus_slave_process_0x70(uint8_t *request, size_t request_len);
#endif // MODBUS_SLAVE_RAW_H