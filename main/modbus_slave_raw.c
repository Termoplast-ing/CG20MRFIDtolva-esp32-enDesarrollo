/*
#include "modbus_slave_raw.h"
#include "modbus_crc.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define UART_PORT_NUM MODBUS_UART_PORT
#define GPIO_NUM_4 4
#define MY_SLAVE_ID MODBUS_SLAVE_ID
static const char *TAG = "MODBUS_SLAVE";
static uint16_t holding_registers[MODBUS_REGISTER_COUNT];

esp_err_t modbus_slave_init(void) {
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(MODBUS_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(MODBUS_UART_PORT, &uart_config);
    uart_set_pin(MODBUS_UART_PORT, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // RX, TX
    return ESP_OK;
}

static void send_response(const uint8_t *data, uint16_t len) {
    uint8_t tx_buffer[MODBUS_MAX_FRAME];
    memcpy(tx_buffer, data, len);
    uint16_t crc = modbus_crc16(tx_buffer, len);
    tx_buffer[len++] = crc & 0xFF;
    tx_buffer[len++] = crc >> 8;
    uart_write_bytes(MODBUS_UART_PORT, (const char *)tx_buffer, len);
}

esp_err_t modbus_slave_poll(void) {
    int len = uart_read_bytes(UART_PORT_NUM, rx_buffer, sizeof(rx_buffer), 100 / portTICK_PERIOD_MS);

    if (len < 8) return ESP_FAIL;  // Longitud mínima para cualquier función

    uint8_t slave_addr = rx_buffer[0];
    uint8_t function_code = rx_buffer[1];

    if (slave_addr != MY_SLAVE_ID) return ESP_OK;  // No es para mí

    ESP_LOGI(TAG, "Trama recibida. Función: 0x%02X", function_code);

    switch (function_code) {
        case 0x03:
            return handle_read_holding_registers(rx_buffer, len);
        case 0x06:
            return handle_write_single_register(rx_buffer, len);
        case 0x10:
            return handle_write_multiple_registers(rx_buffer, len);
        default:
            ESP_LOGW(TAG, "Función no soportada: 0x%02X", function_code);
            return ESP_FAIL;
    }
}

void modbus_slave_process_0x03(uint8_t *request, size_t request_len) {
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    uint8_t response[256];
    size_t len = 0;

    response[len++] = request[0]; // slave ID
    response[len++] = 0x03;       // function
    response[len++] = quantity * 2;

    for (int i = 0; i < quantity; i++) {
        uint16_t val = modbus_slave_registers[(start_addr + i) % 100];
        response[len++] = (val >> 8) & 0xFF;
        response[len++] = val & 0xFF;
    }

    uint16_t crc = modbus_crc16(response, len);
    response[len++] = crc & 0xFF;
    response[len++] = (crc >> 8) & 0xFF;

    uart_write_bytes(UART_NUM_1, (const char *)response, len);
}

void modbus_slave_process_0x06(uint8_t *request, size_t request_len) {
    uint16_t reg_addr = (request[2] << 8) | request[3];
    uint16_t value = (request[4] << 8) | request[5];

    modbus_slave_registers[reg_addr % 100] = value;

    // Echo de vuelta la misma trama como respuesta
    uart_write_bytes(UART_NUM_1, (const char *)request, request_len);
}

void modbus_slave_process_0x10(uint8_t *request, size_t request_len) {
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t quantity = (request[4] << 8) | request[5];
    uint8_t byte_count = request[6];

    for (int i = 0; i < quantity; i++) {
        uint16_t value = (request[7 + i * 2] << 8) | request[8 + i * 2];
        modbus_slave_registers[(start_addr + i) % 100] = value;
    }

    uint8_t response[8];
    size_t len = 0;
    response[len++] = request[0];
    response[len++] = 0x10;
    response[len++] = request[2];
    response[len++] = request[3];
    response[len++] = request[4];
    response[len++] = request[5];

    uint16_t crc = modbus_crc16(response, len);
    response[len++] = crc & 0xFF;
    response[len++] = (crc >> 8) & 0xFF;

    uart_write_bytes(UART_NUM_1, (const char *)response, len);
}
*/

#include "modbus_slave_raw.h"
#include "modbus_crc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "estructuras.h"

#define UART_NUM2 UART_NUM_2
// #define UART_FIFO_LEN 128

// Function prototypes for custom Modbus handlers
void modbus_slave_process_0x51(uint8_t *request, size_t len);
void modbus_slave_process_0x40(uint8_t *request, size_t len);
void modbus_slave_process_0x41(uint8_t *request, size_t len);
void modbus_slave_process_0x42(uint8_t *request, size_t len);
void modbus_slave_process_0x60(uint8_t *request, size_t len);
void modbus_slave_process_0x61(uint8_t *request, size_t len);

#define SLAVE_ADDR slave_addr


data_animal auxiliar;
configuration auxConfig;
tipo_curva auxCurva;
tipo_curva auxCurva2;

uint8_t data[BUF_SIZE2];
static const char *TAG = "MODBUS_SLAVE";
static uint16_t holding_registers[BUF_SIZE2]; // Array de registros de retención

static QueueHandle_t uart_queue2;

static void modbus_uart_task(void *arg){
    uart_event_t event;
    //uint8_t data[BUF_SIZE2];

    while (1) {
        
        if (xQueueReceive(uart_queue2, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_NUM2, data, event.size, pdMS_TO_TICKS(100));
                printf("Evento UART recibido: %d bytes\n", len);
                if (len >= 8) {  
                        // Verificamos dirección
                        if (data[0] != SLAVE_ADDR) {
                            continue;
                        }
                        // Verificamos CRC
                        uint16_t crc_calc = modbus_crc16(data, len - 2);
                        uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
                        if (crc_calc != crc_recv) {
                            continue;
                        }
printf("event.type: %d\n", event.size);
                        ESP_LOGI(TAG, "Trama recibida (%d bytes):", len);
                        // Procesamos la trama Modbus                
                    modbus_slave_process(data, len);
                    uart_flush_input(UART_NUM2); // Vaciar el buffer UART
                    xQueueReset(uart_queue2);  

                }
                
            }
        }
    }
}

void modbus_slave_init()
{
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);

    const uart_config_t uart_config1 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM2, &uart_config1);
    uart_set_pin(UART_NUM2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // TXD=17, RXD=16
    uart_driver_install(UART_NUM2, BUF_SIZE2 * 2, BUF_SIZE2 * 2, 20, &uart_queue2, 0);

    xTaskCreate(modbus_uart_task, "modbus_uart_task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART inicializado y cola de interrupción creada");
}

void modbus_slave_process(uint8_t *request, size_t len) {

    uint8_t function_code = request[1];
   // uint8_t funcion_codigo = request[2];
   printf("brian%02x\n", function_code);
    switch (function_code) {
        case 0x03:
            modbus_slave_process_0x03(request, len);
            break;
        case 0x06:
            modbus_slave_process_0x06(request, len);
            break;
        case 0x10:
            modbus_slave_process_0x10(request, len);
            break;
        case 0x40:
            modbus_slave_process_0x40(request, len);
            break;
        case 0x41:
            modbus_slave_process_0x41(request, len);
            break;
        case 0x42:
            modbus_slave_process_0x42(request, len);
            break;
        case 0x43:
           // modbus_slave_process_0x43(request, len);
            break;
        case 0x44:
           // modbus_slave_process_0x44(request, len);
            break;
        case 0x45:
            //modbus_slave_process_0x45(request, len);
            break;
        case 0x51:
            modbus_slave_process_0x51(request, len);
            break;
        case 0x60:
            modbus_slave_process_0x60(request, len);
            break;
        case 0x61:
            modbus_slave_process_0x61(request, len);
            break;
        default:
            ESP_LOGW(TAG, "Función no soportada: 0x%02X", function_code);
            break;
    }
    //actualizar();
}

void modbus_slave_poll(){
    uint8_t request[BUF_SIZE2];
    int len = uart_read_bytes(UART_NUM2, request, BUF_SIZE2, 20 / portTICK_PERIOD_MS);
    if (len <= 0) return;

    ESP_LOGI(TAG, "Trama recibida (%d bytes):", len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", request[i]);
    }
    printf("\n");

    if (len < 8) {
        ESP_LOGW(TAG, "Trama muy corta");
        return;
    }

    // Verificamos dirección
    if (request[0] != SLAVE_ADDR) {
        ESP_LOGW(TAG, "Trama no es para mí (ID %02X)", request[0]);
        return;
    }

    // Verificamos CRC
    uint16_t crc_calc = modbus_crc16(request, len - 2);
    uint16_t crc_recv = request[len - 2] | (request[len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC inválido (calc=0x%04X, recv=0x%04X)", crc_calc, crc_recv);
        return;
    }

    // Procesamos función
    switch (request[1]) {
        case 0x03:
            modbus_slave_process_0x03(request, len);
            break;
        case 0x06:
            modbus_slave_process_0x06(request, len);
            break;
        case 0x10:
            modbus_slave_process_0x10(request, len);
            break;
        default:
            ESP_LOGW(TAG, "Función 0x%02X no soportada", request[1]);
            break;
    }
}

void modbus_slave_process_0x03(uint8_t *request, size_t len)
{
    uint16_t reg_start = (request[2] << 8) | request[3];
    uint16_t quantity  = (request[4] << 8) | request[5];
    
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Solicitud fuera de rango");
        return;
    }

    uint8_t response[BUF_SIZE2];
    int resp_len = 0;
    response[resp_len++] = SLAVE_ADDR;
    response[resp_len++] = 0x03;
    response[resp_len++] = quantity * 2;

    for (int i = 0; i < quantity; i++) {
        response[resp_len++] = holding_registers[reg_start + i] >> 8;
        response[resp_len++] = holding_registers[reg_start + i] & 0xFF;
    }

    uint16_t crc = modbus_crc16(response, resp_len);
    response[resp_len++] = crc & 0xFF;
    response[resp_len++] = crc >> 8;
    
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    printf("Enviando respuesta: ");
    uart_write_bytes(UART_NUM2, (const char *)response, resp_len);

    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    //uart_write_bytes(UART_NUM2, (const char *)response, resp_len);
    ESP_LOGI(TAG, "Respondido 0x03 con %d registros", quantity);
}

void modbus_slave_process_0x06(uint8_t *request, size_t len)
{
    for (int i = 0; i < len; i++) {
        printf("%02X ", request[i]);
    }
    uint16_t reg_addr = (request[2] << 8) | request[3];
    uint16_t value    = (request[4] << 8) | request[5];

    if (reg_addr >= sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Registro fuera de rango");
        return;
    }

    holding_registers[reg_addr] = value;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)request, len);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    // Eco de vuelta la misma trama
    ESP_LOGI(TAG, "Registro %d actualizado con valor 0x%04X", reg_addr, value);
}

void modbus_slave_process_0x10(uint8_t *request, size_t len)
{
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    //uint8_t byte_count  = request[6];

    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }

    for (int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }

    // Respuesta: dirección, función, registro inicial, cantidad, CRC
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x10;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];

    uint16_t crc = modbus_crc16(response, 6);
    response[6] = crc & 0xFF;
    response[7] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    ESP_LOGI(TAG, "Escribí %d registros desde %d", quantity, reg_start);

}
// --- Ejemplo: Escribir múltiples registros (0x51) ---
//Función: escribir datos de dieta de caravanas
void modbus_slave_process_0x51(uint8_t *request, size_t len)
{
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    uint64_t tiempoDatosCentral = 0;

    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
    printf("hol %d ",quantity);
    for(int i = 0; i < 8; i++) {
        tiempoDatosCentral = tiempoDatosCentral | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }
    printf("tiempoDatosCentral: %llx\n", tiempoDatosCentral);
/*
    tiempoDatosCentral = ((uint64_t)holding_registers[7] << 48) |
                        ((uint64_t)holding_registers[8] << 32) |
                        ((uint64_t)holding_registers[9] << 16) |
                        ((uint64_t)holding_registers[10]);
*/

    // Verificamos si el tiempo de datos es mayor que el tiempo de la central

    // Respuesta: dirección, función, registro inicial, cantidad, CRC
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x51;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x51 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    printf("datos time central:%llx\n",tiempoDatosCentral);
    if (tiempoDatosCentral > timeStampDatos) {
        for (int i = 15; i<31; i++) {
            auxiliar.nombre[(i-15)]= request[(i)]; //>> 8) & 0xFF; // Byte alto
            //axiliar.nombre[i*2 + 1] = request[((i*2)+1)] & 0xFF;        // Byte bajo
            printf("%c ", auxiliar.nombre[i-15]);
        }
        auxiliar.tipoCurva = request[31];
        auxiliar.pesoDosis = request[32];
        for(int i = 0; i < 8; i++) {
        auxiliar.fechaServicio = auxiliar.fechaServicio | ((uint64_t)request[33+i] << (8 * (7 - i)));
        }
        auxiliar.indiceCorporal = request[41];
        auxiliar.agua = request[42] & 0x01; // Asumiendo que el bit 0 indica si hay agua
        auxiliar.cantDosis=request[43];
        auxiliar.intervaloMin=(((uint16_t) request[44]) << 8 |
                                ((uint16_t) request[45] & 0xFF));
    }
}
// --- Ejemplo: Escribir múltiples registros (0x40) ---
//Función: escribir datos de configuracion
void modbus_slave_process_0x40(uint8_t *request, size_t len)
{
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    uint64_t tiempoConfigCentral = 0;

    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
    printf("hol %d ",quantity);
    for(int i = 0; i < 8; i++) {
        tiempoConfigCentral = tiempoConfigCentral | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }
    printf("tiempoDatosCentral: %llx\n", tiempoConfigCentral);

    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x51;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x40 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    if (tiempoConfigCentral > timeStampDatos) {
        auxConfig.calibracionMotor = request[15];
        auxConfig.calibracionAgua = request[16];
        auxConfig.pesoAnimalDesconocido = request[17];
    }        
        printf("calibracionMotor= %02X\n", auxConfig.calibracionMotor);
        printf("calibracionAgua= %02X\n", auxConfig.calibracionAgua);
        printf("pesoAnimalDesconocido= %02X\n", auxConfig.pesoAnimalDesconocido);
}
// --- Ejemplo: Escribir múltiples registros (0x41) ---
//Función: escribir datos de caravanas libre 1
void modbus_slave_process_0x41(uint8_t *request, size_t len)
{
    printf("holis 41\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];

    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
   
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x41;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x41 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
        for (int i = 0; i<15; i++) {
            auxConfig.caravanaLibre1[(i)]= request[(i+7)];
        }
        printf("L1= %s\n", auxConfig.caravanaLibre1);

}
// --- Ejemplo: Escribir múltiples registros (0x42) ---
//Función: escribir datos de caravanas libre 2
void modbus_slave_process_0x42(uint8_t *request, size_t len){
    printf("holis 42\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];

    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
   
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x42;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x42 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
        for (int i = 0; i<15; i++) {
            auxConfig.caravanaLibre2[(i)]= request[(i+7)];
        }
        printf("L1= %s\n", auxConfig.caravanaLibre2);
}
// --- Ejemplo: Escribir múltiples registros (0x60) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x60(uint8_t *request, size_t len){
    printf("holis 60\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    uint64_t tiempoCurvaCentral = 0;

    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }

    for(int i = 0; i < 8; i++) {
        tiempoCurvaCentral = tiempoCurvaCentral | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }
    printf("tiempoCurvaCentral: %llx\n", tiempoCurvaCentral);
   
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x60;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
}
// --- Ejemplo: Escribir múltiples registros (0x61) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x61(uint8_t *request, size_t len){
    printf("holis 61\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x61;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x61 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
        auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
        auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);
    }
}

void modbus_slave_process_0x70(uint8_t *request, size_t len){
    printf("holis 70\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x61;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = request[5];
    response[6] = request[6];

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    
    ESP_LOGI(TAG, "0x61 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
        auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
        auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);
    }
    
}


