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
#include "reloj.h"


#define UART_NUM2 UART_NUM_2
#define MAX_ANIMALES 100
// #define UART_FIFO_LEN 128

// Function prototypes for custom Modbus handlers
void modbus_slave_process_0x51(uint8_t *request, size_t len);
void modbus_slave_process_0x40(uint8_t *request, size_t len);
void modbus_slave_process_0x41(uint8_t *request, size_t len);
//void modbus_slave_process_0x42(uint8_t *request, size_t len);
void modbus_slave_process_0x60(uint8_t *request, size_t len);
void modbus_slave_process_0x61(uint8_t *request, size_t len);
void modbus_slave_process_0x70(uint8_t *request, size_t len);
void modbus_slave_process_0x20(uint8_t *request, size_t len);

#define SLAVE_ADDR slave_addr


//data_animal auxiliar;
configuration auxConfig;
tipo_curva auxCurva;
tipo_curva auxCurva2;
time_t tiempoReal = 0; // Variable para almacenar el tiempo real del reloj

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
               // printf("Evento UART recibido: %d bytes\n", len);
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
                       // printf("event.type: %d\n", event.size);
                       // ESP_LOGI(TAG, "Trama recibida (%d bytes):", len);
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
   // ESP_LOGI(TAG, "UART inicializado y cola de interrupción creada");
}

void modbus_slave_process(uint8_t *request, size_t len) {
    /*--- DEBUG: VER PAQUETE COMPLETO QUE LLEGA DEL MASTER ---
    printf(">>> MODBUS PACKET RECIBIDO (%d bytes) <<<\n", len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", request[i]);
    }
    printf("\n");*/
    uint8_t function_code = request[1];
   // uint8_t funcion_codigo = request[2];
//printf("%d\n", function_code);
//printf("Función recibida: 0x%02X\n", function_code);
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
       /*case 0x42:
          modbus_slave_process_0x42(request, len);
            break;
        case 0x43:
            modbus_slave_process_0x43(request, len);
            break;
        case 0x44:
            modbus_slave_process_0x44(request, len);
            break;
        case 0x45:
            modbus_slave_process_0x45(request, len);
            break;
        */case 0x51:
            modbus_slave_process_0x51(request, len);
            break;
        case 0x60:
            modbus_slave_process_0x60(request, len);
            break;
        case 0x61:
            modbus_slave_process_0x61(request, len);
            break;
        case 0x62:
            modbus_slave_process_0x62(request, len);
            break;
        case 0x63:
            modbus_slave_process_0x63(request, len);
            break;
        case 0x64:
            modbus_slave_process_0x64(request, len);
            break;
        case 0x65:
            modbus_slave_process_0x65(request, len);
            break;
        case 0x70:
            modbus_slave_process_0x70(request, len);
            break;
        case 0x20:
        //printf("hola mundo\n");
            modbus_slave_process_0x20(request, len);
            break;
        default:
            ESP_LOGW(TAG, "Función no soportada: 0x%02X", function_code);
            break;
    }
    //actualizar();
}

/*
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
*/

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
    //printf("Enviando respuesta: ");
    uart_write_bytes(UART_NUM2, (const char *)response, resp_len);

    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    //uart_write_bytes(UART_NUM2, (const char *)response, resp_len);
    ESP_LOGI(TAG, "Respondido 0x03 con %d registros", quantity);
}

void modbus_slave_process_0x06(uint8_t *request, size_t len)
{
   // for (int i = 0; i < len; i++) {
   //     printf("%02X ", request[i]);
   // }
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
    //printf("tareaMODBUS 51");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = request[5];
    uint64_t tiempoDatosCentral = 0;
    bool copiar = false;
    printf("estoy en gestion 51\n");

    //uint8_t byte_count  = request[6];
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
  //  printf("hol %d ",quantity);
    for(int i = 0; i < 8; i++) {
        tiempoDatosCentral = tiempoDatosCentral | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }
  //  printf("tiempoDatosCentral: %llx\n", tiempoDatosCentral);
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
        response[5] = request[5]; // Mantener el byte de cantidad de registros
        response[6] = request[6]; // Asignar el byte de cantidad de registros
  
    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 8);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
   // printf("indice de caravana: %d\n", request[4]);
   // printf("tiempo datos: %lld\n", timeStampDatos);
   // printf("tiempo central: %lld\n", tiempoDatosCentral);
    
    //ESP_LOGI(TAG, "0x51 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
  //  printf("datos time central:%lld\n",tiempoDatosCentral);
   // printf("datos time local:%lld\n",timeStampDatos);
   // printf("indice %d\n",request[4]);
    if(((tiempoDatosCentral > timeStampDatos) /*|| (tiempoDatosCentral < timeStampDatos)*/ || (timeStampDatos == 0))){
    copiar= true;
}
 if(copiar){       // Actualizar el timestamp de datos

        for (int i = 15; i<31; i++) {
       
            auxiliar.nombre[(i-15)]= request[(i)]; //>> 8) & 0xFF; // Byte alto
            //axiliar.nombre[i*2 + 1] = request[((i*2)+1)] & 0xFF;        // Byte bajo
            //printf("%c ", auxiliar.nombre[i-15]);
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
        
        /*printf("nombre: %s\n", auxiliar.nombre);
        printf("tipoCurva: %d\n", auxiliar.tipoCurva);
        printf("pesoDosis: %d\n", auxiliar.pesoDosis);
        printf("fechaServicio: %lld\n", auxiliar.fechaServicio);
        printf("indiceCorporal: %d\n", auxiliar.indiceCorporal);
        printf("agua: %d\n", auxiliar.agua);
        printf("cantDosis: %d\n", auxiliar.cantDosis);
        printf("intervaloMin: %d\n", auxiliar.intervaloMin);*/
                //                for(uint8_t i = 0; i < 20; i++) {
              // Buscamos un espacio libre en el corral que puede estar lleno de char'0' o estar vacio
                //if( (corral[i].nombre[0] == '\0')||(corral[i].nombre=='000000000000000') ){ // Verificamos si el nombre está vacío
                  //  corral[i]= auxiliar; // Copiamos los datos del animal al corral
                   // break;
               // }
              // printf("posicion de animal: %d", request[4]);
            corral[request[4]]=auxiliar;     
            auxiliar = (data_animal){0}; // Limpiamos la estructura auxiliar para evitar datos residuales
                            //printf("aqui\n");

 }
 //if(copiar && (request[4]==19)){
 //   timeStampDatos = tiempoDatosCentral; // Actualizar el timestamp de datos
 //   copiar=false;
 //}
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
    //printf("hol %d ",quantity);
    for(int i = 0; i < 8; i++) {
        tiempoConfigCentral = tiempoConfigCentral | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }
    //printf("tiempoDatosCentral: %lld\n", tiempoConfigCentral);
    //printf("tiempoConfigCentral: %lld\n", timeStampConfig);

    uint8_t response[9];

    printf("tiempo central: %lld\n", tiempoConfigCentral);
    printf("tiempo config: %lld\n", timeStampConfig);

    if (tiempoConfigCentral > timeStampConfig) {
            response[0] = SLAVE_ADDR;
    response[1] = 0x40;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
        response[5] = 0xFF;
        response[6] =  0xFF;
        printf("%d\n",response[5]);
        printf("%d\n",response[6]);
    
        timeStampConfig = tiempoConfigCentral; // Actualizar el timestamp de configuración
    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;
printf("response[5]= %d\n", response[5]);
printf("response[6]= %d\n", response[6]);
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 9);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
   

    printf("%d\n",response[5]);
printf("%d\n",response[6]);
    
    //ESP_LOGI(TAG, "0x40 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    
        auxConfig.calibracionMotor = request[15];
        auxConfig.calibracionAgua = request[16];
        auxConfig.pesoAnimalDesconocido = request[17];
          
        //printf("calibracionMotor= %d\n", auxConfig.calibracionMotor);
        //printf("calibracionAgua= %d\n", auxConfig.calibracionAgua);
        //printf("pesoAnimalDesconocido= %d\n", auxConfig.pesoAnimalDesconocido);
        configuracion.calibracionMotor=auxConfig.calibracionMotor;
        configuracion.calibracionAgua = auxConfig.calibracionAgua;
        configuracion.pesoAnimalDesconocido = auxConfig.pesoAnimalDesconocido;
        strcpy(corral[20].nombre, "999999999999999");
        corral[20].nombre[15] = '\0';
        corral[20].agua=255;
    corral[20].cantDosis=1;
    corral[20].indiceCorporal=1;
    corral[20].intervaloMin=0;
    corral[20].pesoDosis=configuracion.pesoAnimalDesconocido;
    corral[20].tipoCurva=1;
    corral[20].fechaServicio=0;
    }else{
                  response[0] = SLAVE_ADDR;
    response[1] = 0x40;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
            response[5] = 0x00;
        response[6] =  0x00;

    uint16_t crc = modbus_crc16(response, 6);
    response[7] = crc & 0xFF;
    response[8] = crc >> 8;

    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    uart_write_bytes(UART_NUM2, (const char *)response, 9);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);

        }
}
// --- Ejemplo: Escribir múltiples registros (0x41) ---
//Función: escribir datos de caravanas libre 1
void modbus_slave_process_0x41(uint8_t *request, size_t len)
{

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
    
    //ESP_LOGI(TAG, "0x41 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
        for (int i = 0; i<15; i++) {
            auxConfig.caravanaLibre1[(i)]= request[(i+7)];
            auxConfig.caravanaLibre2[(i)]= request[(i+7)+16];
            auxConfig.caravanaLibre3[(i)]= request[(i+7)+39];
            auxConfig.caravanaLibre4[(i)]= request[(i+7)+55];
            auxConfig.caravanaLibre5[(i)]= request[(i+7)+71];
        }
        printf("L1= %s\n", auxConfig.caravanaLibre1);
        printf("L2= %s\n", auxConfig.caravanaLibre2);
        printf("L3= %s\n", auxConfig.caravanaLibre3);
        memcpy(configuracion.caravanaLibre1, auxConfig.caravanaLibre1, sizeof(configuracion.caravanaLibre1));
        memcpy(configuracion.caravanaLibre2, auxConfig.caravanaLibre2, sizeof(configuracion.caravanaLibre2));
        memcpy(configuracion.caravanaLibre3, auxConfig.caravanaLibre3, sizeof(configuracion.caravanaLibre3));
        memcpy(configuracion.caravanaLibre4, auxConfig.caravanaLibre4, sizeof(configuracion.caravanaLibre4));
        memcpy(configuracion.caravanaLibre5, auxConfig.caravanaLibre5, sizeof(configuracion.caravanaLibre5));

memcpy(corral[21].nombre,configuracion.caravanaLibre1,sizeof(configuracion.caravanaLibre1));
memcpy(corral[22].nombre,configuracion.caravanaLibre2,sizeof(configuracion.caravanaLibre2));
memcpy(corral[23].nombre,configuracion.caravanaLibre3,sizeof(configuracion.caravanaLibre3));
memcpy(corral[24].nombre,configuracion.caravanaLibre4,sizeof(configuracion.caravanaLibre4));
memcpy(corral[25].nombre,configuracion.caravanaLibre5,sizeof(configuracion.caravanaLibre5));

for(uint8_t i=21; i<25;i++){
    corral[i].agua=255;
    corral[i].cantDosis=1;
    corral[i].indiceCorporal=1;
    corral[i].intervaloMin=1/3600;
    corral[i].pesoDosis=1;
    corral[i].tipoCurva=1;
    corral[i].fechaServicio=mktime(&ahora);
    }

}

// --- Ejemplo: Escribir múltiples registros (0x42) ---
//Función: escribir datos de caravanas libre 2
/*void modbus_slave_process_0x42(uint8_t *request, size_t len){
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
}*/
// --- Ejemplo: Escribir múltiples registros (0x60) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x60(uint8_t *request, size_t len){
    //printf("holis 60\n");
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
    //printf("tiempoCurvaCentral: %llx\n", tiempoCurvaCentral);
   
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x60;
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
       if (tiempoCurvaCentral > timeStampCurva) {
        response[5] = 0x00;
        response[6] =  0x00;
        timeStampCurva = tiempoCurvaCentral; // Actualizar el timestamp de curva
       }else
       {
        response[5] = 0x00;
        response[6] =  0x00;
       }
       

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
    
    //ESP_LOGI(TAG, "0x61 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
       // auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
       // auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    /*printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);*/
    }
    curva[0]=auxCurva;
    memset(&auxCurva, 0, sizeof(auxCurva));

           // printf("imprimiendo curva !!!1\n");
           // for(uint8_t j=0;j<17;j++){
           //     printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[0].segmentos[j].inicio, curva[0].segmentos[j].pesoInicio);
            //}
     
}
// --- Ejemplo: Escribir múltiples registros (0x62) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x62(uint8_t *request, size_t len){
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
    response[1] = 0x62;
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
    
    ESP_LOGI(TAG, "0x62 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
       // auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
       // auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    /*printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);*/
    }
    curva[1]=auxCurva;
    memset(&auxCurva, 0, sizeof(auxCurva));
    
           /* printf("imprimiendo curva ???2\n");
            for(uint8_t j=0;j<17;j++){
                printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[1].segmentos[j].inicio, curva[1].segmentos[j].pesoInicio);
            }*/
   
}
// --- Ejemplo: Escribir múltiples registros (0x63) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x63(uint8_t *request, size_t len){
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
    response[1] = 0x63;
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
    
    ESP_LOGI(TAG, "0x63 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
       // auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
       // auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    /*printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);*/
    }
    curva[2]=auxCurva;
    memset(&auxCurva, 0, sizeof(auxCurva));
 
           // printf("imprimiendo curva 3\n");
            //for(uint8_t j=0;j<17;j++){
            //    printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[2].segmentos[j].inicio, curva[2].segmentos[j].pesoInicio);
            //}
        
}
// --- Ejemplo: Escribir múltiples registros (0x64) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x64(uint8_t *request, size_t len){
    /*printf("holis 64\n");
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
    response[1] = 0x64;
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
    
    ESP_LOGI(TAG, "0x64 - Escribí %d registros desde %d", quantity, reg_start);
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
    }*/
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
    response[1] = 0x64;
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
    
    ESP_LOGI(TAG, "0x64 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
       // auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
       // auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    /*printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);*/
    }
    curva[3]=auxCurva;
    memset(&auxCurva, 0, sizeof(auxCurva));
/*
            printf("imprimiendo curva 4\n");
            for(uint8_t j=0;j<17;j++){
                printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[3].segmentos[j].inicio, curva[3].segmentos[j].pesoInicio);
            }*/

}
// --- Ejemplo: Escribir múltiples registros (0x65) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x65(uint8_t *request, size_t len){
    //printf("tarea 65\n");
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
    response[1] = 0x65;
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
    
    ESP_LOGI(TAG, "0x65 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    for (int i = 0; i<17; i++) {
        auxCurva.segmentos[i].inicio = request[7+(i*2)];
        auxCurva.segmentos[i].pesoInicio = request[8+(i*2)];
       // auxCurva2.segmentos[i+16].inicio = request[41+(i*2)];
       // auxCurva2.segmentos[i+16].pesoInicio = request[42+(i*2)];
    /*printf("indice:%d",i);
    printf("C1= %d\n", auxCurva.segmentos[i].inicio);
    printf("C1= %d\n", auxCurva.segmentos[i].pesoInicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].inicio);
    printf("C2= %d\n", auxCurva2.segmentos[i+16].pesoInicio);*/
    }
    curva[4]=auxCurva;
    memset(&auxCurva, 0, sizeof(auxCurva));

            /*printf("imprimiendo curva 4\n");
            for(uint8_t j=0;j<17;j++){
                printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[4].segmentos[j].inicio, curva[4].segmentos[j].pesoInicio);

}*/
}
// --- Ejemplo: Escribir múltiples registros (0x60) ---
//Función: escribir datos de tipo de curva
void modbus_slave_process_0x70(uint8_t *request, size_t len){
    //printf("holis 70\n");
    uint16_t reg_start  = (request[2] << 8) | request[3];
    uint16_t quantity   = (request[4] << 8) | request[5];
    //uint8_t byte_count  = request[6];
   /* if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Rango fuera de los registros disponibles");
        return;
    }*/
    for(int i = 0; i < quantity; i++) {
        uint16_t val = (request[7 + i*2] << 8) | request[8 + i*2];
        holding_registers[reg_start + i] = val;
    }
    uint8_t response[8];
    response[0] = SLAVE_ADDR;
    response[1] = 0x70;
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
    
    ESP_LOGI(TAG, "0x70 - Escribí %d registros desde %d", quantity, reg_start);
    ///copia en numero de caravana
    
    tiempoReal=0;
    for(int i = 0; i < 8; i++) {
        tiempoReal = tiempoReal | ((uint64_t)request[7+i] << (8 * (7 - i)));
    }

    now = tiempoReal;
    printf("timestamp tarea: %lld\n", now);
    localtime_r(&now, &ahora);
    printf("Fecha y hora actual tarea: %04d-%02d-%02d %02d:%02d:%02d\n",
           ahora.tm_year + 1900, ahora.tm_mon + 1, ahora.tm_mday,
           ahora.tm_hour, ahora.tm_min, ahora.tm_sec);


        ds1307_write_register(0x00, (decimal_to_bcd(ahora.tm_sec)));
    ds1307_write_register(0x01, (decimal_to_bcd(ahora.tm_min)));
    ds1307_write_register(0x02,(decimal_to_bcd(ahora.tm_hour)));
    ds1307_write_register(0x04,(decimal_to_bcd(ahora.tm_mday)));
    ds1307_write_register(0x05,(decimal_to_bcd(ahora.tm_mon + 1)));
    ds1307_write_register(0x06,(decimal_to_bcd(ahora.tm_year - 100)));
   

    ds1307_write_register(0x07,0x93);

   // printf("tiemo real del reloj %llx\n", tiempoReal);
    }
    
void modbus_slave_process_0x20(uint8_t *request, size_t len) 
{   
    int ultimo_valido= -1;
    uint16_t reg_start = (request[2] << 8) | request[3];
    uint16_t quantity  = (request[4] << 8) | request[5];
    
    if (reg_start + quantity > sizeof(holding_registers)/sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Solicitud fuera de rango");
        return;
    }
//printf("%d\n",request[1]);
//printf("modbus 80\n");
    

    // Buscar último índice con nombre válido (de adelante hacia atrás)
    for (int i = 0; i < 20; i++) {
        if (animal_tranfer[i].nombre[0]!='\0'){
            if (strcmp(animal_tranfer[i].nombre, "000000000000000") != 0){
                printf("entro aca %d / %s\n",i, animal_tranfer[i].nombre);
                ultimo_valido = i;
            }
        }
    }
    //printf("cantidad de animales: %d\n", ultimo_valido);

    if (ultimo_valido != -1) {
        // Copiar a auxiliar
        strcpy(animal_leido_AUX.nombre, animal_tranfer[ultimo_valido].nombre);
        animal_leido_AUX.fechaDispensado = animal_tranfer[ultimo_valido].fechaDispensado;
        animal_leido_AUX.pesoDispensado = animal_tranfer[ultimo_valido].pesoDispensado;

        // Borrar ese elemento

        printf("Último animal válido encontrado en índice %d: %s\n", 
               ultimo_valido, auxiliar.nombre);
    } else {
        printf("No se encontró ningún animal con datos válidos.\n");
    }

    uint8_t response[BUF_SIZE2];
    response[0] = request[0];
    response[1] = request[1];
    response[2] = request[2];
    response[3] = request[3];
    response[4] = request[4];
    response[5] = 17;
    response[6] = 33;
    // RESPONDER CON DATOS DEL ANIMAL LEIDO

    for(uint8_t i = 0; i < 16; i++) {
        response[i + 7] = animal_leido_AUX.nombre[i];
    }
    
    response[23] = (animal_leido_AUX.fechaDispensado >> 56 ) & 0xFF;
    response[24] = (animal_leido_AUX.fechaDispensado >> 48 ) & 0xFF;
    response[25] = (animal_leido_AUX.fechaDispensado >> 40 ) & 0xFF;
    response[26] = (animal_leido_AUX.fechaDispensado >> 32 ) & 0xFF;
    response[27] = (animal_leido_AUX.fechaDispensado >> 24 ) & 0xFF;
    response[28] = (animal_leido_AUX.fechaDispensado >> 16 ) & 0xFF;
    response[29] = (animal_leido_AUX.fechaDispensado >> 8 ) & 0xFF;
    response[30] = (animal_leido_AUX.fechaDispensado >> 0 ) & 0xFF;
    
    response[31] = animal_leido_AUX.pesoDispensado;

    uint16_t crc = modbus_crc16(response, 31);
    response[32] = crc & 0xFF;
    response[33] = crc >> 8;

    uart_flush_input(UART_NUM2);
    
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
   // printf("Enviando respuesta: ");
    uart_write_bytes(UART_NUM2, (const char *)response, 33);

    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_4, 0);
    //uart_write_bytes(UART_NUM2, (const char *)response, resp_len);
    ESP_LOGI(TAG, "Respondido 0x80 con %d registros", quantity);
    /*for(uint8_t j=0;j<20;j++){
       printf("ENVIO MODBUS:Nombre del animal: ");
    for(uint8_t i = 0; i < 16; i++) {
        printf("%c", animal_tranfer[j].nombre[i]);
    }
    printf("\n");
    printf("Fecha de dispensado: %lld\n", animal_tranfer[j].fechaDispensado);
    printf("Peso dispensado: %d\n", animal_tranfer[j].pesoDispensado);
    printf("numero de caravana: %d\n", SLAVE_ADDR);
    }*/

    strncpy(animal_tranfer[ultimo_valido].nombre, "000000000000000", sizeof(animal_tranfer[ultimo_valido].nombre));
    animal_tranfer[ultimo_valido].nombre[15] = '\0';
    animal_tranfer[ultimo_valido].fechaDispensado = 0;
    animal_tranfer[ultimo_valido].pesoDispensado = 0;
    strncpy(animal_leido_AUX.nombre, "000000000000000", sizeof(animal_tranfer[ultimo_valido].nombre));
    animal_leido_AUX.nombre[15] = '\0';
    animal_leido_AUX.fechaDispensado = 0;
    animal_leido_AUX.pesoDispensado = 0;


            //strcpy(animal_leido[ultimo_valido].nombre, "000000000000000");
        //animal_leido[ultimo_valido].pesoDispensado = 0;
        //animal_leido[ultimo_valido].fechaDispensado = 0;

   /* printf("ENVIO MODBUS:Nombre del animal: ");
    for(uint8_t i = 0; i < 16; i++) {
        printf("%c", animal_leido_AUX.nombre[i]);
    }
    printf("\n");
    printf("Fecha de dispensado: %lld\n", animal_leido_AUX.fechaDispensado);
    printf("Peso dispensado: %d\n", animal_leido_AUX.pesoDispensado);
    printf("numero de caravana: %d\n", SLAVE_ADDR);
*/
/*for(uint8_t j=0;j<20;j++){
       printf("ENVIO MODBUS:Nombre del animal: ");
    for(uint8_t i = 0; i < 16; i++) {
        printf("%c", animal_tranfer[j].nombre[i]);
    }
    printf("\n");
    printf("Fecha de dispensado: %lld\n", animal_tranfer[j].fechaDispensado);
    printf("Peso dispensado: %d\n", animal_tranfer[j].pesoDispensado);
    printf("numero de caravana: %d\n", SLAVE_ADDR);
    }
    */
}


