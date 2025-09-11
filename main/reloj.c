#include <sys/time.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "reloj.h"
#include "estructuras.h"


#define I2C_SLAVE_ADDR 0x68
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define TIMEOUT_MS 1000

/////inicializacion de la comunicacion I2C (SDA = GPIO21)(SCL = GPIO22)/////
void init_i2c() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}


/////funcion para leer los registros del ds1307/////
esp_err_t ds1307_read_register(uint8_t reg_addr, uint8_t *data) {
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, &reg_addr, 1, pdMS_TO_TICKS(TIMEOUT_MS));
    if (err != ESP_OK) return err;
    return i2c_master_read_from_device(I2C_NUM_0, I2C_SLAVE_ADDR, data, 1, pdMS_TO_TICKS(TIMEOUT_MS));
}

/////funcion para escribir en los registros del ds1307///// 
esp_err_t ds1307_write_register(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buffer[2] = { reg_addr, data };
    
    return i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, write_buffer, sizeof(write_buffer), pdMS_TO_TICKS(TIMEOUT_MS));
}

/////funcion para de BDC a decimal/////
uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint8_t decimal_to_bcd(uint8_t bcd) {
    uint8_t bcd1 = (bcd / 10) << 4; // Desplazar el dígito de las decenas a la posición alta
    uint8_t bcd2 = bcd % 10; // Obtener el dígito de las unidades
    return (bcd1 | bcd2); // Combinar ambos dígitos
}

/////funcion para leer fecha y hora en tiempo real con tratamiento de errores/////
esp_err_t read_time() {
    printf("llego a leer hora\n");
    uint8_t seconds, minutes, hours,day,month,year;
    int retries = 3;
    esp_err_t result;

    while (retries--) {
        result = ds1307_read_register(0x00, &seconds);
        result |= ds1307_read_register(0x01, &minutes);
        result |= ds1307_read_register(0x02, &hours);
        result |= ds1307_read_register(0x04, &day);
        result |= ds1307_read_register(0x05, &month);
        result |= ds1307_read_register(0x06, &year);
        if (result == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("llego a leer hora 2\n");

    if (result != ESP_OK) {
      //  errores.errorHoraDS1307 = true;
        printf("Error al leer la hora del DS1307 tras múltiples intentos\n");
        return ESP_FAIL;
    }
    ///si se logra leer la hora se limpia el error y se guarda en la estructura de tiempo
    ///y lo paso a un formato de tiempo
    //errores.errorHoraDS1307 = false;
    ahora.tm_hour = bcd_to_decimal(hours);
    ahora.tm_min = bcd_to_decimal(minutes);
    ahora.tm_sec = bcd_to_decimal(seconds);
    ahora.tm_mday = bcd_to_decimal(day);
    ahora.tm_mon = bcd_to_decimal(month) - 1;
    ahora.tm_year = bcd_to_decimal(year) + 100;
    return ESP_OK;
}