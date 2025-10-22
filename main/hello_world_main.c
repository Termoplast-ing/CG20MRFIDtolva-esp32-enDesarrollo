   /////Carga de Librerias/////
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include "estructuras.h"
#define TAG "UART_EVENT_TASK"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "modbus_crc.h"
#include "modbus_slave_raw.h"
#include "esp_err.h"
#include "reloj.h"
/////Definicion de Pines y Variables/////

#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_25
#define RXD_PIN GPIO_NUM_26
#define RTS_PIN GPIO_NUM_27
#define BUF_SIZE 1024
//#define I2C_SLAVE_ADDR 0x68
//#define I2C_SDA GPIO_NUM_21
//#define I2C_SCL GPIO_NUM_22
//#define TIMEOUT_MS 1000
#define jaula GPIO_NUM_34
#define tolva GPIO_NUM_35
#define motor GPIO_NUM_32
#define valvula GPIO_NUM_33
#define rele GPIO_NUM_14
#define UART_NUM2 UART_NUM_2
//#define UART_FIFO_LEN 128
//#define BUF_SIZE2 2048


/////Definicion de Estructuras/////
/*
///Estructura para almacenar los errores del sistema///
typedef struct {
    bool errorHoraDS1307;
    bool errorUART;
    bool tolvaVacia;
    bool jaulaTrabada;
    bool errorMotor;
    bool errorAgua;
    bool caravanaNoReconocida;
} error_status_t;



/////Estructura para almacenar los datos de los animales en el corral/////
typedef struct {
    char nombre[16];
    uint8_t tipoCurva;
    uint8_t pesoDosis;
    time_t fechaServicio;
    uint8_t indiceCorporal;
    bool agua;
    uint8_t cantDosis;
    uint16_t intervaloMin;    
} data_animal;

/////Estructura para almacenar los datos de los animales leidos por la antena RFID/////
typedef struct{
    char nombre[15];
    time_t fechaDispensado;
    uint8_t pesoDispensado;  
} data_animal_leido;

/////Estructura para almacenar los segmentos de las curvas de crecimiento/////
typedef struct{
    uint8_t inicio;
    uint8_t pesoInicio;
} segmento;

/////Estructura para almacenar las curvas de crecimiento/////
typedef struct{
    segmento segmentos[17];  
} tipo_curva;*/



/////Definicion de Variables/////
error_status_t errores = {0};

//data_animal_leido animal_leido[300];
//tipo_curva curva[10];
//data_animal corral[25]; // Inicializar el corral con 25 animales
uint32_t diaGestacion=0;
time_t now;
struct tm ahora;
time_t timeStampDatos=1;
time_t timeStampCentralDatos=1;
time_t timeStampConfig=1;
time_t timeStampCentralConfig=1;
time_t timeStampCurva=1;
time_t timeStampCentralCurva=1;
uint8_t actual=0;
uint8_t diaAnterior=0;
time_t horaLecturaAnterior=0;
char caravanaAnterior[16] = "000000000000000";
uint8_t calibracionMotor=1;
uint8_t calibracionAgua=1;
QueueHandle_t uart_queue;
char caravana[16] = "000000000000001";
uint16_t dispensado=1;
uint8_t slave_addr = 0x02;
uint8_t funcionCom=0; 
float corporal=1;
float indiceCorporal[3]={0.5,1,2};




/*void inicializarDatos(){
    for(uint8_t i=0;i<25;i++){
        corral[i].nombre[0]='\0';
        corral[i].tipoCurva=0xff;
        corral[i].pesoDosis=0;
        corral[i].fechaServicio=0;
        corral[i].indiceCorporal=0;
        corral[i].agua=false;
        corral[i].cantDosis=0;
        corral[i].intervaloMin=0;
    }
}*/
/*void inicializarConfig(){
    for(uint8_t i=0;i<5;i++){
        configuracion[i]=0;
    }
}*/


void guardarPersonasEnNVS(){}
/////funcion que pide los datos del corral por uart a la central y los deja en la RAM y una copia en la ROM/////
void traerDatos(){
    //comunicarme con la central y traerme los datos del corral
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    err = nvs_set_blob(handle, "corral", corral, sizeof(corral));
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error guardando blob en NVS");
    } else {
        ESP_LOGI("NVS", "Arreglo de personas guardado correctamente");
        nvs_commit(handle);  // Confirmar cambios
    }
    nvs_close(handle);
}

void ROMtoRAMDatos(){
    nvs_handle_t handle;
    esp_err_t err;
    size_t required_size = 0;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    err = nvs_get_blob(handle, "corral", NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("NVS", "No se encontraron datos, inicializando con valores por defecto");
        guardarPersonasEnNVS();
    } else if (required_size == sizeof(corral)) {
        // Leer blob
        err = nvs_get_blob(handle, "corral", corral, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Arreglo de personas cargado correctamente");
        } else {
            ESP_LOGE("NVS", "Error leyendo blob de NVS");
        }
    }
    nvs_close(handle);
}
/* PRUEBA DE ESCRITURA EN NVS 
void test_nvs_corral() {
    printf("=== TEST NVS CORRAL ===\n");

    // 1. Guardar datos de ejemplo
    strncpy(corral[0].nombre, "123456789012345", sizeof(corral[0].nombre) - 1);
    corral[0].nombre[sizeof(corral[0].nombre) - 1] = '\0';
    corral[0].tipoCurva = 2;
    corral[0].pesoDosis = 55;
    corral[0].fechaServicio = 1740970800;
    corral[0].indiceCorporal = 3;
    corral[0].agua = true;
    corral[0].cantDosis = 2;
    corral[0].intervaloMin = 90;

    traerDatos(); // Guarda en NVS

    // 2. Limpiar RAM
    memset(corral, 0, sizeof(corral));

    // 3. Leer desde NVS
    ROMtoRAMDatos();

    // 4. Mostrar resultado
    printf("Nombre: %s\n", corral[0].nombre);
    printf("Tipo curva: %d\n", corral[0].tipoCurva);
    printf("Peso dosis: %d\n", corral[0].pesoDosis);
    printf("Fecha servicio: %lld\n", corral[0].fechaServicio);
    printf("Indice corporal: %d\n", corral[0].indiceCorporal);
    printf("Agua: %d\n", corral[0].agua);
    printf("Cantidad dosis: %d\n", corral[0].cantDosis);
    printf("Intervalo min: %d\n", corral[0].intervaloMin);
}*/
/////funcion que pide los datos de configuracion por uart a la central y los deja en la RAM y una copia en la ROM/////
void traerConfig(){
    //comunicarme con la central y traerme los datos de configuracion
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    //err = nvs_set_blob(handle, "configuracion", configuracion, sizeof(configuracion));
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error guardando blob en NVS");
    } else {
        ESP_LOGI("NVS", "Arreglo de personas guardado correctamente");
        nvs_commit(handle);  // Confirmar cambios
    }
    nvs_close(handle);
}

/////funcion que pide los datos de las curvas por uart a la central y los deja en la RAM y una copia en la ROM/////
void traerCurva(){
    //comunicarme con la central y traerme los datos de los tipos de curva
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    err = nvs_set_blob(handle, "curva", curva, sizeof(curva));
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error guardando blob en NVS");
    } else {
        ESP_LOGI("NVS", "Arreglo de personas guardado correctamente");
        nvs_commit(handle);  // Confirmar cambios
    }
    nvs_close(handle);
}

/////funcion que obtiene los datos del corral desde la ROM y los copia en la RAM/////


/////funcion que obtiene los datos de configuracion desde la ROM y los copia en la RAM/////
void ROMtoRAMConfig(){
    nvs_handle_t handle;
    esp_err_t err;
    size_t required_size = 0;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    err = nvs_get_blob(handle, "configuracion", NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("NVS", "No se encontraron datos, inicializando con valores por defecto");
        guardarPersonasEnNVS();
    } else if (required_size == sizeof(configuracion)) {
        // Leer blob
        err = nvs_get_blob(handle, "configuracion", &configuracion, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Arreglo de personas cargado correctamente");
                        // ---- PRINTS PARA VERIFICAR DATOS ----
            printf("---- Datos de configuracion leidos de NVS ----\n");
            printf("Motor calibracion: %d\n", configuracion.calibracionMotor);
            printf("Agua calibracion: %d\n", configuracion.calibracionAgua);
            printf("Peso animal desconocido: %d\n", configuracion.pesoAnimalDesconocido);
            printf("Caravana libre 1: %s\n", configuracion.caravanaLibre1);
            printf("Caravana libre 2: %s\n", configuracion.caravanaLibre2);
            printf("Caravana libre 3: %s\n", configuracion.caravanaLibre3);
            printf("Caravana libre 4: %s\n", configuracion.caravanaLibre4);
            printf("Caravana libre 5: %s\n", configuracion.caravanaLibre5);
            printf("--------------------------------------------\n");
        } else {
            ESP_LOGE("NVS", "Error leyendo blob de NVS");
        }
    }
    nvs_close(handle);
}

/////funcion que obtiene los datos de la curvas desde la ROM y los copia en la RAM/////
void ROMtoRAMCurva(){
    nvs_handle_t handle;
    esp_err_t err;
    size_t required_size = 0;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS para leer");
        return;
    }

    err = nvs_get_blob(handle, "curva", NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("NVS", "No se encontraron datos, inicializando con valores por defecto");
        guardarPersonasEnNVS();
    } else if (required_size == sizeof(curva)) {
        // Leer blob
        err = nvs_get_blob(handle, "curva", curva, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Arreglo de personas cargado correctamente");
        } else {
            ESP_LOGE("NVS", "Error leyendo blob de NVS");
        }
    }
    nvs_close(handle);
}

/////funcion que compara los timestamp con la central para ver si necesita actualizar datos/////

/////funcion apagar motor y electrovalula despues de tiempo seteado///// 
void apagado_motor(void *pvParameters) {
    float tiempoA=0;
    float tiempoM=0;
    tiempoM=((float)((float)dispensado)*configuracion.calibracionMotor);
    tiempoA=tiempoM*(((float)configuracion.calibracionAgua)/((float)configuracion.calibracionMotor));
    printf("dispensado %d\n",dispensado);
    printf("calibracionMotor %d\n",configuracion.calibracionMotor);
    printf("calibracionAgua %d\n",configuracion.calibracionAgua);
    printf("tiempoAgua %d\n",(int)tiempoA);
    printf("tiempoMotor %d\n",(int)tiempoM);
    
    if(tiempoA>tiempoM){
        printf("arrancaTM");
        vTaskDelay(pdMS_TO_TICKS((int)(tiempoM*1000)));
        gpio_set_level(motor, 1);
        printf("arrancaTA");
        vTaskDelay(pdMS_TO_TICKS((int)((tiempoA-tiempoM)*1000)));
        gpio_set_level(valvula,1);
        printf("arrancaTM");
    }else{
        vTaskDelay(pdMS_TO_TICKS((int)(tiempoA*1000)));
        gpio_set_level(valvula, 1);
        vTaskDelay(pdMS_TO_TICKS((int)((tiempoM-tiempoA)*1000)));
        gpio_set_level(motor,1);
    }
    vTaskDelete(NULL);
}

/////Funcion para reportar los errores del sistema/////
void reportarErrores() {
    if (errores.errorHoraDS1307) printf("Error en el DS1307\n");
    if (errores.errorUART) printf("Error en la UART\n");
    if (errores.tolvaVacia) printf("Tolva vacía\n");
    if (errores.jaulaTrabada) printf("Jaula trabada\n");
    if (errores.errorMotor) printf("Error en el motor\n");
    if (errores.errorAgua) printf("Error en la electroválvula de agua\n");
    if (errores.caravanaNoReconocida) printf("Caravana no reconocida\n");
}

/////Funcion para calcular el dia de gestacion///// 
uint8_t dia_gestacion(uint8_t indice){
    printf("indice %d\n",indice);
    printf("fecha servicio %lld\n",corral[indice].fechaServicio);
    time_t fechaInseminacion = corral[indice].fechaServicio;
    now = mktime(&ahora);
    if(fechaInseminacion>now){
        diaGestacion = 0;
        return diaGestacion;
    }else{
        u_int64_t seconds_diff = difftime(now, fechaInseminacion);
        if(seconds_diff==0){
            seconds_diff=1;
        }
        diaGestacion = seconds_diff / (60 * 60 * 24);
        diaGestacion=diaGestacion+1;
        if(diaGestacion>120){
            diaGestacion=120;
        }
        return diaGestacion;
    }
}

/////Funcion para calcular el peso de la dosis/////
uint16_t calcular_peso(uint8_t indice){
    uint8_t diaGestacionAUX=dia_gestacion(indice);
    printf("diaGestacionAUX: %d\n", diaGestacionAUX);
    uint8_t diaInicio=0;
    uint8_t diaFin=0;
    uint8_t pesoInicio=0;
    uint8_t pesoFin=0;
    uint8_t curvaUsada=corral[indice].tipoCurva;
    float pendiente = 0.0;
    float denominador = 1.0;
    uint8_t pesoDosis=0;
    float pesoTotal=0;
    if(indice<20){    
        for(uint8_t i=0;i<17;i++){
            printf("indice:%d\n", indice);
            printf("tipocurva:%d\n", corral[indice].tipoCurva);
            printf("curva usa:%d\n", curva[corral[indice].tipoCurva].segmentos[i].inicio);
            if (curva[curvaUsada].segmentos[i].inicio>diaGestacionAUX){
                diaFin=curva[corral[indice].tipoCurva].segmentos[i].inicio;
                pesoFin=curva[corral[indice].tipoCurva].segmentos[i].pesoInicio;
                diaInicio=curva[corral[indice].tipoCurva].segmentos[i-1].inicio;
                pesoInicio=curva[corral[indice].tipoCurva].segmentos[i-1].pesoInicio;
                break;
            }
        }
        corporal=indiceCorporal[corral[indice].indiceCorporal];
        denominador=diaFin-diaInicio;
        pendiente=((float)(pesoFin-pesoInicio)/(float)(denominador));
        pesoDosis=(pendiente*(diaGestacionAUX-diaInicio))+pesoInicio;
        printf("pesoDosis: %d\n", pesoDosis);
        printf("cantDosis: %d\n", corral[indice].cantDosis);
        printf("indiceCorporal: %d\n", corral[indice].indiceCorporal);
        printf("pesoAnimal: %d\n", corral[indice].pesoDosis);

        pesoTotal=(float)((corporal*(((float)pesoDosis/100.0f)*((float)corral[indice].pesoDosis))/(corral[indice].cantDosis)));
        printf("pesoTotal: %f\n", pesoTotal);
    }else{
        if(indice==20){
            pesoTotal=(float)(configuracion.pesoAnimalDesconocido/10.0f);
            printf("pesoTotal Animal desconocido: %f\n", pesoTotal);
        }else{
            pesoTotal=0.1f;
        }
    }
    return pesoTotal;
}

///// funcion para guardar el registro del animal leido y la accion del sistema en un arreglo que es el/////
/////que se le va  a pasar al controlador central///// 
void guardar_registro(uint8_t indice,struct tm ahora){
    for(uint16_t i=0;i<100;i++){
        if(animal_leido[i].nombre[0]=='\0'){
            strcpy(animal_leido[i].nombre, corral[indice].nombre);
            animal_leido[i].fechaDispensado=mktime(&ahora);
            animal_leido[i].pesoDispensado=dispensado;
                        // *** PRINT DE DEPURACIÓN ***
            printf("[GUARDAR] Animal guardado en índice %d -> Nombre: %s, Peso: %d, Fecha: %lld\n",
                   i,
                   animal_leido[i].nombre,
                   animal_leido[i].pesoDispensado,
                   animal_leido[i].fechaDispensado);
            break;
        }
    }
    for(uint16_t i=0;i<20;i++){
        if(animal_tranfer[i].nombre[0]=='\0'){
            strcpy(animal_tranfer[i].nombre, corral[indice].nombre);
            animal_tranfer[i].fechaDispensado=mktime(&ahora);
            animal_tranfer[i].pesoDispensado=dispensado;
                        // *** PRINT DE DEPURACIÓN ***
            //printf("[GUARDAR] Animal guardado en índice %d -> Nombre: %s, Peso: %d, Fecha: %lld\n",
               //    i,
                 //  animal_leido[i].nombre,
                  // animal_leido[i].pesoDispensado,
                  // animal_leido[i].fechaDispensado);
            break;
        }
    }
}

/////Funcion para mover el motor para tirar comida y abrir la electrovalvula segun la configuracion del//////
/////del sistema y de los animales/////
void dispensar_alimento(uint8_t indice){
    //motor
    gpio_set_level(motor,0);
    if(corral[indice].agua>0){
        gpio_set_level(valvula,0);
    }
    printf("encendido");
    xTaskCreate(apagado_motor, "apagado_motor", 2048, NULL, 6, NULL);
}
    
///// funcion a la que acude el programa despues de leer la caravana en la antena/////
void debe_comer(uint8_t indice){
    uint8_t dosisDadas=0;
    uint8_t pesoDado=0;
    time_t ultimaDosis=0;
    
   // if(indice==20){
     //   memcpy(corral[21].nombre,configuracion.caravanaLibre1,sizeof(configuracion.caravanaLibre1));
    
    ///busco en los registro del dia si ya comio el animal
    for(uint8_t j=0;j<100;j++){
        if(indice==20){
            if((strcmp(animal_leido[j].nombre,corral[20].nombre) == 0)){
            dosisDadas ++;
            ultimaDosis = animal_leido[j].fechaDispensado;
            pesoDado = animal_leido[j].pesoDispensado;
        }
        }else{
        if((strcmp(animal_leido[j].nombre, corral[indice].nombre) == 0)){
            dosisDadas ++;
            ultimaDosis = animal_leido[j].fechaDispensado;
            pesoDado = animal_leido[j].pesoDispensado;
        }
    }
        
    }
    printf("dosisDadas: %d\n", dosisDadas);
    ///si no comio llamo a la funcion que calcula el peso de la dosis
    if(dosisDadas==0){
        if(indice<20){
            printf("El animal %s no ha comido hoy. Preparando dosis.\n", corral[indice].nombre);
        printf("pesoComidaAnimal: %d\n",corral[indice].pesoDosis);
        printf("diaGestacion: %d\n",dia_gestacion(indice));
        dispensado=calcular_peso(indice);
        printf("PESO dispensado: %d\n",dispensado);
        }else{
            dispensado=(uint16_t)corral[indice].pesoDosis;
        }
    }else{
        ///si ya comio verifico si ya paso el tiempo para volver a darle de comer
        if((dosisDadas<corral[indice].cantDosis)&&(((uint64_t)difftime(now, ultimaDosis))>((uint64_t)(corral[indice].intervaloMin)*60*60))){
            dispensado=(uint16_t)pesoDado;
           
            
            
            
        }else{
            dispensado=0;
        }
    }

    ///si no hay que dar de comer guardo el registro en el arreglo de registros para saber que el animal
    ///entro pero no se le dio comida
    if(dispensado==0){
        if(indice<21){
            //guardar_registro(indice,ahora);
            printf("No se le da de comer al animal %s\n", corral[indice].nombre);
        }
    ///si hay que dar de comer guardo el registro en el arreglo de registros para saber que el animal
    ///entro y se le dio comida y llamo a la funcion que dispensa la comida
    }else{
        printf("estoy aca\n");
        if(indice<25){
            if(indice<21){
                guardar_registro(indice,ahora);
            };
            printf("estoy aca guardando\n");
            dispensar_alimento(indice);
        }
    }
}

/////funcion para atender la lectura de un animal
void atencion_lectura(){
    ///verifico que si se cambio el dia de gestacion se borren los registros de los animales
uint8_t indico=0;
    actual=dia_gestacion(0);
    printf("actual %d\n",actual);
    printf("diaAnterior %d\n",diaAnterior);
    if(actual>diaAnterior){
        for(uint8_t i=0;i<100;i++){
            animal_leido[i].nombre[0]='\0';
            animal_leido[i].fechaDispensado=0;
            animal_leido[i].pesoDispensado=0;
        }
        diaAnterior=actual;
    }
    //indentifico que posicion del la variable corral ocupa la caravana leida
    
    

    ///verifico si la caravana leida es de un animal conocido
    for (uint8_t i = 0; i <= 25; i++){
        indico=i;
        printf("%d\n",indico);
        printf("1:%s\n",corral[i].nombre);
        printf("2:%s\n",caravana);
        if(strcmp(corral[i].nombre, caravana) == 0){
            
            debe_comer(i);
            break;
        }
    }
    indico++;
    if(indico==26){
strcpy(corral[20].nombre, caravana);
        printf("Caravana no reconocida: %s\n", caravana);
        printf("DATOS ANIMAL %d\n", 20);
            printf("nombre: %s\n", corral[20].nombre);
            printf("tipoCurva: %d\n", corral[20].tipoCurva);
            printf("pesoDosis: %d\n", corral[20].pesoDosis);
            printf("fechaServicio: %lld\n", corral[20].fechaServicio);   
            printf("indiceCorporal: %d\n", corral[20].indiceCorporal);
            printf("agua: %d\n", corral[20].agua);   
            printf("cantDosis: %d\n", corral[20].cantDosis);
            printf("intervaloMin: %d\n", corral[20].intervaloMin);
            printf("========================================\n");

        debe_comer(20);
    strcpy(corral[20].nombre, "999999999999999");
        corral[20].nombre[15] = '\0';
    }
}

/////inicializacion de la comunicacion I2C (SDA = GPIO21)(SCL = GPIO22)/////
/*void init_i2c() {
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

/////funcion para leer fecha y hora en tiempo real con tratamiento de errores/////
esp_err_t read_time() {
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

    if (result != ESP_OK) {
        errores.errorHoraDS1307 = true;
        printf("Error al leer la hora del DS1307 tras múltiples intentos\n");
        return ESP_FAIL;
    }
    ///si se logra leer la hora se limpia el error y se guarda en la estructura de tiempo
    ///y lo paso a un formato de tiempo
    errores.errorHoraDS1307 = false;
    ahora.tm_hour = bcd_to_decimal(hours);
    ahora.tm_min = bcd_to_decimal(minutes);
    ahora.tm_sec = bcd_to_decimal(seconds);
    ahora.tm_mday = bcd_to_decimal(day);
    ahora.tm_mon = bcd_to_decimal(month) - 1;
    ahora.tm_year = bcd_to_decimal(year) + 100;
    return ESP_OK;
}
*/
/////Monitorear sensores de tolva y jaula/////
void verificarSensores() {
    errores.tolvaVacia = gpio_get_level(tolva) == 0;
    errores.jaulaTrabada = gpio_get_level(jaula) == 0;
}

/////atencion de la interrupcion por lectura de la antena RFID/////
void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    int len = uart_read_bytes(UART_NUM, data, event.size, pdMS_TO_TICKS(100));
                    if (len > 0) {
                        data[len] = '\0';
                        //printf("Dato recibido: %s\n", data);
                        caravana[0] = data[9];
                        caravana[1] = data[10];
                        caravana[2] = data[11];
                        caravana[3] = data[12];
                        caravana[4] = data[13];
                        caravana[5] = data[14];
                        caravana[6] = data[15];
                        caravana[7] = data[16];
                        caravana[8] = data[17];
                        caravana[9] = data[18];
                        caravana[10] = data[19];
                        caravana[11] = data[20];
                        caravana[12] = data[21];
                        caravana[13] = data[22];
                        caravana[14] = data[23];
                        caravana[15] = '\0';
                        //printf("Caravana: %s\n", caravana);
                        //printf("[UART] Caravana procesada -> %s\n", caravana);
                        //leo la hora real y comparo si es el mismo animal que el anterior y pasaron
                        //mas de 5 minutos lo tomo como valido sino los descarto y si es diferente 
                        //lo tomo como valido independientemente del tiempo
                        read_time();
                        now=mktime(&ahora);
                        //printf("Hora: %d:%d:%d\n", ahora.tm_hour, ahora.tm_min, ahora.tm_sec);
                        //printf("%lld\n",now);  
                        //printf("%s\n",caravana);
                        //printf("%s\n",corral[1].nombre);
                        //printf("%s\n",caravanaAnterior);          
                        if(strcmp(caravana,caravanaAnterior) != 0){
                            printf("repite animal\n");
                            strcpy(caravanaAnterior, caravana);
                            horaLecturaAnterior=mktime(&ahora);
                            atencion_lectura();     
                        }else{
                            printf("otro animal\n");
                            now = mktime(&ahora);
                            if(difftime(now, horaLecturaAnterior) > 300) {
                                atencion_lectura();
                                horaLecturaAnterior=now;
                            }
                        }
                        //$A0112OKD98200045957891832#
                    }
                //tratatamiento de errores de la UART por desbordamiento de buffer    
                        break;
                case UART_FIFO_OVF:
                    printf("¡Desbordamiento de buffer UART!\n");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    printf("¡Buffer UART lleno!\n");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                default:
                    break;
                
            }
        }
    }
}

/////incicializa la comunicacion UART2 (RX=GPIO26) (TX=GPIO25) (RTS=GPIO27)/////
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
}

///// Verificar UART y limpiar error después de 30 segundos sin errores/////
void verificarUART(uart_event_t event) {
    static int64_t lastErrorTime = 0;
    int64_t now = esp_timer_get_time() / 1000;

    switch (event.type) {
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
        case UART_BREAK:
        case UART_PARITY_ERR:
        case UART_FRAME_ERR:
            errores.errorUART = true;
            lastErrorTime = now;
            printf("Error UART detectado: %d\n", event.type);
            break;
        case UART_DATA:
            if (event.size > 0) errores.errorUART = false;
            break;
        default:
            break;
    }

    if (errores.errorUART && (now - lastErrorTime > 30000)) {
        errores.errorUART = false;
        printf("Error UART limpiado tras 30 segundos sin fallos\n");
    }
}

void active_message_task(void *pvParameters) {
    while (1) {
        //////leo timestamp de los datos del corral si es necesario actualizo con los datos de la central 
        /////sino cargo lo que tengo en la rom en la memoria
        funcionCom=0x01;
       // actualizar();
        /*if(pasarDatos==0x00){
            holding_registers[0]=0x01;
            pasarDatos=1;
        }
        if((pasarDatos==0x01)){
            if(datosActual){
                //comparo los time stamp para ver si pido los datos
            }else{
                if(configActual){
                    //comparo los time stamp para ver si pido los datos de config
                }else{
                    if(curvaActual){
                        //comparo los time stamp para ver si pido los datos de curva
                    }else{
                        holding_registers[0]=0x02;
                        pasarDatos=2;
                    }
                }
            }*/
        
        
        //////leo timestamp de los datos que tengo en el registro de los animales leidos y comparo con 
        /////con lo que esta en la central si es necesario se envian los datos
        printf("funcionando\n");
        
      for(uint8_t j=0; j<20; j++){
            printf("DATOS ANIMAL %d\n", j);
            printf("nombre: %s\n", corral[j].nombre);
            printf("tipoCurva: %d\n", corral[j].tipoCurva);
            printf("pesoDosis: %d\n", corral[j].pesoDosis);
            printf("fechaServicio: %lld\n", corral[j].fechaServicio);   
            printf("indiceCorporal: %d\n", corral[j].indiceCorporal);
            printf("agua: %d\n", corral[j].agua);   
            printf("cantDosis: %d\n", corral[j].cantDosis);
            printf("intervaloMin: %d\n", corral[j].intervaloMin);
            printf("========================================\n");

        }

        /// DATOS DE CONFIGURACION
        /*printf("Caravana Libre 1 :");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%c", configuracion.caravanaLibre1[i]);
        }
        printf("\n");
        printf("Caravana Libre 2 :");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%c", configuracion.caravanaLibre2[i]);
        }
        printf("\n");
        printf("Caravana Libre 3 :");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%c", configuracion.caravanaLibre3[i]);
        }
        printf("\n");
        printf("Caravana Libre 4 :");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%c", configuracion.caravanaLibre4[i]);
        }
        printf("\n");
        printf("Caravana Libre 5 :");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%c", configuracion.caravanaLibre5[i]);
        }
        printf("\n");
        printf("Calibracion Motor: %d\n", configuracion.calibracionMotor);
        printf("Calibracion Agua: %d\n", configuracion.calibracionAgua);
        printf("peso Dosis desconocidos: %d\n", configuracion.pesoAnimalDesconocido);*/
       //DATOS DE CURVAS

        /*for(uint8_t i=0;i<5;i++){
            printf("imprimiendo curva %d\n", i);
            for(uint8_t j=0;j<18;j++){
                printf("Segmento %d: Inicio: %d, Peso Inicio: %d\n", j, curva[i].segmentos[j].inicio, curva[i].segmentos[j].pesoInicio);
            }
        }*/
      /*  for (int i = 0; i < 100; i++) {
            if (strcmp(animal_leido[i].nombre, "000000000000000") != 0 && animal_leido[i].nombre[0] != '\0') {
                printf("[LISTA] Animal %d -> Nombre: %s, Peso: %d, Fecha: %lld\n",
                i,
                animal_leido[i].nombre,
                animal_leido[i].pesoDispensado,
                animal_leido[i].fechaDispensado);
    }
}
for (int i = 0; i < 20; i++) {
            if (strcmp(animal_tranfer[i].nombre, "000000000000000") != 0 && animal_tranfer[i].nombre[0] != '\0') {
                printf("[LISTA] Animal %d -> Nombre: %s, Peso: %d, Fecha: %lld\n",
                i,
                animal_tranfer[i].nombre,
                animal_tranfer[i].pesoDispensado,
                animal_tranfer[i].fechaDispensado);
    }
}*/
                // Mostrar configuración actual
       printf("=== CONFIGURACION ACTUAL ===\n");
        printf("Caravana Libre 1: %s\n", configuracion.caravanaLibre1);
        printf("Caravana Libre 2: %s\n", configuracion.caravanaLibre2);
        printf("Caravana Libre 3: %s\n", configuracion.caravanaLibre3);
        printf("Caravana Libre 4: %s\n", configuracion.caravanaLibre4);
        printf("Caravana Libre 5: %s\n", configuracion.caravanaLibre5);
        printf("Calibracion Motor: %d\n", configuracion.calibracionMotor);
        printf("Calibracion Agua: %d\n", configuracion.calibracionAgua);
        printf("Peso Animal Desconocido: %d\n", configuracion.pesoAnimalDesconocido);
        printf("============================\n");
        printf("Caravana leida: %s\n", caravana);
        read_time();
        printf("Hora actual: %d:%d:%d\n", ahora.tm_hour, ahora.tm_min, ahora.tm_sec);
        printf("Fecha actual: %d/%d/%d\n", ahora.tm_mday, ahora.tm_mon + 1, ahora.tm_year + 1900);
        printf("Timestamp actual: %lld\n", mktime(&ahora));
        verificarSensores();
        reportarErrores();
        vTaskDelay(pdMS_TO_TICKS(60000)); // 10 minutos
    }
}

void app_main(void) {
 //   printf(">>> Arrancando app_main <<<\n");

 vTaskDelay(pdMS_TO_TICKS(60000));
    gpio_set_direction(motor, GPIO_MODE_OUTPUT);
    gpio_set_direction(valvula, GPIO_MODE_OUTPUT);
    gpio_set_level(valvula,1);
    gpio_set_level(motor,1);
    //test_nvs_corral();//(FUENCION PARA PROBAR LA LECTURA Y ESCRITURA EN NVS)
    modbus_slave_init();
  //  cargar_datos_prueba(animal_leido,12);
    inicializar_curvas();
esp_err_t err;
    /////leer timestamp de la central y compararlos con los de la ROM


    ///// Inicializar NVS NO SE VUELVE A EJECUTAR

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error inicializando NVS");
        return;
    }

    nvs_handle_t handle;
    

    err = nvs_open("TimeStamp", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error abriendo NVS");
        return;
    }

    err = nvs_get_i64(handle, "timeStampDatos", &timeStampDatos);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        timeStampDatos=0;
        err = nvs_set_i64(handle, "timeStampDatos", timeStampDatos);
        if(err != ESP_OK) {
            ESP_LOGE("NVS", "Error guardando valor (%s)", esp_err_to_name(err));
        } else {
            err = nvs_commit(handle);
            if (err != ESP_OK) {
                ESP_LOGE("NVS", "Error al hacer commit (%s)", esp_err_to_name(err));
            } else {
                ESP_LOGI("NVS", "Valor guardado correctamente");
            }
        }
    }

    err = nvs_get_i64(handle, "timeStampConfig", &timeStampConfig);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        timeStampConfig=0;
        err = nvs_set_i64(handle, "timeStampConfig", timeStampConfig);
        if(err != ESP_OK) {
            ESP_LOGE("NVS", "Error guardando valor (%s)", esp_err_to_name(err));
        } else {
            err = nvs_commit(handle);
            if (err != ESP_OK) {
                ESP_LOGE("NVS", "Error al hacer commit (%s)", esp_err_to_name(err));
            } else {
                ESP_LOGI("NVS", "Valor guardado correctamente");
            }
        }
    }

    err = nvs_get_i64(handle, "timeStampCurva", &timeStampCurva);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        timeStampCurva=0;
        err = nvs_set_i64(handle, "timeStampCurva", timeStampCurva);
        if(err != ESP_OK) {
            ESP_LOGE("NVS", "Error guardando valor (%s)", esp_err_to_name(err));
        } else {
            err = nvs_commit(handle);
            if (err != ESP_OK) {
                ESP_LOGE("NVS", "Error al hacer commit (%s)", esp_err_to_name(err));
            } else {
                ESP_LOGI("NVS", "Valor guardado correctamente");
            }
        }
    }

    nvs_close(handle);


  /*  if(timeStampDatos==0){
   //     inicializarDatos();
    }else{
        ROMtoRAMDatos();
    }
        
    if(timeStampConfig==0){
       //printf("Inicializando configuración por primera vez...\n");
        //inicializarConfig();
    }else{
        ROMtoRAMConfig();
    }
        // Imprimir configuración para verificar que se cargó correctamente
       *//*printf("Configuración cargada:\n");
        printf("Caravana Libre 1: %s\n", configuracion.caravanaLibre1);
        printf("Caravana Libre 2: %s\n", configuracion.caravanaLibre2);
        printf("Caravana Libre 3: %s\n", configuracion.caravanaLibre3);
        printf("Caravana Libre 4: %s\n", configuracion.caravanaLibre4);
        printf("Caravana Libre 5: %s\n", configuracion.caravanaLibre5);
        printf("Calibracion Motor: %d\n", configuracion.calibracionMotor);
        printf("Calibracion Agua: %d\n", configuracion.calibracionAgua);
        printf("Peso Animal Desconocido: %d\n", configuracion.pesoAnimalDesconocido);
    if(timeStampCurva==0){
        //inicializarCurva();
    }else{
        ROMtoRAMCurva();
    } strncpy(corral[0].nombre, "999000000000015", sizeof(corral[0].nombre) - 1);
    corral[0].nombre[sizeof(corral[0].nombre) - 1] = '\0';
    strncpy(corral[1].nombre, "982000459578918", sizeof(corral[1].nombre) - 1);
    corral[1].nombre[sizeof(corral[1].nombre) - 1] = '\0';
    corral[0].tipoCurva=1;
    corral[1].tipoCurva=1;
    corral[0].pesoDosis=20;
    corral[1].pesoDosis=30;
    corral[0].fechaServicio=1740970800;
    corral[1].fechaServicio=1741143600;
    corral[0].indiceCorporal=1;
    corral[1].indiceCorporal=1;
    corral[0].agua=true;
    corral[1].agua=false;
    corral[0].cantDosis=3;
    corral[1].cantDosis=2;
    corral[0].intervaloMin=60;
    corral[1].intervaloMin=120;
    corral[20].cantDosis=2;
    corral[20].pesoDosis=25;
    corral[20].intervaloMin=1;
    curva[1].segmentos[0].inicio=1;
    curva[1].segmentos[0].pesoInicio=10;
    curva[1].segmentos[1].inicio=114;
    curva[1].segmentos[1].pesoInicio=20;
    calibracionMotor=5;
    calibracionAgua=10;*/
    init_i2c();
    //printf("aca si");
    init_uart();
    //printf("aca no");
    
    //ds1307_write_register(0x00,0x00);
    //ds1307_write_register(0x01,0x10);
    //ds1307_write_register(0x02,0x07);
    //ds1307_write_register(0x03,0x01);
    //ds1307_write_register(0x04,0x10);
    //ds1307_write_register(0x05,0x03);
    //ds1307_write_register(0x06,0x25);

    ds1307_write_register(0x07,0x93);
    //traerDatosROMtoRAM();
    printf("arrancando\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(active_message_task, "active_message_task", 2048*4, NULL, 9, NULL);
}