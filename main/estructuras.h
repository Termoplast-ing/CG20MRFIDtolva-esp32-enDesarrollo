#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

typedef struct {
    bool errorHoraDS1307;
    bool errorUART;
    bool tolvaVacia;
    bool jaulaTrabada;
    bool errorMotor;
    bool errorAgua;
    bool caravanaNoReconocida;
} error_status_t;

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
} tipo_curva;

typedef struct {
    uint8_t caravanaLibre1[17];
    uint8_t caravanaLibre2[17];
    uint8_t caravanaLibre3[17];
    uint8_t caravanaLibre4[17];
    uint8_t caravanaLibre5[17];
    uint8_t calibracionMotor;
    uint8_t calibracionAgua;
    uint8_t pesoAnimalDesconocido;
} configuration;