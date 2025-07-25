#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#define MAX_animales 100

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
    char nombre[16];
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
    uint8_t caravanaLibre1[16];
    uint8_t caravanaLibre2[16];
    uint8_t caravanaLibre3[16];
    uint8_t caravanaLibre4[16];
    uint8_t caravanaLibre5[16];
    uint8_t calibracionMotor;
    uint8_t calibracionAgua;
    uint8_t pesoAnimalDesconocido;
} configuration;

void cargar_datos_prueba(data_animal_leido animales1[], int cantidad);

extern data_animal corral[25];
extern data_animal auxiliar;
extern configuration configuracion;
extern tipo_curva curva[5];
extern data_animal_leido animal_leido[MAX_animales];
extern data_animal_leido animal_leido_AUX; // Inicializar el registro de animales leídos