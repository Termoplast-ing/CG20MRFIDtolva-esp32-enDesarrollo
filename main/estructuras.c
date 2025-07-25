#include "estructuras.h"

data_animal corral[25]= {0}; // Inicializar el corral con 25 animales
data_animal auxiliar = {0}; // Estructura auxiliar para operaciones temporales
configuration configuracion = {0};
tipo_curva curva[5] = {0}; // Inicializar las curvas de crecimiento
data_animal_leido animal_leido[100] = {0}; // Inicializar el registro de animales leídos
data_animal_leido animal_leido_AUX = {0}; // Inicializar el registro de animales leídos auxiliar

void cargar_datos_prueba(data_animal_leido animales1[], int cantidad) {
    uint8_t MAX_ANIMALES=100;
    for (int i = 0; i < cantidad && i < MAX_ANIMALES; i++) {
        snprintf(animales1[i].nombre, sizeof(animales1[i].nombre), "Animal_%02d", i + 1);
        animales1[i].fechaDispensado = 1753440000+(100000*i); // hace i horas
        animales1[i].pesoDispensado = 100 + i; // peso ficticio
    }

    // Resto con nombre vacío (simula lugares libres)
    for (int i = cantidad; i < MAX_ANIMALES; i++) {
        strcpy(animales1[i].nombre, "000000000000000");
        animales1[i].fechaDispensado = 0;
        animales1[i].pesoDispensado = 0;
    }
}