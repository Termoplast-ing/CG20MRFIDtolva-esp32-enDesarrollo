#include "estructuras.h"

data_animal corral[26]= {0}; // Inicializar el corral con 25 animales
data_animal auxiliar = {0}; // Estructura auxiliar para operaciones temporales
configuration configuracion = {0};
tipo_curva curva[5] = {0}; // Inicializar las curvas de crecimiento
data_animal_leido animal_leido[100] = {0}; // Inicializar el registro de animales leídos
data_animal_leido animal_leido_AUX = {0}; // Inicializar el registro de animales leídos auxiliar



/*void cargar_datos_prueba(data_animal_leido animales1[], int cantidad) {
    uint8_t MAX_ANIMALES=100;
    for (int i = 0; i < cantidad && i < MAX_ANIMALES; i++) {
        snprintf(animales1[i].nombre, sizeof(animales1[i].nombre), "Animal_%02d", i + 1);
        animales1[i].fechaDispensado = 1753440000+(100000*i); // hace i horas
        animales1[i].pesoDispensado = 100 + i; // peso ficticio
    }

    // Resto con nombre vacío (simula lugares libres)
	
    for (uint8_t i = 0; i < 100; i++) {
        strcpy(animales1[i].nombre, "000000000000000");
        animales1[i].fechaDispensado = 0;
        animales1[i].pesoDispensado = 0;
    }
}*/



    void inicializar_curvas(){
	curva[0].segmentos[0].inicio = 1;
	curva[0].segmentos[0].pesoInicio = 50;
	curva[0].segmentos[1].inicio = 114;
	curva[0].segmentos[1].pesoInicio = 100;
	curva[1].segmentos[0].inicio = 1;
	curva[1].segmentos[0].pesoInicio = 100;
	curva[1].segmentos[1].inicio = 114;
	curva[1].segmentos[1].pesoInicio = 100;
	curva[2].segmentos[0].inicio = 1;
	curva[2].segmentos[0].pesoInicio = 100;
	curva[2].segmentos[1].inicio = 114;
	curva[2].segmentos[1].pesoInicio = 50;
    curva[3].segmentos[0].inicio = 1;
	curva[3].segmentos[0].pesoInicio = 100;
	curva[3].segmentos[1].inicio = 55;
	curva[3].segmentos[1].pesoInicio = 50;
    curva[3].segmentos[2].inicio = 114;
	curva[3].segmentos[2].pesoInicio = 100;

/*corral[0].nombre[0] = '9';
corral[0].nombre[1] = '9';
corral[0].nombre[2] = '9';
corral[0].nombre[3] = '0';
corral[0].nombre[4] = '0';
corral[0].nombre[5] = '0';
corral[0].nombre[6] = '2';
corral[0].nombre[7] = '0';
corral[0].nombre[8] = '2';
corral[0].nombre[9] = '4';
corral[0].nombre[10] = '0';
corral[0].nombre[11] = '0';
corral[0].nombre[12] = '0';
corral[0].nombre[13] = '1';
corral[0].nombre[14] = '9';
corral[0].nombre[15] = '\0';

corral[0].tipoCurva = 1;
corral[0].pesoDosis = 200;
corral[0].fechaServicio = 1741591022; // 20 de noviembre
corral[0].indiceCorporal = 1;
corral[0].agua = true;
corral[0].cantDosis = 2;
corral[0].intervaloMin = 60;	

configuracion.calibracionAgua= 10;
configuracion.calibracionMotor= 5;
configuracion.pesoAnimalDesconocido= 50;*/
}