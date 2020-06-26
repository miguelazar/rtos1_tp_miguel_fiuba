

/*==================[inclusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// sAPI header
#include "sapi.h"
#include "FreeRTOSConfig.h"

/* Demo includes. */
#include "supporting_functions.h"

/*==================[definiciones y macros]==================================*/
#define TIEMPO_BLINK_LED_AZUL 200
/*==================[definiciones de datos internos]=========================*/
//typedef struct sensor_t sensor_t;
typedef struct {
	uint16_t medicion_tem;
	uint16_t medicion_hum;
}sensor_t;
/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

// C++ version 0.4 char* style "itoa":
// Written by Lukás Chmela
// Released under GPLv3.

char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_lcd( void* pvParameters);
void tarea_tem(void * pvParameters);
//void tarea_hum(void * pvParameters);
static void tarea_led(void* pvParameters);
//uint16_t leer_temperatura();
//void leer_temperatura(void *pvParameter);
void leer_temperatura(sensor_t *param);
//uint16_t leer_humedad();
//float leer_temperatura();


QueueHandle_t queue_datos;

//Mutex para imprimir

SemaphoreHandle_t Mutex_print;

//QueueHandle_t queue_hum;

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{

    // ---------- CONFIGURACIONES ------------------------------
    // Inicializar y configurar la plataforma
    boardConfig();

    // UART for debug messages
    debugPrintConfigUart( UART_USB, 115200 );

    debugPrintlnString( "Trabajo Final RTOS / Protocolos de Comunicación en SE\n" );
    
    // Inicializacion de AnalogIO
    adcConfig(ADC_ENABLE); // ADC

    // Crear cola en freeRTOS
    //queue_datos = xQueueCreate(3, sizeof(float));
    //queue_datos = xQueueCreate(5, sizeof(uint16_t));
    queue_datos = xQueueCreate(5, sizeof(sensor_t));
    if (queue_datos == NULL) {
        debugPrintlnString("La cola no puede ser creada");
    }
/*    queue_hum = xQueueCreate(5, sizeof(uint16_t));
    if (queue_hum == NULL) {
        debugPrintlnString("La cola no puede ser creada");
    }*/

    Mutex_print = xSemaphoreCreateMutex();

    // Crear tareas en freeRTOS
    
    // Creacion de la tarea que imprime en el LCD
    xTaskCreate(
    	tarea_lcd,                     	// Funcion de la tarea a ejecutar
		//( const char * )
		"TAREA LCD",   					// Nombre de la tarea como String amigable para el usuario
		configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
		0,                          	// Parametros de tarea, se puede usar NULL
		tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
		0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );

    // Creacion de la tarea que realiza lectura de temperatura
    xTaskCreate(
        tarea_tem,                    	 // Funcion de la tarea a ejecutar
        //( const char * )
		"TAREA TEMPERATURA",   			// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );
    
    // Creacion de la tarea que realiza lectura de humedad
/*    xTaskCreate(
        tarea_hum,                    	 // Funcion de la tarea a ejecutar
        //( const char * )
		"TAREA HUMEDAD",   			// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );*/

    // Creacion de la tarea que mantiene un led indicador de funcionamiento
    xTaskCreate(
        tarea_led,                     // Funcion de la tarea a ejecutar
        ( const char * )"TAREA LED",   	// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );
    
    // Iniciar scheduler
    vTaskStartScheduler(); // Enciende tick | Crea idle y pone en ready | Evalua las tareas creadas | Prioridad mas alta pasa a running

    // ---------- REPETIR POR SIEMPRE --------------------------
    while( TRUE )
    {
        // Si cae en este while 1 significa que no pudo iniciar el scheduler
    }

    // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
    // directamenteno sobre un microcontroladore y no es llamado por ningun
    // Sistema Operativo, como en el caso de un programa para PC.
    return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/




//-----------------------------------------------------------
//---------------------- T E M P ----------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
void tarea_tem(void * pvParameters) {
    
    // ---------- CONFIGURACIONES ------------------------------
    //uint16_t temperatura_actual;
	sensor_t temperatura_actual;
    //float temperatura_actual;

    // ---------- REPETIR POR SIEMPRE --------------------------
    while(1) {
        //temperatura_actual = leer_temperatura();
    	leer_temperatura(&temperatura_actual);
        xQueueSend(queue_datos, &temperatura_actual, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//-----------------------------------------------------------
//---------------------- H U M ------------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
/*void tarea_hum(void * pvParameters) {

    // ---------- CONFIGURACIONES ------------------------------
    uint16_t humedad_actual;
    //float temperatura_actual;

    // ---------- REPETIR POR SIEMPRE --------------------------
    while(1) {
        humedad_actual = leer_humedad();
        xQueueSend(queue_hum, &humedad_actual, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*/

//-----------------------------------------------------------
//------------------------ L C D ----------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
void tarea_lcd( void* pvParameters)
{
    // ---------- CONFIGURACIONES ------------------------------

	TickType_t xLastWakeTime;
	const TickType_t xDelay750ms = pdMS_TO_TICKS( 750UL );
	xLastWakeTime = xTaskGetTickCount();
	TickType_t xPeriodicity =  900/portTICK_RATE_MS;
    //uint16_t temperatura;
    //uint16_t humedad;
	sensor_t temperatura;
    //float temperatura;
    //char buffer[20];
    static char buffer[20];

    i2cInit(I2C0, 100000); 	// Inicialización del protocolo
    
    // Inicializar LCD de 16x2 (caracteres x lineas) con cada caracter de 5x2 pixeles
    lcdInit(16, 2, 5, 8);
    
    // Apaga el cursor
    lcdCursorSet(LCD_CURSOR_OFF);
    
    // ---------- REPETIR POR SIEMPRE --------------------------
    while( TRUE )
    {
        if (xQueueReceive(queue_datos, &temperatura, xPeriodicity) == pdPASS) {
        	if (pdTRUE == xSemaphoreTake( Mutex_print, portMAX_DELAY)){
        		lcdClear();
        		lcdGoToXY(0, 0); // Poner cursor en 0, 0
        		lcdSendStringRaw("Tem: ");
        		lcdGoToXY(0, 7);
        		itoa( temperatura.medicion_tem, buffer, 10 );
        		lcdSendStringRaw(buffer);
        		lcdGoToXY(0, 1);
        		lcdSendStringRaw("Hum: ");
        		lcdGoToXY(1, 7);
        		itoa( temperatura.medicion_hum, buffer, 10 );
        		lcdSendStringRaw(buffer);

        		vTaskDelay(pdMS_TO_TICKS(1000));
            	xSemaphoreGive( Mutex_print );
        	}
        	vTaskDelayUntil( &xLastWakeTime, xDelay750ms );
        }

        /*if (xQueueReceive(queue_hum, &humedad, portMAX_DELAY) == pdPASS) {
        	//lcdClear();
            lcdGoToXY(0, 1); // Poner cursor en 0, 0
            lcdSendStringRaw("Hum: ");
        	//sprintf(buffer,"%3.2f",temperatura);
            lcdGoToXY(1, 7);
            itoa( humedad, buffer, 10 );
            lcdSendStringRaw(buffer);
            //lcdSendStringRaw("hola");
            vPrintStringAndNumber("Temperatura = ", temperatura);
        }
            else{
            	vPrintStringAndNumber("No hay datos para mostrar \r\n ", temperatura);

        }*/
    }
}



//-----------------------------------------------------------
//------------------------ L E D ----------------------------
//-----------------------------------------------------------
// Tarea de led testigo o indicador de funcionamiento
static void tarea_led( void * pvParameters )
{
	// Blink cada 200ms.
	TickType_t xLastWakeTime;
	TickType_t xPeriodicity =  200/portTICK_RATE_MS;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
    	// Prendo el led azul
    	gpioWrite( LEDB, ON );
    	vTaskDelayUntil(&xLastWakeTime,xPeriodicity );
    	// Apago el led azul
    	gpioWrite( LEDB, OFF );
    	vTaskDelayUntil(&xLastWakeTime, xPeriodicity);
    }
}

//-----------------------------------------------------------
//---------------- S E N S O R    T E M P -------------------
//-----------------------------------------------------------
// Funcion para la lectura del sensor de temperatura
void leer_temperatura(sensor_t *param) {
	//sensor_t sensor;
	//sensor.medicion_tem = 0;
	//sensor.medicion_hum = 0;

	//char valortemp[1]={0};
	//	char valorhum[1]={0};
    // Variable para almacenar el valor leido del ADC CH1
    //uint16_t data;

    // Leo la Entrada Analogica AI0 - ADC0 CH1
	param->medicion_tem = adcRead(CH1);
	param->medicion_hum = adcRead(CH2);
}

//-----------------------------------------------------------
//---------------- S E N S O R    H U M ---------------------
//-----------------------------------------------------------
// Funcion para la lectura del sensor de temperatura
/*uint16_t leer_humedad() {

    // Variable para almacenar el valor leido del ADC CH2
    uint16_t data;

    // Leo la Entrada Analogica AI0 - ADC0 CH2
    data = adcRead(CH2);

    return data;
}*/
/*==================[fin del archivo]========================================*/
