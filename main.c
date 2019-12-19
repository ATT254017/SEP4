/*
* FreeRTOS_ATMEGA.c
*
* Created: 15/10/2018 13:08:53
* Author : Afonso
*/

//needed for LoRa
#include <ihal.h>
#include <lora_driver.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>

#include <ATMEGA_FreeRTOS.h>

#include <semphr.h>

#include <iled.h>

#include <hih8120.h>

#include <display_7seg.h>

#include "../FreeRTOSTraceDriver/FreeRTOSTraceDriver.h"

#include <stdio.h>
#include <stdio_driver.h>
#include <stddef.h>

#include <avr/interrupt.h>

#include <mh_z19.h>
#include <util/delay.h>

#include <queue.h>

#define LED_TASK_PRIORITY 20
#define LORA_appEUI "d4fa9edc5efe7723"
#define LORA_appKEY "77a62d36181cddb23bbd6e15e5ddb19f"

uint16_t mainPpm;

static char _out_buf[100];

static lora_payload_t _uplink_payload;

mh_z19_return_code_t rc;

//define queues
QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t co2_queue;

//define semaphore handles
SemaphoreHandle_t xTempSemaphore;
SemaphoreHandle_t xHumSemaphore;
SemaphoreHandle_t xCo2Semaphore;
SemaphoreHandle_t xSharedSensorSemaphore;

//define task handlers
void task_read_temperature( void *pvParameters );
void task_read_humidity( void *pvParameters );
void task_read_co2( void *pvParameters );
void lora_handler_task( void *pvParameters);

static void _lora_setup(void)
{
	e_LoRa_return_code_t rc;
	led_slow_blink(led_ST2); // OPTIONAL: Led the green led blink slowly while we are setting up LoRa

	// Factory reset the transceiver
	printf("FactoryReset >%s<\n", lora_driver_map_return_code_to_text(lora_driver_rn2483_factory_reset()));

	// Configure to EU868 LoRaWAN standards
	printf("Configure to EU868 >%s<\n", lora_driver_map_return_code_to_text(lora_driver_configure_to_eu868()));

	// Get the transceivers HW EUI
	rc = lora_driver_get_rn2483_hweui(_out_buf);
	printf("Get HWEUI >%s<: %s\n",lora_driver_map_return_code_to_text(rc), _out_buf);

	// Set the HWEUI as DevEUI in the LoRaWAN software stack in the transceiver
	printf("Set DevEUI: %s >%s<\n", _out_buf, lora_driver_map_return_code_to_text(lora_driver_set_device_identifier(_out_buf)));

	// Set Over The Air Activation parameters to be ready to join the LoRaWAN
	printf("Set OTAA Identity appEUI:%s appKEY:%s devEUI:%s >%s<\n", LORA_appEUI, LORA_appKEY, _out_buf, lora_driver_map_return_code_to_text(lora_driver_set_otaa_identity(LORA_appEUI,LORA_appKEY,_out_buf)));

	// Save all the MAC settings in the transceiver
	printf("Save mac >%s<\n",lora_driver_map_return_code_to_text(lora_driver_save_mac()));

	// Enable Adaptive Data Rate
	printf("Set Adaptive Data Rate: ON >%s<\n", lora_driver_map_return_code_to_text(lora_driver_set_adaptive_data_rate(LoRa_ON)));

	// Join the LoRaWAN
	uint8_t maxJoinTriesLeft = 5;
	do {
		rc = lora_driver_join(LoRa_OTAA);
		printf("Join Network TriesLeft:%d >%s<\n", maxJoinTriesLeft, lora_driver_map_return_code_to_text(rc));

		if ( rc != LoRa_ACCEPTED)
		{
			// Make the red led pulse to tell something went wrong
			led_long_puls(led_ST1); // OPTIONAL
			// Wait 5 sec and lets try again
			vTaskDelay(pdMS_TO_TICKS(5000UL));
		}
		else
		{
			break;
		}
	} while (--maxJoinTriesLeft);

	if (rc == LoRa_ACCEPTED)
	{
		// Connected to LoRaWAN :-)
		// Make the green led steady
		led_led_on(led_ST2); // OPTIONAL
	}
	else
	{
		// Something went wrong
		// Turn off the green led
		led_led_off(led_ST2); // OPTIONAL
		// Make the red led blink fast to tell something went wrong
		led_fast_blink(led_ST1); // OPTIONAL

		// Lets stay here
		while (1)
		{
			taskYIELD();
		}
	}
}



void my_co2_call_back(uint16_t ppm)
{
	// Here you can use the CO2 ppm value
	mainPpm= ppm;
}

void innitialise()
{
	hal_create(LED_TASK_PRIORITY); // Must be called first!! LED_TASK_PRIORITY must be a high priority in your system
	lora_driver_create(ser_USART1); // The parameter is the USART port the RN2483 module is connected to - in this case USART1


	mainPpm = 0;
	// The first parameter is the USART port the MH-Z19 sensor is connected to - in this case USART3
	// The second parameter is the address of the call back function
	mh_z19_create(ser_USART3, my_co2_call_back);

	if ( HIH8120_OK == hih8120Create() )
	{
		// Driver created OK
		printf("hih8210Create() returns: %d  We need 0\n", hih8120Create());
	}
}

/*-----------------------------------------------------------*/
void create_tasks_and_semaphores(void)
{
	// Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
	// because it is sharing a resource, such as the Serial port.
	// Semaphores should only be used whilst the scheduler is running, but we can set it up here.
	if(xTempSemaphore == NULL)
	{
		xTempSemaphore = xSemaphoreCreateMutex();
	}
	if(xHumSemaphore == NULL)
	{
		xHumSemaphore = xSemaphoreCreateMutex();
	}
	if(xCo2Semaphore == NULL)
	{
		xCo2Semaphore = xSemaphoreCreateMutex();
	}
	if(xSharedSensorSemaphore == NULL){
		xSharedSensorSemaphore = xSemaphoreCreateMutex();
	}

	//Create queues
	temperature_queue = xQueueCreate(1, sizeof(uint16_t));
	humidity_queue = xQueueCreate(1, sizeof(uint16_t));
	co2_queue = xQueueCreate(1, sizeof(uint16_t));

	xTaskCreate(
	task_read_temperature
	,  (const portCHAR *)"TempTask"  // A name just for humans
	,  configMINIMAL_STACK_SIZE  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );

	xTaskCreate(
	task_read_humidity
	,  (const portCHAR *)"HumTask"  // A name just for humans
	,  configMINIMAL_STACK_SIZE  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );

	xTaskCreate(
	task_read_co2
	,  (const portCHAR *)"Co2Task"  // A name just for humans
	,  configMINIMAL_STACK_SIZE  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );

	xTaskCreate(
	lora_handler_task
	,  (const portCHAR *)"LoRaTask"  // A name just for humans
	,  configMINIMAL_STACK_SIZE  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2
	,  NULL );

}

/*-----------------------------------------------------------*/

void task_read_temperature( void *pvParameters )
{
	#if (configUSE_APPLICATION_TASK_TAG == 1)
	// Set task no to be used for tracing with R2R-Network for digital to analog conversion
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	#endif

	uint16_t temperature = 0;

	for(;;)
	{
		if(xSemaphoreTake(xSharedSensorSemaphore,portMAX_DELAY)){
		
			if (HIH8120_OK != hih8120Wakeup()){
				printf("wake up failed\n");
			}

			vTaskDelay(100);

			if (HIH8120_OK != hih8120Meassure()){
				printf("data polling failed\n");
			}

			vTaskDelay(100);
			
			temperature=hih8120GetTemperature_x10();
			xSemaphoreGive(xSharedSensorSemaphore);
		}
		

		if(xSemaphoreTake(xTempSemaphore,portMAX_DELAY))
		{

			printf("Temperature to queue is: %d\n", temperature);

			//Send temperature to queue
			xQueueSend(temperature_queue, //queue handle
			(void*) &temperature, //pointer to the temperature measurement
			0);
		}

	}
}

/*-----------------------------------------------------------*/

void task_read_humidity( void *pvParameters )
{
	#if (configUSE_APPLICATION_TASK_TAG == 1)
	// Set task no to be used for tracing with R2R-Network for digital to analog conversion
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	#endif

	uint16_t humidity = 0;

	for(;;)
	{
		if(xSemaphoreTake(xSharedSensorSemaphore,portMAX_DELAY)){
			
		
			if (HIH8120_OK != hih8120Wakeup()){
				printf("wake up failed\n");
			}

			vTaskDelay(100);

			if (HIH8120_OK != hih8120Meassure()){
				printf("data polling failed\n");
			}

			vTaskDelay(100);
			
			humidity=hih8120GetHumidityPercent_x10();
			
			xSemaphoreGive(xSharedSensorSemaphore);
	}
	

		if(xSemaphoreTake(xHumSemaphore,portMAX_DELAY))
		{

			printf("Humidity to queue is: %d\n", humidity);

			//Send humidity to queue
			xQueueSend(humidity_queue, //queue handle
			(void*) &humidity, //pointer to the humidity measurement
			0);
		}

	}
}

/*-----------------------------------------------------------*/
void getCo2(void)
{
	uint16_t ppm=0;
	mh_z19_return_code_t rc;
	rc = mh_z19_take_meassuring();
}


void task_read_co2( void *pvParameters )
{
	#if (configUSE_APPLICATION_TASK_TAG == 1)
	// Set task no to be used for tracing with R2R-Network for digital to analog conversion
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	#endif

	for(;;)
	{

		if(xSemaphoreTake(xCo2Semaphore,portMAX_DELAY))
		{

			

			getCo2();
			vTaskDelay(60);
			
			printf("Co2 to queue is: %d\n", mainPpm);

			xQueueSend(co2_queue,
			(void*) &mainPpm,
			0);
		}

	}
}

/*-----------------------------------------------------------*/

void lora_handler_task( void *pvParameters )
{
	static e_LoRa_return_code_t rc;

	// Hardware reset of LoRaWAN transceiver
	lora_driver_reset_rn2483(1);
	vTaskDelay(2);
	lora_driver_reset_rn2483(0);
	// Give it a chance to wakeup
	vTaskDelay(150);

	lora_driver_flush_buffers(); // get rid of first version string from module after reset!

	_lora_setup();

	uint16_t hum = 0; // humidity
	int16_t temp = 0; // temperature
	uint16_t co2_ppm = 0; // CO2

	for(;;)
	{

		xSemaphoreGive(xTempSemaphore);
		xSemaphoreGive(xHumSemaphore);
		xSemaphoreGive(xCo2Semaphore);

		//delay between each sent payload
		vTaskDelay(360000);

		xQueueReceive(temperature_queue, // queue handle
		&temp, // address of temperature placeholder
		portMAX_DELAY);  // time out if the queue is empty


		xQueueReceive(humidity_queue, // queue handle
		&hum, // address of humidity placeholder
		portMAX_DELAY);  // time out if the queue is empty


		xQueueReceive(co2_queue,
		&co2_ppm,
		portMAX_DELAY);


		/* PRINTING RESULTS IN THE QUEUE AFTER RECEIVING */
		printf("Temperature from queue: %d\n", temp);
		printf("Humidity from queue: %d\n", hum);
		printf("CO2 from queue: %d\n", co2_ppm);


		_uplink_payload.len = 6;
		_uplink_payload.port_no = 2;
		_uplink_payload.bytes[0] = hum >> 8;
		_uplink_payload.bytes[1] = hum & 0xFF;
		_uplink_payload.bytes[2] = temp >> 8;
		_uplink_payload.bytes[3] = temp & 0xFF;
		_uplink_payload.bytes[4] = co2_ppm >> 8;
		_uplink_payload.bytes[5] = co2_ppm & 0xFF;

		led_short_puls(led_ST4);  // OPTIONAL

		printf("Upload Message >%s<\n", lora_driver_map_return_code_to_text( lora_driver_sent_upload_message(false, &_uplink_payload)));
	}
}


/*-----------------------------------------------------------*/
int main(void)
{

	stdioCreate(0);

	sei();

	innitialise();

	trace_init();

	create_tasks_and_semaphores();

	vTaskStartScheduler(); // initialise and run the freeRTOS scheduler. Execution should never return here.

	while (1)
	{
	}
}