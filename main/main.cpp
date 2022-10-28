#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"

#include "Adafruit_MAX31865.h"



extern "C" void app_main();


#define PIN_NUM_CLK  	18
#define PIN_NUM_MISO 	19
#define PIN_NUM_MOSI 	23
//#define PIN_NUM_CS   	( (gpio_num_t) 32 )

#define SPI_SS_MAX31865_1  ((gpio_num_t) 32 )
#define SPI_SS_MAX31865_2  ((gpio_num_t)  33) 
#define SPI_SS_MAX31865_3  ((gpio_num_t)  25) 
#define SPI_SS_MAX31865_4  ((gpio_num_t) 26)
#define SPI_SS_MAX31865_5  ((gpio_num_t) 14) 


spi_device_handle_t 	_spi;

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

void AtualizaSPI(int pinCS);

void initializeSPI( int mosi, int miso, int clk, int cs )
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
	memset( &buscfg, 0, sizeof(spi_bus_config_t) );

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
	buscfg.sclk_io_num = clk;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 0;
	buscfg.flags = 0;
	buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
	buscfg.intr_flags = 0;


    // Started working after reduce clock speed to 8MHz but then when I changed
    // back to 10 Mhz it continued working. Not sure whats going on

    spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof(spi_device_interface_config_t) );

   	devcfg.address_bits = 8;
    devcfg.mode= 1;
	devcfg.clock_speed_hz = 100000;

	//devcfg.spics_io_num=cs;
	devcfg.spics_io_num=-1;

	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	devcfg.queue_size = 1;


    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    if ( ret > 0 )
    	return;

    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);


}

void AtualizaSPI( gpio_num_t pinCS){

    gpio_num_t PIN_NUM_CS = pinCS;

	initializeSPI( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS );

    gpio_pad_select_gpio(PIN_NUM_CS);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

	gpio_set_level(PIN_NUM_CS, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(PIN_NUM_CS, 1);

}

void app_main(void)
{
    while(1)
    {
		AtualizaSPI(SPI_SS_MAX31865_1);
		
	    Adafruit_MAX31865 tempSensor ( _spi );
	    tempSensor.begin(MAX31865_3WIRE); 

    	uint16_t rtd = tempSensor.readRTD();
    	//uint16_t rtd1 = tempSensor.readRTD();

    	float tempMAX31865 = tempSensor.temperature(RNOMINAL, RREF);
        
    	printf( "(MAX31865) %7.4f\n", tempMAX31865);
    	vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

}

