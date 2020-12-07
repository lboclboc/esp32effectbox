#include <string>
#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/event_groups.h>

#define MINOR_COGS 9
#define MAJOR_COGS 37
#define STEPS_PER_TURN 4096
#define SIDEREAL_DAY_SECONDS ((((23 * 60.0) + 56) * 60.0) + 4.091)

void app_main(void);

#ifdef __cplusplus
}
#endif

/* FreeRTOS event group to signal when we are connected*/
//static EventGroupHandle_t wifi_event_group;

/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void app_main(void)
{
	static const int i2s_num = 0; // i2s port number

	static const i2s_config_t i2s_config = {
	    .mode = I2S_MODE_MASTER | I2S_MODE_TX,
	    .sample_rate = 44100,
	    .bits_per_sample = 16,
	    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
	    .intr_alloc_flags = 0, // default interrupt priority
	    .dma_buf_count = 8,
	    .dma_buf_len = 64,
	    .use_apll = false
	};

	i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
}
