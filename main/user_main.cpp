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
    printf("SDK version:%s\n", esp_get_idf_version());

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_0);
    while(true)
    {
        int val = adc1_get_raw(ADC1_GPIO35_CHANNEL);
        std::cout << "ADC Value: " << val << std::endl;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
