#ifndef SENSOR_H
#define SENSOR_H

#include "esp_adc/adc_continuous.h"

void adc_init(void);
void adc_task(void *pvParameters);
extern adc_continuous_handle_t adc_handle;

#endif // SENSOR_H
