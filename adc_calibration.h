// adc_calibration.h
// Written by: Nathan - Static Calibration Engineer
// Plugs into stage_calibration.c at the HOOK point

#ifndef ADC_CALIBRATION_H
#define ADC_CALIBRATION_H

#include "stm32g0xx_hal.h"

// Stores the calibration results after the 6-step routine
typedef struct {
    float offset_x;   // ADC count at true 0g on X
    float offset_y;   // ADC count at true 0g on Y
    float offset_z;   // ADC count at true 0g on Z
    float scale_x;    // ADC counts per 1g on X
    float scale_y;    // ADC counts per 1g on Y
    float scale_z;    // ADC counts per 1g on Z
    uint8_t is_valid;  // 1 = calibration passed, 0 = failed
    uint8_t fail_x;    // 1 = X axis failed validation
    uint8_t fail_y;    // 1 = Y axis failed validation
    uint8_t fail_z;    // 1 = Z axis failed validation
} CalibData;

// Pass your ADC handle in before Stage_Calibration_Run() is called
void Calibration_SetADC(ADC_HandleTypeDef *hadc);

// Called by stage_calibration.c at the HOOK point after each button press
// i = 0(+X), 1(-X), 2(+Y), 3(-Y), 4(+Z), 5(-Z)
void Calibration_SampleAxis(uint8_t i);

// Called after all 6 steps to compute offset and scale
void Calibration_Compute(void);

// Convert a raw ADC reading to g using calibration results
float Calibration_ToG(uint32_t raw, float offset, float scale);

// Access the results after Calibration_Compute() is called
CalibData* Calibration_GetData(void);

#endif
