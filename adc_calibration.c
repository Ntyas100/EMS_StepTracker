// adc_calibration.c
// Written by: Nathan - Static Calibration Engineer
// Contains all ADC sampling and calibration math logic

#include "adc_calibration.h"

#define NUM_SAMPLES 64  // readings to average per measurement

// -------------------------------------------------------
// Validation thresholds — no PCB gain, direct ADXL335 to ADC
// ADXL335 sensitivity at 3V: 270-330 mV/g (datasheet)
// Ratiometric adjustment for 3.3V supply: × (3.3/3.0)
//   270 × 1.1 = 297 mV/g (minimum)
//   330 × 1.1 = 363 mV/g (maximum)
// ADC: 12-bit at 3.3V = 4095/3.3 = 1241 counts/V
// Expected scale:
//   297 mV/g × 1.241 counts/mV = 368 counts/g (min)
//   363 mV/g × 1.241 counts/mV = 450 counts/g (max)
// With 15% tolerance: 313 to 518 counts/g
// 0g bias: VS/2 = 1.65V = ~2048 counts, ±600 tolerance
// -------------------------------------------------------
#define SCALE_MIN   313.0f   // counts per g minimum (with tolerance)
#define SCALE_MAX   518.0f   // counts per g maximum (with tolerance)

// OFFSET (0g ADC count):
// The ADXL335 outputs VS/2 at 0g (datasheet: ratiometric)
// At 3.3V supply: 3.3V / 2 = 1.65V at 0g
// In ADC counts: 1.65V x 1241 counts/V = 2048 counts
// Allow +-600 counts of tolerance to account for:
//   - ADXL335 0g bias error (datasheet: typ +-150mV = +-186 counts)
//   - Temperature drift (datasheet: +-1 mg/degC)
//   - ADC offset error and component variation
// Lower bound: 2048 - 600 = 1448 counts
// Upper bound: 2048 + 600 = 2648 counts
// -------------------------------------------------------
#define OFFSET_MIN  1448.0f  // minimum sensible 0g ADC count
#define OFFSET_MAX  2648.0f  // maximum sensible 0g ADC count

// Returns 1 if valid, 0 if out of range
static uint8_t ValidateAxis(float offset, float scale, const char *axis_name)
{
    if (scale < SCALE_MIN || scale > SCALE_MAX) return 0;
    if (offset < OFFSET_MIN || offset > OFFSET_MAX) return 0;
    return 1;
}

// Raw averaged ADC readings — [step 0-5][axis: 0=X, 1=Y, 2=Z]  columns are the sensor axis
static float raw[6][3];

// Final computed calibration results
static CalibData calib = {0};

// ADC handle — set before calibration starts
static ADC_HandleTypeDef *adc = NULL;

// -------------------------------------------------------
// Set the ADC handle — call this in main.c before
// Stage_Calibration_Run() is called
// -------------------------------------------------------
void Calibration_SetADC(ADC_HandleTypeDef *hadc)
{
    adc = hadc;
}

// -------------------------------------------------------
// Read a single conversion on a given ADC channel
// -------------------------------------------------------
static uint32_t ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(adc, &cfg);

    HAL_ADC_Start(adc);
    HAL_ADC_PollForConversion(adc, 10);
    uint32_t val = HAL_ADC_GetValue(adc);
    HAL_ADC_Stop(adc);
    return val;
}

// -------------------------------------------------------
// Average NUM_SAMPLES readings on one channel
// -------------------------------------------------------
static float AverageChannel(uint32_t channel)
{
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += ReadChannel(channel);
        HAL_Delay(2);
    }
    return (float)sum / (float)NUM_SAMPLES;
}

// -------------------------------------------------------
// Sample all three axes for one calibration step
// Called by stage_calibration.c at the HOOK point
// i = 0(+X), 1(-X), 2(+Y), 3(-Y), 4(+Z), 5(-Z)
// -------------------------------------------------------
void Calibration_SampleAxis(uint8_t i)
{
    if (adc == NULL) return;  // safety check

    raw[i][0] = AverageChannel(ADC_CHANNEL_1);  // X on PA1
    raw[i][1] = AverageChannel(ADC_CHANNEL_2);  // Y on PA2
    raw[i][2] = AverageChannel(ADC_CHANNEL_3);  // Z on PA3
}

// -------------------------------------------------------
// Calculate offset and scale from the 6 raw measurements
// Call this after Stage_Calibration_Run() returns
//
// offset = midpoint between +1g and -1g readings = true 0g
// scale  = half the gap = ADC counts per 1g
// -------------------------------------------------------
void Calibration_Compute(void)
{
    calib.offset_x = (raw[0][0] + raw[1][0]) / 2.0f; // i.e. it takes +Xup + Xdown / 2
    calib.scale_x  = (raw[0][0] - raw[1][0]) / 2.0f;

    calib.offset_y = (raw[2][1] + raw[3][1]) / 2.0f;
    calib.scale_y  = (raw[2][1] - raw[3][1]) / 2.0f;

    calib.offset_z = (raw[4][2] + raw[5][2]) / 2.0f;
    calib.scale_z  = (raw[4][2] - raw[5][2]) / 2.0f;

    // Validate each axis against datasheet-derived thresholds
    uint8_t x_ok = ValidateAxis(calib.offset_x, calib.scale_x, "X");
    uint8_t y_ok = ValidateAxis(calib.offset_y, calib.scale_y, "Y");
    uint8_t z_ok = ValidateAxis(calib.offset_z, calib.scale_z, "Z");

    if (x_ok && y_ok && z_ok)
    {
        calib.is_valid = 1;  // All axes passed
    }
    else
    {
        calib.is_valid = 0;  // One or more axes failed

        // Store which axes failed for display purposes
        calib.fail_x = !x_ok;
        calib.fail_y = !y_ok;
        calib.fail_z = !z_ok;
    }

    // calib.is_valid = 1; Need to add threshold pass|fail return here
}

// -------------------------------------------------------
// Convert raw ADC reading to g
// -------------------------------------------------------
float Calibration_ToG(uint32_t raw_adc, float offset, float scale)
{
    return ((float)raw_adc - offset) / scale;
}

// -------------------------------------------------------
// Get pointer to calibration results
// -------------------------------------------------------
CalibData* Calibration_GetData(void)
{
    return &calib;
}
