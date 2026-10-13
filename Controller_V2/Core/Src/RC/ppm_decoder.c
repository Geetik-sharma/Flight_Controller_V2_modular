#include "rc/ppm_decoder.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"



#define PPM_CHANNEL_MIN_US     900U
#define PPM_CHANNEL_MAX_US    2100U
#define PPM_FRAME_GAP_MIN_US  3000U

static PPMFrame_t ppm_working_frame;

static PPMFrame_t ppm_frame;

static uint32_t ppm_previous_capture;
static uint8_t ppm_capture_initialized;
static uint8_t ppm_channel_index;
volatile uint32_t ppm_last_delta;
volatile uint32_t ppm_capture_count;
volatile uint32_t ppm_min_delta;
volatile uint32_t ppm_max_delta;

void PPMDecoder_Init(void)
{
    PPMDecoder_Reset();
}

void PPMDecoder_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < PPM_MAX_CHANNELS; i++)
    {
        ppm_working_frame.channel[i] = 0U;
        ppm_frame.channel[i] = 0U;
    }

    ppm_working_frame.channel_count = 0U;
    ppm_working_frame.frame_valid = 0U;

    ppm_frame.channel_count = 0U;
    ppm_frame.frame_valid = 0U;

    ppm_previous_capture = 0U;
    ppm_capture_initialized = 0U;
    ppm_channel_index = 0U;
}

void PPMDecoder_ProcessCapture(uint32_t capture)
{
    uint32_t delta;

    if (ppm_capture_initialized == 0U)
    {
        ppm_previous_capture = capture;
        ppm_capture_initialized = 1U;
        return;
    }


    delta = capture - ppm_previous_capture;
    ppm_previous_capture = capture;


    if (delta > PPM_FRAME_GAP_MIN_US)
    {

        if (ppm_channel_index == PPM_MAX_CHANNELS)
        {
            ppm_frame = ppm_working_frame;

            ppm_frame.channel_count = PPM_MAX_CHANNELS;
            ppm_frame.frame_valid = 1U;
        }


        ppm_channel_index = 0U;

        return;
    }


    if ((delta >= PPM_CHANNEL_MIN_US) &&
        (delta <= PPM_CHANNEL_MAX_US))
    {
        if (ppm_channel_index < PPM_MAX_CHANNELS)
        {
            ppm_working_frame.channel[ppm_channel_index] =
                (uint16_t)delta;

            ppm_channel_index++;
        }
    }
}

void PPMDecoder_GetFrame(PPMFrame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    *frame = ppm_frame;

    taskEXIT_CRITICAL();
}
