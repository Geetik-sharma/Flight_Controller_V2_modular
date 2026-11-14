#include "rc/ppm_decoder.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

#define PPM_CHANNEL_MIN_US      900U
#define PPM_CHANNEL_MAX_US     2100U
#define PPM_FRAME_GAP_MIN_US   3000U

static PPMFrame_t ppm_working_frame;
static PPMFrame_t ppm_frame;

static uint32_t ppm_previous_capture;
static uint32_t ppm_last_valid_frame_capture;

static uint8_t ppm_capture_initialized;
static uint8_t ppm_channel_index;
static uint8_t ppm_frame_timestamp_valid;

/* Debug variables */
volatile uint32_t ppm_last_delta;
volatile uint32_t ppm_capture_count;
volatile uint32_t ppm_min_delta;
volatile uint32_t ppm_max_delta;
uint32_t frame_age;
void PPMDecoder_Init(void)
{
    PPMDecoder_Reset();
}

void PPMDecoder_Reset(void)
{
    uint8_t i;

    taskENTER_CRITICAL();

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
    ppm_last_valid_frame_capture = 0U;

    ppm_capture_initialized = 0U;
    ppm_channel_index = 0U;
    ppm_frame_timestamp_valid = 0U;

    taskEXIT_CRITICAL();
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

    ppm_last_delta = delta;
    ppm_capture_count++;

    if (delta > PPM_FRAME_GAP_MIN_US)
    {
        if (ppm_channel_index == PPM_MAX_CHANNELS)
        {
            /*
             * Publish the completed frame.
             * This function is called from the TIM2 capture ISR.
             */
            ppm_frame.channel_count = PPM_MAX_CHANNELS;

            for (uint8_t i = 0U; i < PPM_MAX_CHANNELS; i++)
            {
                ppm_frame.channel[i] =
                    ppm_working_frame.channel[i];
            }

            ppm_last_valid_frame_capture = capture;
            ppm_frame_timestamp_valid = 1U;
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
        else
        {
            /* Ignore extra channel pulses until the next sync gap. */
        }
    }
}

void PPMDecoder_GetFrame(PPMFrame_t *frame)
{
    uint32_t current_capture;


    if (frame == NULL)
    {
        return;
    }

    /*
     * Read the current TIM2 counter and published data.
     * TIM2 is configured as a 32-bit free-running counter.
     */
    taskENTER_CRITICAL();

    current_capture = __HAL_TIM_GET_COUNTER(&htim2);

    frame->channel_count = ppm_frame.channel_count;
    frame->frame_valid = ppm_frame.frame_valid;

    for (uint8_t i = 0U; i < PPM_MAX_CHANNELS; i++)
    {
        frame->channel[i] = ppm_frame.channel[i];
    }

    if (ppm_frame_timestamp_valid != 0U)
    {
        frame_age = current_capture - ppm_last_valid_frame_capture;

        if (frame_age > PPM_FRAME_TIMEOUT_US)
        {
            frame->frame_valid = 0U;
        }
    }
    else
    {
        frame->frame_valid = 0U;
    }

    taskEXIT_CRITICAL();
}
