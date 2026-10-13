/*
 * ppm_decoder.h
 *
 *  Created on: 05-Oct-2026
 *      Author: GeetikSharma
 */

#ifndef INC_RC_PPM_DECODER_H_
#define INC_RC_PPM_DECODER_H_


#include <stdint.h>

#define PPM_MAX_CHANNELS    6U


typedef struct
{
    uint16_t channel[PPM_MAX_CHANNELS];
    uint8_t channel_count;
    uint8_t frame_valid;
} PPMFrame_t;

void PPMDecoder_Init(void);

void PPMDecoder_Reset(void);

void PPMDecoder_GetFrame(PPMFrame_t *frame);

void PPMDecoder_ProcessCapture(uint32_t capture);
#endif /* INC_RC_PPM_DECODER_H_ */
