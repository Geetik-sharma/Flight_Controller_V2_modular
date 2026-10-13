/*
 * system_event.h
 *
 *  Created on: 25-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_SYSTEM_SYSTEM_EVENT_H_
#define INC_SYSTEM_SYSTEM_EVENT_H_

#include "FreeRTOS.h"
#include "event_groups.h"


/* =========================
 * System Events
 * ========================= */

#define SYSTEM_EVENT_CALIBRATION_DONE    (1U << 0)


/* =========================
 * Initialization
 * ========================= */

void SystemEvents_Init(void);


/* =========================
 * Set Event
 * ========================= */

void SystemEvents_Set(
    EventBits_t event
);


/* =========================
 * Wait For Event
 * ========================= */

EventBits_t SystemEvents_Wait(
    EventBits_t event
);


#endif /* INC_SYSTEM_SYSTEM_EVENT_H_ */
