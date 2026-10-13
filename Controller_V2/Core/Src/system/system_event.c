/*
 * system_event.c
 *
 *  Created on: 25-Sept-2026
 *      Author: GeetikSharma
 */


/*
 * system_events.c
 *
 *  Created on: 25-Sept-2026
 *      Author: GeetikSharma
 */

#include "system/system_event.h"


/* =========================
 * Event Group Handle
 * ========================= */

static EventGroupHandle_t system_event_group;


/* =========================
 * Initialization
 * ========================= */

void SystemEvents_Init(void)
{
    system_event_group = xEventGroupCreate();
}


/* =========================
 * Set Event
 * ========================= */

void SystemEvents_Set(
    EventBits_t event
)
{
    if (system_event_group == NULL)
    {
        return;
    }

    xEventGroupSetBits(
        system_event_group,
        event
    );
}


/* =========================
 * Wait For Event
 * ========================= */

EventBits_t SystemEvents_Wait(
    EventBits_t event
)
{
    if (system_event_group == NULL)
    {
        return 0U;
    }

    return xEventGroupWaitBits(
        system_event_group,
        event,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
}
