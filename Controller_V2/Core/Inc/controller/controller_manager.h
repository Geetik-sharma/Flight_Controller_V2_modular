/*
 * controller_manager.h
 *
 *  Created on: 29-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_CONTROLLER_CONTROLLER_MANAGER_H_
#define INC_CONTROLLER_CONTROLLER_MANAGER_H_

#include "estimation/estimator_state.h"
#include "controller/rate_controller.h"

/* =========================
 * Controller Manager
 * ========================= */

void ControllerManager_Init(void);
void ControllerManager_Start(void);
void ControllerManager_Update(float dt);


#endif /* INC_CONTROLLER_CONTROLLER_MANAGER_H_ */
