
#ifndef ESTIMATION_ESTIMATOR_TASK_H_
#define ESTIMATION_ESTIMATOR_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t xEstimatorTaskHandle;
extern TaskHandle_t xControllerManagerTaskHandle;
void EstimatorTask_Start(void);

#endif /* ESTIMATION_ESTIMATOR_TASK_H_ */
