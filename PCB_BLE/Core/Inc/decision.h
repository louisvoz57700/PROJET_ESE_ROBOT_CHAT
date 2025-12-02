/*
 * decision.h
 *
 *  Created on: Nov 25, 2025
 *      Author: ombeline
 */

#ifndef INC_DECISION_H_
#define INC_DECISION_H_


#include <stdint.h>
#include <stdbool.h>


typedef enum {
    STATE_IDLE = 0,
    STATE_CHASE,
    STATE_FLEE
} State;

typedef struct {
    float enemyDist;    // distance to target 
    float enemyAngle;   // angle of target (deg): 0 = front, +right, -left
    float enemyX;       
    float enemyY;

    bool edgeFront;
    bool edgeLeft;
    bool edgeRight;
    bool edgeBack;
} SensorData;

typedef struct {
    int16_t forwardSpeed; //  -100..100 (%)
    int16_t steering;     //  -100..100 (left/right)
} DriveCommand;

void Decision_Init(void);
DriveCommand Decision_Update(const SensorData *s);

void Decision_SetState(State newState);
State Decision_GetState(void);

#endif /* INC_DECISION_H_ */
