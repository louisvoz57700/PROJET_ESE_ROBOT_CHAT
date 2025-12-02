/*
 * decision.c
 *
 *  Created on: Nov 25, 2025
 *      Author: thomas
 */

#include "decision.h"
#include <math.h>

#define AGGRESSIVITY_LEVEL 1.0f

static State currentState = STATE_IDLE;

void Decision_Init(void)
{
    currentState = STATE_IDLE;
}

void Decision_SetState(State newState)
{
    currentState = newState;
}

State Decision_GetState(void)
{
    return currentState;
}

DriveCommand Decision_Update(const SensorData *s)
{
    DriveCommand cmd = {0, 0};

    // ------------------------------
    // 1. AVOID FALLING OFF THE TABLE
    // ------------------------------
    if (s->edgeFront)
    {
        cmd.forwardSpeed = -80; // hard reverse
        cmd.steering = 0;
        return cmd;
    }
    if (s->edgeLeft)
    {
        cmd.forwardSpeed = 80;
        cmd.steering = 100; // turn hard right
        return cmd;
    }
    if (s->edgeRight)
    {
        cmd.forwardSpeed = 80;
        cmd.steering = -100; // turn hard left
        return cmd;
    }

    if (s->edgeBack)
    {
        cmd.forwardSpeed = 80;
        cmd.steering = 0; // go forward
        return cmd;
    }

    // ============================================================
    //                            FSM
    // ============================================================
    switch (currentState)
    {

    // ------------------------------------------------------------
    // 1. IDLE STATE
    // ------------------------------------------------------------
    case STATE_IDLE:
        cmd.forwardSpeed = 0;
        cmd.steering = 0;
        return cmd;

    // ------------------------------------------------------------
    // 2. FLEE(mouse) STATE (for now unchanged)
    // ------------------------------------------------------------
    case STATE_FLEE:
        cmd.forwardSpeed = 0;
        cmd.steering = 0;
        return cmd;

    // ------------------------------------------------------------
    // 3. CHASE(cat) STATE
    // ------------------------------------------------------------
    case STATE_CHASE:
    {
        // defensive: ensure sensor pointer valid
        if (!s)
        {
            cmd.forwardSpeed = 0;
            cmd.steering = 0;
            return cmd;
        }

        // No target -> search (tbd: implement search behavior or just stop?)
        if (isnan(s->enemyDist) || s->enemyDist <= 0.0f)
        {
            cmd.forwardSpeed = 0;
            cmd.steering = 0;
            return cmd;
        }

        // Target found -> attack
        float angle = s->enemyAngle; // degrees
        float steer_f = angle * AGGRESSIVITY_LEVEL;

        // clamp to [-100, 100]
        if (steer_f > 100.0f)
            steer_f = 100.0f;
        if (steer_f < -100.0f)
            steer_f = -100.0f;

        // explicit rounding/cast to match DriveCommand field type
        cmd.steering = (int)roundf(steer_f);
        cmd.forwardSpeed = 100;
    }
        return cmd;
    }
    return cmd;
}
