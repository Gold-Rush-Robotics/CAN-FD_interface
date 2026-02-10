#ifndef BOARD_BUILDER_H
#define BOARD_BUILDER_H

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>

/* Controller headers */
#include "brushed_motor_controller.h"
#include "brushless_motor_controller.h"
#include "servo_controller.h"
#include "grr_can.h"

/* Generated config – produced by scripts/generate_board_config.py */
#include "board_config_generated.h"

/* ================================================================
 *   BoardBuilder
 * ================================================================
 * Reads the compile-time #defines from board_config_generated.h
 * and constructs + initialises every controller declared in the
 * YAML config.
 *
 * Usage:
 *   BoardBuilder builder;
 *   if (!builder.build()) { LOG_ERR("build failed"); }
 *
 *   // Access controllers:
 *   builder.brushed_motors[0].setVelocity(500);
 *   builder.servos[1].setAngleDegrees(45.0f);
 * ================================================================ */

struct BoardBuilder {

    /* ---- Public arrays sized by generated config ---- */
#if NUM_BRUSHED_MOTORS > 0
    BrushedMotorController   brushed_motors[NUM_BRUSHED_MOTORS];
#endif
#if NUM_BRUSHLESS_MOTORS > 0
    BrushlessMotorController brushless_motors[NUM_BRUSHLESS_MOTORS];
#endif
#if NUM_SERVOS > 0
    PWMServoController       servos[NUM_SERVOS];
#endif

    /* ---- Build everything from generated defines ---- */
    bool build();

    /* ---- Convenience: emergency-stop all actuators ---- */
    void emergencyStopAll();

    /* ---- Convenience: update all controllers ---- */
    void updateAll(float dt);

    /* ---- Convenience: send CAN status for all ---- */
    void sendAllStatus();
};

#endif /* BOARD_BUILDER_H */
