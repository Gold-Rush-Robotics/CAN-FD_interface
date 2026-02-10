#include "../include/board_builder.h"
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(board_builder, LOG_LEVEL_INF);

/* ================================================================
 *   Macro helpers – iterate the generated config defines
 *   Each INIT_BRUSHED_MOTOR_N / INIT_SERVO_N block is guarded by
 *   NUM_BRUSHED_MOTORS / NUM_SERVOS so unused code is compiled out.
 * ================================================================ */

bool BoardBuilder::build()
{
    LOG_INF("BoardBuilder: name=\"%s\"  node_id=0x%02X", BOARD_NAME, BOARD_NODE_ID);
    LOG_INF("  brushed=%d  brushless=%d  servos=%d",
            NUM_BRUSHED_MOTORS, NUM_BRUSHLESS_MOTORS, NUM_SERVOS);

    /* ---- Apply configured node ID ---- */
    g_node_id = BOARD_NODE_ID;

    /* ==============================================================
     *   Brushed motors
     * ============================================================== */
#if NUM_BRUSHED_MOTORS > 0
    {
        /* --- Motor 0 --- */
#define _BM0_INIT
        brushed_motors[0] = BrushedMotorController(BRUSHED_MOTOR_0_ID,
                                                    BRUSHED_MOTOR_0_JOINT_NAME);
        brushed_motors[0].setPIDGains(BRUSHED_MOTOR_0_KP,
                                      BRUSHED_MOTOR_0_KI,
                                      BRUSHED_MOTOR_0_KD);
        brushed_motors[0].setLimits(BRUSHED_MOTOR_0_MAX_VEL,
                                    BRUSHED_MOTOR_0_MAX_ACCEL,
                                    BRUSHED_MOTOR_0_MIN_POS,
                                    BRUSHED_MOTOR_0_MAX_POS);
        brushed_motors[0].setMaxDuty(BRUSHED_MOTOR_0_MAX_DUTY);

        /* Hardware init – uncomment when devicetree nodes are ready:
         * const struct device *pwm0 = DEVICE_DT_GET(BRUSHED_MOTOR_0_PWM_LABEL);
         * const struct device *gpio0 = DEVICE_DT_GET(BRUSHED_MOTOR_0_GPIO_LABEL);
         * brushed_motors[0].init(pwm0, BRUSHED_MOTOR_0_PWM_CHANNEL,
         *                       BRUSHED_MOTOR_0_PWM_PERIOD_NS,
         *                       gpio0, BRUSHED_MOTOR_0_DIR_PIN,
         *                       BRUSHED_MOTOR_0_SLP_PIN, BRUSHED_MOTOR_0_FLT_PIN);
         */

#if BRUSHED_MOTOR_0_HAS_ENCODER
        /* Encoder – uncomment when devicetree node is ready:
         * const struct device *enc_gpio0 = DEVICE_DT_GET(BRUSHED_MOTOR_0_ENC_GPIO_LABEL);
         * brushed_motors[0].attachEncoder(enc_gpio0,
         *     BRUSHED_MOTOR_0_ENC_A_PIN, BRUSHED_MOTOR_0_ENC_B_PIN,
         *     BRUSHED_MOTOR_0_ENC_TPR);
         */
#endif
#if BRUSHED_MOTOR_0_HAS_CS
        /* Current sense – uncomment when devicetree node is ready:
         * const struct device *adc0 = DEVICE_DT_GET(BRUSHED_MOTOR_0_CS_ADC_LABEL);
         * brushed_motors[0].attachCurrentSense(adc0,
         *     BRUSHED_MOTOR_0_CS_ADC_CH, BRUSHED_MOTOR_0_CS_OC_MA);
         */
#endif
        LOG_INF("  brushed[0] = id:%d \"%s\"",
                BRUSHED_MOTOR_0_ID, BRUSHED_MOTOR_0_JOINT_NAME);
    }
#endif /* NUM_BRUSHED_MOTORS > 0 */

#if NUM_BRUSHED_MOTORS > 1
    {
        brushed_motors[1] = BrushedMotorController(BRUSHED_MOTOR_1_ID,
                                                    BRUSHED_MOTOR_1_JOINT_NAME);
        brushed_motors[1].setPIDGains(BRUSHED_MOTOR_1_KP,
                                      BRUSHED_MOTOR_1_KI,
                                      BRUSHED_MOTOR_1_KD);
        brushed_motors[1].setLimits(BRUSHED_MOTOR_1_MAX_VEL,
                                    BRUSHED_MOTOR_1_MAX_ACCEL,
                                    BRUSHED_MOTOR_1_MIN_POS,
                                    BRUSHED_MOTOR_1_MAX_POS);
        brushed_motors[1].setMaxDuty(BRUSHED_MOTOR_1_MAX_DUTY);

#if BRUSHED_MOTOR_1_HAS_ENCODER
        /* Encoder attach – uncomment with real devicetree */
#endif
#if BRUSHED_MOTOR_1_HAS_CS
        /* Current-sense attach – uncomment with real devicetree */
#endif
        LOG_INF("  brushed[1] = id:%d \"%s\"",
                BRUSHED_MOTOR_1_ID, BRUSHED_MOTOR_1_JOINT_NAME);
    }
#endif /* NUM_BRUSHED_MOTORS > 1 */

    /* ==============================================================
     *   Brushless motors
     * ============================================================== */
#if NUM_BRUSHLESS_MOTORS > 0
    {
        brushless_motors[0] = BrushlessMotorController(BRUSHLESS_MOTOR_0_ID,
                                                        BRUSHLESS_MOTOR_0_JOINT_NAME);
        brushless_motors[0].setPIDGains(BRUSHLESS_MOTOR_0_KP,
                                        BRUSHLESS_MOTOR_0_KI,
                                        BRUSHLESS_MOTOR_0_KD);
        brushless_motors[0].setLimits(BRUSHLESS_MOTOR_0_MAX_VEL,
                                      BRUSHLESS_MOTOR_0_MAX_ACCEL,
                                      BRUSHLESS_MOTOR_0_MIN_POS,
                                      BRUSHLESS_MOTOR_0_MAX_POS);

        LOG_INF("  brushless[0] = id:%d \"%s\"",
                BRUSHLESS_MOTOR_0_ID, BRUSHLESS_MOTOR_0_JOINT_NAME);
    }
#endif

    /* ==============================================================
     *   Servos
     * ============================================================== */
#if NUM_SERVOS > 0
    {
        servos[0] = PWMServoController(SERVO_0_ID, SERVO_0_JOINT_NAME);
        servos[0].setAngleLimits(SERVO_0_ANGLE_MIN, SERVO_0_ANGLE_MAX);
        servos[0].setAngleOffset(SERVO_0_ANGLE_OFFSET);
        servos[0].setMoveSpeed(SERVO_0_SPEED_DPS);
        servos[0].setPWMTiming(SERVO_0_PWM_MIN_US, SERVO_0_PWM_MAX_US);

        /* Hardware init – uncomment when devicetree nodes are ready:
         * const struct device *spwm0 = DEVICE_DT_GET(SERVO_0_PWM_LABEL);
         * servos[0].init(spwm0, SERVO_0_PWM_CHANNEL,
         *                SERVO_0_PWM_MIN_US, SERVO_0_PWM_MAX_US,
         *                SERVO_0_PWM_PERIOD_US);
         */
        LOG_INF("  servo[0] = id:%d \"%s\"", SERVO_0_ID, SERVO_0_JOINT_NAME);
    }
#endif

#if NUM_SERVOS > 1
    {
        servos[1] = PWMServoController(SERVO_1_ID, SERVO_1_JOINT_NAME);
        servos[1].setAngleLimits(SERVO_1_ANGLE_MIN, SERVO_1_ANGLE_MAX);
        servos[1].setAngleOffset(SERVO_1_ANGLE_OFFSET);
        servos[1].setMoveSpeed(SERVO_1_SPEED_DPS);
        servos[1].setPWMTiming(SERVO_1_PWM_MIN_US, SERVO_1_PWM_MAX_US);

        LOG_INF("  servo[1] = id:%d \"%s\"", SERVO_1_ID, SERVO_1_JOINT_NAME);
    }
#endif

    LOG_INF("BoardBuilder: build complete");
    return true;
}

/* ================================================================ */

void BoardBuilder::emergencyStopAll()
{
#if NUM_BRUSHED_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHED_MOTORS; i++) {
        brushed_motors[i].emergencyStop();
    }
#endif
#if NUM_BRUSHLESS_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHLESS_MOTORS; i++) {
        brushless_motors[i].emergencyStop();
    }
#endif
#if NUM_SERVOS > 0
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].disable();
    }
#endif
}

void BoardBuilder::updateAll(float dt)
{
#if NUM_BRUSHED_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHED_MOTORS; i++) {
        brushed_motors[i].update(dt);
    }
#endif
#if NUM_BRUSHLESS_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHLESS_MOTORS; i++) {
        brushless_motors[i].update(dt);
    }
#endif
#if NUM_SERVOS > 0
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].update(dt);
    }
#endif
}

void BoardBuilder::sendAllStatus()
{
    const struct device *can_dev = get_can_device();
    if (!can_dev) return;

#if NUM_BRUSHED_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHED_MOTORS; i++) {
        brushed_motors[i].sendCANStatus(can_dev);
    }
#endif
#if NUM_BRUSHLESS_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHLESS_MOTORS; i++) {
        brushless_motors[i].sendCANStatus(can_dev);
    }
#endif
#if NUM_SERVOS > 0
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].sendCANStatus(can_dev);
    }
#endif
}
