#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "../include/grr_can.h"
#include "../include/board_builder.h"

LOG_MODULE_REGISTER(grr_main);

/* ================================================================
 *   BOARD BUILDER  (constructed from YAML-generated config)
 * ================================================================ */

static BoardBuilder board;

/* ================================================================
 *   TIMERS
 * ================================================================ */

static struct k_timer hb_timer;
static struct k_timer control_timer;
static struct k_timer status_timer;

/* ================================================================
 *   LED CONFIGURATION
 * ================================================================ */

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ================================================================
 *   CAN MESSAGE QUEUE
 * ================================================================ */

CAN_MSGQ_DEFINE(my_can_msgq, 16);

/* ================================================================
 *   CAN COMMAND HANDLERS
 * ================================================================
 *   These dispatch incoming motor / servo frames to the correct
 *   controller instances owned by the BoardBuilder.
 * ================================================================ */

static void motor_command_handler(const struct can_frame *frame)
{
    if (!frame) return;

    uint8_t target_motor = frame->id & 0xFF;

    /* E-STOP → stop every actuator */
    if (frame->id == E_STOP) {
        LOG_WRN("Motor E-STOP triggered");
        board.emergencyStopAll();
        return;
    }

    /* Dispatch to brushed motors */
#if NUM_BRUSHED_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHED_MOTORS; i++) {
        if (board.brushed_motors[i].getMotorId() == target_motor ||
            (frame->dlc > 0 && frame->data[0] == 0xFF)) {
            board.brushed_motors[i].processCANCommand(frame);
        }
    }
#endif

    /* Dispatch to brushless motors */
#if NUM_BRUSHLESS_MOTORS > 0
    for (int i = 0; i < NUM_BRUSHLESS_MOTORS; i++) {
        if (board.brushless_motors[i].getMotorId() == target_motor ||
            (frame->dlc > 0 && frame->data[0] == 0xFF)) {
            board.brushless_motors[i].processCANCommand(frame);
        }
    }
#endif
}

static void servo_command_handler(const struct can_frame *frame)
{
    if (!frame) return;

    uint8_t target_servo = frame->id & 0xFF;

    if (frame->id == E_STOP) {
        LOG_WRN("Servo E-STOP triggered");
#if NUM_SERVOS > 0
        for (int i = 0; i < NUM_SERVOS; i++) {
            board.servos[i].disable();
        }
#endif
        return;
    }

#if NUM_SERVOS > 0
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (board.servos[i].getServoId() == target_servo ||
            (frame->dlc > 0 && frame->data[0] == 0xFF)) {
            board.servos[i].processCANCommand(frame);
        }
    }
#endif
}

/* ================================================================
 *   TIMER CALLBACKS
 * ================================================================ */

static void hb_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);

    bool sent = send_heartbeat();
    if (!sent) {
        LOG_ERR("Failed to send heartbeat");
    }
}

static void control_loop_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    board.updateAll(CONTROL_DT);
}

static void status_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    board.sendAllStatus();
}

/* ================================================================
 *   LED INIT
 * ================================================================ */

static bool init_led(void)
{
    if (!device_is_ready(led.port)) {
        LOG_ERR("LED device not ready");
        return false;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("LED pin configuration failed");
        return false;
    }

    LOG_INF("LED initialized");
    return true;
}

/* ================================================================
 *   MAIN
 * ================================================================ */

int main(void)
{
    /* Allow USB CDC to enumerate */
    k_sleep(K_MSEC(1000));

    printk("\n\n\r");
    LOG_INF("========================================");
    LOG_INF("  GRR CAN-FD  –  %s", BOARD_NAME);
    LOG_INF("========================================");

    /* ---- LED ---- */
    if (!init_led()) {
        LOG_ERR("LED initialization failed");
        return 1;
    }

    /* ---- CAN bus ---- */
    if (!init_can()) {
        LOG_ERR("CAN initialization failed");
        gpio_pin_set_dt(&led, 1);
        return 1;
    }
    LOG_INF("CAN initialized successfully");

    if (!setup_can_filters(&my_can_msgq)) {
        LOG_ERR("CAN filter setup failed");
        return 1;
    }
    LOG_INF("CAN filters configured");

    /* ---- Board builder (YAML-generated config) ---- */
    if (!board.build()) {
        LOG_ERR("Board builder failed");
        return 1;
    }

    /* ---- Unique-ID handshake ---- */
    can_announce_uid();

    /* ---- Register CAN command dispatchers ---- */
    register_motor_command_handler(motor_command_handler);
    register_servo_command_handler(servo_command_handler);
    LOG_INF("CAN command handlers registered");

    /* ---- Timers ---- */
    k_timer_init(&hb_timer, hb_timeout, NULL);
    k_timer_start(&hb_timer, K_MSEC(HEARTBEAT_INTERVAL_MS),
                  K_MSEC(HEARTBEAT_INTERVAL_MS));

    k_timer_init(&control_timer, control_loop_timeout, NULL);
    k_timer_start(&control_timer, K_MSEC(1000 / CONTROL_LOOP_HZ),
                  K_MSEC(1000 / CONTROL_LOOP_HZ));

    k_timer_init(&status_timer, status_timeout, NULL);
    k_timer_start(&status_timer, K_MSEC(STATUS_INTERVAL_MS),
                  K_MSEC(STATUS_INTERVAL_MS));

    LOG_INF("All systems go  –  control=%d Hz  status=%d ms  heartbeat=%d ms",
            CONTROL_LOOP_HZ, STATUS_INTERVAL_MS, HEARTBEAT_INTERVAL_MS);

    /* ---- Main loop: drain CAN RX queue ---- */
    struct can_frame rx_frame;
    int ret;

    while (1) {
        ret = k_msgq_get(&my_can_msgq, &rx_frame, K_MSEC(100));

        if (ret == 0) {
            bool handled = handle_incoming_frame(&rx_frame);
            if (!handled) {
                LOG_DBG("Unhandled CAN frame: id=0x%03X dlc=%d",
                        rx_frame.id, rx_frame.dlc);
            }
        }

        /* Blink LED to show liveness (~1 Hz) */
        static uint32_t led_counter = 0;
        if (++led_counter >= 10) {
            gpio_pin_toggle_dt(&led);
            led_counter = 0;
        }
    }

    return 0;
}