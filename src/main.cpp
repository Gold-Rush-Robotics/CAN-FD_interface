#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "../include/grr_can.h"
#include "../include/motor_controller.h"
#include "../include/servo_controller.h"

LOG_MODULE_REGISTER(grr_main);

/* ================================================================
 *   CONFIGURATION
 * ================================================================ */

#define NUM_MOTORS      2
#define NUM_SERVOS      4
#define CONTROL_LOOP_HZ 100
#define CONTROL_DT      (1.0f / CONTROL_LOOP_HZ)

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
 *   MOTOR AND SERVO CONTROLLERS
 * ================================================================ */

static BrushlessMotorController motors[NUM_MOTORS];
static PWMServoController servos[NUM_SERVOS];

/* ================================================================
 *   CAN COMMAND HANDLERS
 * ================================================================ */

static void motor_command_handler(const struct can_frame *frame)
{
    if (!frame) return;
    
    /* Extract motor ID from lower byte of CAN ID (0x30xx) */
    uint8_t target_motor = frame->id & 0xFF;
    
    /* E-STOP handling */
    if (frame->id == E_STOP) {
        LOG_WRN("Motor E-STOP triggered");
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].emergencyStop();
        }
        return;
    }
    
    /* Process command for specific motor or broadcast (0xFF in first byte) */
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].getMotorId() == target_motor || 
            (frame->dlc > 0 && frame->data[0] == 0xFF)) {
            motors[i].processCANCommand(frame);
        }
    }
}

static void servo_command_handler(const struct can_frame *frame)
{
    if (!frame) return;
    
    /* Extract servo ID from lower byte of CAN ID (0x40xx) */
    uint8_t target_servo = frame->id & 0xFF;
    
    /* E-STOP handling */
    if (frame->id == E_STOP) {
        LOG_WRN("Servo E-STOP triggered");
        for (int i = 0; i < NUM_SERVOS; i++) {
            servos[i].disable();
        }
        return;
    }
    
    /* Process command for specific servo or broadcast */
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servos[i].getServoId() == target_servo ||
            (frame->dlc > 0 && frame->data[0] == 0xFF)) {
            servos[i].processCANCommand(frame);
        }
    }
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
    
    /* Update all motors */
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].update(CONTROL_DT);
    }
    
    /* Update all servos */
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].update(CONTROL_DT);
    }
}

static void status_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    
    const struct device *can_dev = get_can_device();
    if (!can_dev) return;
    
    /* Send status for all motors */
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].sendCANStatus(can_dev);
    }
    
    /* Send status for all servos */
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].sendCANStatus(can_dev);
    }
}

/* ================================================================
 *   INITIALIZATION FUNCTIONS
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

static bool init_motors(void)
{
    LOG_INF("Initializing %d brushless motors...", NUM_MOTORS);
    
    /* Initialize motor controller objects */
    motors[0] = BrushlessMotorController(1, "motor_left");
    motors[1] = BrushlessMotorController(2, "motor_right");
    
    /* 
     * Note: In a real application, you would get PWM and GPIO devices from devicetree
     * Example:
     *   const struct device *pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
     *   const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
     *   motors[0].init(pwm_dev, 0, 50000, gpio_dev, 10, 11, 12, 13);
     * 
     * For now, we'll just log that motors are created but not hardware-initialized
     */
    
    /* Set default PID gains */
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPIDGains(1.0f, 0.01f, 0.1f);
        motors[i].setLimits(1000.0f, 500.0f, -1000000, 1000000);
        LOG_INF("Motor %d (%s) created", motors[i].getMotorId(), motors[i].getJointName());
    }
    
    return true;
}

static bool init_servos(void)
{
    LOG_INF("Initializing %d servo motors...", NUM_SERVOS);
    
    /* Initialize servo controller objects */
    servos[0] = PWMServoController(1, "servo_pan");
    servos[1] = PWMServoController(2, "servo_tilt");
    servos[2] = PWMServoController(3, "servo_gripper");
    servos[3] = PWMServoController(4, "servo_aux");
    
    /* 
     * Note: In a real application, you would get PWM device from devicetree
     * Example:
     *   const struct device *pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm2));
     *   servos[0].init(pwm_dev, 0);
     *   servos[0].setAngleLimits(-900, 900);  // -90 to +90 degrees
     */
    
    /* Configure servo limits */
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].setAngleLimits(-900, 900);  /* -90 to +90 degrees */
        servos[i].setMoveSpeed(180);           /* 180 degrees per second */
        LOG_INF("Servo %d (%s) created", servos[i].getServoId(), servos[i].getJointName());
    }
    
    return true;
}

/* ================================================================
 *   MAIN FUNCTION
 * ================================================================ */

int main(void)
{
    /* Add delay for USB CDC initialization */
    k_sleep(K_MSEC(1000));
    
    printk("\n\n\r");
    LOG_INF("========================================");
    LOG_INF("Starting GRR CAN-FD Motor/Servo Control");
    LOG_INF("========================================");
    
    /* Initialize LED */
    if (!init_led()) {
        LOG_ERR("LED initialization failed");
        return 1;
    }
    
    /* Initialize CAN bus */
    if (!init_can()) {
        LOG_ERR("CAN initialization failed");
        gpio_pin_set_dt(&led, 1);  /* Turn on LED to indicate error */
        return 1;
    }
    LOG_INF("CAN initialized successfully");
    
    /* Setup CAN filters */
    if (!setup_can_filters(&my_can_msgq)) {
        LOG_ERR("CAN filter setup failed");
        return 1;
    }
    LOG_INF("CAN filters configured");
    
    /* Initialize motors */
    if (!init_motors()) {
        LOG_ERR("Motor initialization failed");
        return 1;
    }
    
    /* Initialize servos */
    if (!init_servos()) {
        LOG_ERR("Servo initialization failed");
        return 1;
    }
    
    /* Register CAN command handlers */
    register_motor_command_handler(motor_command_handler);
    register_servo_command_handler(servo_command_handler);
    LOG_INF("CAN command handlers registered");
    
    /* Initialize and start heartbeat timer (1 Hz) */
    k_timer_init(&hb_timer, hb_timeout, NULL);
    k_timer_start(&hb_timer, K_SECONDS(1), K_SECONDS(1));
    
    /* Initialize and start control loop timer (100 Hz) */
    k_timer_init(&control_timer, control_loop_timeout, NULL);
    k_timer_start(&control_timer, K_MSEC(1000 / CONTROL_LOOP_HZ), K_MSEC(1000 / CONTROL_LOOP_HZ));
    
    /* Initialize and start status reporting timer (10 Hz) */
    k_timer_init(&status_timer, status_timeout, NULL);
    k_timer_start(&status_timer, K_MSEC(100), K_MSEC(100));
    
    LOG_INF("All systems initialized - entering main loop");
    LOG_INF("Control loop: %d Hz, Status report: 10 Hz, Heartbeat: 1 Hz", CONTROL_LOOP_HZ);
    
    /* Main loop - process incoming CAN messages */
    struct can_frame rx_frame;
    int ret;
    
    while (1) {
        /* Wait for incoming CAN frame with timeout */
        ret = k_msgq_get(&my_can_msgq, &rx_frame, K_MSEC(100));
        
        if (ret == 0) {
            /* Process the received frame */
            bool handled = handle_incoming_frame(&rx_frame);
            if (!handled) {
                LOG_DBG("Unhandled CAN frame: id=0x%03X dlc=%d", rx_frame.id, rx_frame.dlc);
            }
        }
        
        /* Toggle LED to show activity */
        static uint32_t led_counter = 0;
        if (++led_counter >= 10) {  /* Toggle every ~1 second */
            gpio_pin_toggle_dt(&led);
            led_counter = 0;
        }
    }
    
    return 0;
}