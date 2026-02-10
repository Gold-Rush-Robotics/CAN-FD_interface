#ifndef GRR_CAN_H
#define GRR_CAN_H

#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/can.h>

/* CAN message IDs (keep consistent with your original enums) */
enum CAN_IDs {
    E_STOP = 0x000,
    HALT = 0x100,
    RESTART = 0x200,
    HEARTBEAT = 0x300,
    QUERY = 0x400,
    ASSIGN_ID = 0x500,
    FIRMWARE = 0x600,
    FATAL = 0x1000,
    ERROR = 0x2000,
    MOTOR_COMMAND = 0x3000,
    MOTOR_STATUS = 0x3100,
    SERVO_CONTROL = 0x4000,
    SERVO_STATUS = 0x4100,
    DIO = 0x5000,
    SENSORS = 0x6000,
    WARNINGS = 0x7000,
    LOGS = 0x8000,
    MOANING = 0xFFFE,
    SGA_WARRANTY = 0xFFFF
};

/* CAN ID masks for filtering */
#define CAN_MASK_EXACT      0x7FF       // Match exact 11-bit ID
#define CAN_MASK_GROUP      0x7F00      // Match message group (upper byte)
#define CAN_MASK_MOTOR      0x7F00      // Motor commands 0x30xx
#define CAN_MASK_SERVO      0x7F00      // Servo commands 0x40xx

/* Node ID - can be configured via ASSIGN_ID command */
extern uint8_t g_node_id;

/* CAN device accessor */
const struct device* get_can_device(void);

/* Initialization */
bool init_can(void);
bool setup_can_filters(struct k_msgq *msgq);

/* Message sending */
bool send_heartbeat(void);
bool send_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_fd);
bool send_motor_status(uint8_t motor_id, uint8_t state, int16_t velocity, 
                       int16_t position, uint8_t fault, uint8_t temp);
bool send_servo_status(uint8_t servo_id, uint8_t state, int16_t current_angle,
                       int16_t target_angle, uint8_t load, uint8_t fault);

/* Message handling */
bool handle_incoming_frame(struct can_frame *frame);

/* Device info */
const char *getDeviceInfo(void);

/* Forward declarations for motor/servo handlers - implemented in respective modules */
typedef void (*motor_command_handler_t)(const struct can_frame *frame);
typedef void (*servo_command_handler_t)(const struct can_frame *frame);

void register_motor_command_handler(motor_command_handler_t handler);
void register_servo_command_handler(servo_command_handler_t handler);

#endif // GRR_CAN_H
