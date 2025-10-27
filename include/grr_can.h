#ifndef GRR_CAN_H
#define GRR_CAN_H

// #include <zephyr.h>
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
  SERVO_CONTROL = 0x4000,
  DIO = 0x5000,
  SENSORS = 0x6000,
  WARNINGS = 0x7000,
  LOGS = 0x8000,
  MOANING = 0xFFFE,
  SGA_WARRANTY = 0xFFFF
};

/* Forward declare a generic frame type so the header stays independent of
 * the low-level CAN implementation. In Zephyr you would normally use
 * struct zcan_frame (include <drivers/can.h>), but to keep this header
 * light we use void* in public prototypes and cast in the implementation.
 */

bool init_can(void);
const char *getDeviceInfo(void);
bool send_heartbeat(void);
bool handle_incoming_frame(can_frame *frame);
#endif // GRR_CAN_H
