#ifndef CAN_BOOTLOADER_H
#define CAN_BOOTLOADER_H

#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

void can_bootloader_init(const struct device *can_device);
void bootloader_can_callback(const struct can_frame *frame, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BOOTLOADER_H */