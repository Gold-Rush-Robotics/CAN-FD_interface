#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/bootloader/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include "grr_can.h"

// CAN bootloader message IDs
#define BOOTLOADER_MSG_ID 0x600
#define BOOTLOADER_RESPONSE_ID 0x601

// Command types
#define CMD_PING 0x01
#define CMD_ERASE 0x02
#define CMD_WRITE 0x03
#define CMD_READ 0x04
#define CMD_VERIFY 0x05
#define CMD_RESET 0x06

// Response codes
#define RESP_OK 0x00
#define RESP_ERROR 0x01

// Flash parameters
#define FLASH_AREA_IMAGE_PRIMARY FIXED_PARTITION_ID(slot0_partition)
#define FLASH_AREA_IMAGE_SECONDARY FIXED_PARTITION_ID(slot1_partition)
#define MAX_DATA_SIZE 64

struct flash_img_context flash_ctx;
const struct device *can_dev;

// Bootloader state
static struct {
    bool in_bootloader;
    uint32_t current_address;
    uint32_t expected_size;
} boot_state;

// Function declarations
static void send_response(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len);
static void process_bootloader_msg(const struct can_frame *frame);

void can_bootloader_init(const struct device *can_device) {
    can_dev = can_device;
    boot_state.in_bootloader = false;
    boot_state.current_address = 0;
    boot_state.expected_size = 0;
}

// CAN message callback
void bootloader_can_callback(const struct can_frame *frame, void *user_data) {
    if (frame->id == BOOTLOADER_MSG_ID) {
        process_bootloader_msg(frame);
    }
}

static void process_bootloader_msg(const struct can_frame *frame) {
    uint8_t cmd = frame->data[0];
    uint8_t response[8] = {cmd, RESP_OK};

    switch (cmd) {
        case CMD_PING:
            send_response(CMD_PING, RESP_OK, NULL, 0);
            break;

        case CMD_ERASE:
            // Prepare for firmware update
            if (flash_img_prepare(&flash_ctx) == 0) {
                boot_state.in_bootloader = true;
                boot_state.current_address = 0;
                boot_state.expected_size = (frame->data[1] << 24) | 
                                         (frame->data[2] << 16) |
                                         (frame->data[3] << 8) |
                                         frame->data[4];
                send_response(CMD_ERASE, RESP_OK, NULL, 0);
            } else {
                send_response(CMD_ERASE, RESP_ERROR, NULL, 0);
            }
            break;

        case CMD_WRITE:
            if (!boot_state.in_bootloader) {
                send_response(CMD_WRITE, RESP_ERROR, NULL, 0);
                return;
            }

            uint16_t offset = (frame->data[1] << 8) | frame->data[2];
            uint8_t size = frame->data[3];
            
            if (flash_img_buffered_write(&flash_ctx, &frame->data[4], size, false) == 0) {
                boot_state.current_address += size;
                send_response(CMD_WRITE, RESP_OK, NULL, 0);
            } else {
                send_response(CMD_WRITE, RESP_ERROR, NULL, 0);
            }
            break;

        case CMD_VERIFY:
            if (!boot_state.in_bootloader) {
                send_response(CMD_VERIFY, RESP_ERROR, NULL, 0);
                return;
            }

            if (flash_img_buffered_write(&flash_ctx, NULL, 0, true) == 0) {
                // Set the upgrade request
                boot_request_upgrade(false);
                send_response(CMD_VERIFY, RESP_OK, NULL, 0);
            } else {
                send_response(CMD_VERIFY, RESP_ERROR, NULL, 0);
            }
            break;

        case CMD_RESET:
            send_response(CMD_RESET, RESP_OK, NULL, 0);
            k_msleep(100);  // Wait for response to be sent
            sys_reboot(SYS_REBOOT_COLD);
            break;

        default:
            send_response(cmd, RESP_ERROR, NULL, 0);
            break;
    }
}

static void send_response(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len) {
    struct can_frame frame = {
        .id = BOOTLOADER_RESPONSE_ID,
        .dlc = len + 2,  // cmd + status + data
        .data = {cmd, status}
    };

    if (data && len > 0) {
        memcpy(&frame.data[2], data, len);
    }

    can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
}
