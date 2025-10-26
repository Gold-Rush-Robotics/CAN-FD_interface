#include "../include/grr_can.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(grr_can, LOG_LEVEL_DBG);
#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)

/* ================================================================
 *   GLOBALS
 * ================================================================ */

static const struct device *can_dev = NULL;

/* ================================================================
 *   INITIALIZATION
 * ================================================================ */

bool init_can(void){

    can_dev = DEVICE_DT_GET(CANBUS_NODE);
	struct k_sem tx_queue_sem;
	struct can_frame frame = {0};
	int err;

	// k_sem_init(&tx_queue_sem, 3,
	// 	   3);

	if (!device_is_ready(can_dev)) {
		printk("CAN device not ready");
		return 0;
	}

    /*
     * Try CAN FD mode first. If the controller/driver doesn't support FD
     * this will usually return -ENOTSUP (operation not supported). In
     * that case we fall back to normal CAN so the board can still be used.
     */
    err = can_set_bitrate(can_dev, 1000000);
    if (err != 0) {
        LOG_ERR("Error setting CAN bitrate (err %d)", err);
        return 0;
    }
    err = can_set_mode(can_dev, CAN_MODE_FD);
    if (err == 0) {
        LOG_INF("CAN set to FD mode");
    } else {
        if (err == -EIO) {
            LOG_WRN("CAN FD not supported by driver/hardware (err %d). Falling back to NORMAL.", err);
            // return 0;
        } else {
            LOG_WRN("Setting CAN FD mode failed (err %d). Falling back to NORMAL.", err);
            // return 0;
        }

        err = can_set_mode(can_dev, CAN_MODE_NORMAL);
        if (err != 0) {
            LOG_ERR("Error setting CAN NORMAL mode (err %d)", err);
            return 0;
        }
        LOG_INF("CAN set to NORMAL mode");
    }
    err = can_set_bitrate_data(can_dev, 4000000);
    if( err != 0) {
        LOG_ERR("Error setting CAN data bitrate (err %d)", err);
        if (err == -EIO) {
            LOG_WRN("CAN FD not supported by driver/hardware (err %d). Falling back to NORMAL.", err);
            return 0;
        } 
        else if (err == -EBUSY) {
            LOG_WRN("CAN FD not supported by driver/hardware (err %d). Falling back to NORMAL.", err);
            return 0;
        }
        else if (err == -EINVAL) {
            LOG_WRN("Invalid bitrate settings for CAN FD (err %d). Falling back to NORMAL.", err);
            return 0;
        }
        else if (err == -ENOTSUP) {
            LOG_WRN("Fault in setting CAN data bitrate (err %d). Falling back to NORMAL.", err);
            // return 0;
        }
        else if (err == -ERANGE) {
            LOG_WRN("Fault in setting CAN data bitrate (err %d). Falling back to NORMAL.", err);
            return 0;
        }
        else {
            LOG_WRN("Setting CAN data bitrate failed (err %d). Falling back to NORMAL.", err);
            return 0;
        }
        // return 0;
    }

    
	err = can_start(can_dev);
	if (err != 0) {
		printk("Error starting CAN controller (err %d)", err);
		return 0;
	}

    LOG_INF("CAN device initialized: %s", can_dev->name);
    return true;
}


bool send_heartbeat(void)
{
    if (!can_dev)
    {
        LOG_INF("Heartbeat: (no CAN device) - would send HEARTBEAT message");
        return false; // Simulate success if no CAN device is available
    }

    struct can_frame tx_frame = {0};
    tx_frame.id = HEARTBEAT;
    tx_frame.dlc = 8;
    memset(tx_frame.data, 0, sizeof(tx_frame.data));

    int ret = can_send(can_dev, &tx_frame, K_MSEC(100), NULL, NULL);
    if (ret < 0)
    {
        LOG_ERR("Failed to send heartbeat (%d)", ret);
        return false;
    }
    else
    {
        LOG_DBG("Heartbeat sent");
        return true;
    }
}


bool handle_incoming_frame(void *frame)
{
    if (!frame)
        return false;

    struct can_frame *f = (struct can_frame *)frame;
    LOG_DBG("Received frame id=0x%X dlc=%d", f->id, f->dlc);
    return true;
}

void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (!frame)
        return;

    LOG_DBG("Received frame id=0x%X dlc=%d", frame->id, frame->dlc);
    // Process the received frame as needed
}
