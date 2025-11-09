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
    err = can_set_bitrate_data(can_dev, 2000000);
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


bool handle_incoming_frame(can_frame *frame)
{
    if (!frame)
        return false;
    if (frame->id == CAN_IDs::E_STOP)
        {
            LOG_WRN("E_STOP command received!");
            // Implement E_STOP handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::HALT)
        {
            LOG_WRN("HALT command received!");
            // Implement HALT handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::RESTART)
        {
            LOG_WRN("RESTART command received!");
            // Implement RESTART handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::QUERY)
        {
            LOG_INF("QUERY command received!");
            // Implement QUERY handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::ASSIGN_ID)
        {
            LOG_INF("ASSIGN_ID command received!");
            // Implement ASSIGN_ID handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::FIRMWARE)
        {
            LOG_INF("FIRMWARE command received!");
            // Implement FIRMWARE handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::MOTOR_COMMAND)
        {
            LOG_INF("MOTOR_COMMAND command received!");
            // Implement MOTOR_COMMAND handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::SERVO_CONTROL)
        {
            LOG_INF("SERVO_CONTROL command received!");
            // Implement SERVO_CONTROL handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::DIO)
        {
            LOG_INF("DIO command received!");
            // Implement DIO handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::SENSORS)
        {
            LOG_INF("SENSORS command received!");
            // Implement SENSORS handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::FATAL)
        {
            LOG_ERR("FATAL error received!");
            // Implement FATAL error handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::ERROR)
        {
            LOG_ERR("ERROR message received!");
            // Implement ERROR message handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::WARNINGS)
        {
            LOG_WRN("WARNINGS message received!");
            // Implement WARNINGS message handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::LOGS)
        {
            LOG_INF("LOGS message received!");
            // Implement LOGS message handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::MOANING)
        {
            LOG_INF("MOANING message received!");
            // Implement MOANING message handling logic here
            return true;
        }
    else if (frame->id == CAN_IDs::SGA_WARRANTY)
        {
            LOG_INF("SGA_WARRANTY message received!");
            // Implement SGA_WARRANTY message handling logic here
            return true;
        }       
        return false;


    LOG_DBG("Received frame id=0x%d dlc=%d", frame->id, frame->dlc);
    return true;
}


