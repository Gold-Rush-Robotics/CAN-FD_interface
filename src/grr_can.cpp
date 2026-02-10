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
uint8_t g_node_id = 0x01;  /* Default node ID, can be changed via ASSIGN_ID */

/* Callback function pointers for motor and servo handlers */
static motor_command_handler_t motor_handler = NULL;
static servo_command_handler_t servo_handler = NULL;

/* E-STOP flag - global emergency stop state */
static volatile bool g_estop_active = false;

/* ================================================================
 *   DEVICE ACCESSOR
 * ================================================================ */

const struct device* get_can_device(void)
{
    return can_dev;
}

/* ================================================================
 *   HANDLER REGISTRATION
 * ================================================================ */

void register_motor_command_handler(motor_command_handler_t handler)
{
    motor_handler = handler;
    LOG_INF("Motor command handler registered");
}

void register_servo_command_handler(servo_command_handler_t handler)
{
    servo_handler = handler;
    LOG_INF("Servo command handler registered");
}

/* ================================================================
 *   CAN FILTER CALLBACK
 * ================================================================ */

static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct k_msgq *msgq = (struct k_msgq *)user_data;
    
    if (frame == NULL) {
        return;
    }
    
    /* Put frame into message queue for processing in main context */
    int ret = k_msgq_put(msgq, frame, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("CAN RX queue full, frame dropped (id=0x%03X)", frame->id);
    }
}

/* ================================================================
 *   INITIALIZATION
 * ================================================================ */

bool init_can(void)
{
    can_dev = DEVICE_DT_GET(CANBUS_NODE);
    int err;

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return false;
    }

    /* Set nominal bitrate (1 Mbps for CAN FD) */
    err = can_set_bitrate(can_dev, 1000000);
    if (err != 0) {
        LOG_ERR("Error setting CAN bitrate (err %d)", err);
        return false;
    }

    /* Try CAN FD mode first */
    err = can_set_mode(can_dev, CAN_MODE_FD);
    if (err == 0) {
        LOG_INF("CAN set to FD mode");
        
        /* Set data bitrate for FD mode (2 Mbps) */
        err = can_set_bitrate_data(can_dev, 2000000);
        if (err != 0) {
            LOG_WRN("Error setting CAN data bitrate (err %d)", err);
            if (err == -ENOTSUP) {
                LOG_WRN("Data bitrate setting not supported, continuing");
            } else {
                /* Fall back to normal mode */
                err = can_set_mode(can_dev, CAN_MODE_NORMAL);
                if (err != 0) {
                    LOG_ERR("Error setting CAN NORMAL mode (err %d)", err);
                    return false;
                }
                LOG_INF("CAN set to NORMAL mode (FD data bitrate failed)");
            }
        }
    } else {
        LOG_WRN("CAN FD mode not supported (err %d), using NORMAL mode", err);
        err = can_set_mode(can_dev, CAN_MODE_NORMAL);
        if (err != 0) {
            LOG_ERR("Error setting CAN NORMAL mode (err %d)", err);
            return false;
        }
        LOG_INF("CAN set to NORMAL mode");
    }

    /* Start the CAN controller */
    err = can_start(can_dev);
    if (err != 0) {
        LOG_ERR("Error starting CAN controller (err %d)", err);
        return false;
    }

    LOG_INF("CAN device initialized: %s", can_dev->name);
    return true;
}

bool setup_can_filters(struct k_msgq *msgq)
{
    if (!can_dev || !msgq) {
        LOG_ERR("CAN device or message queue not available");
        return false;
    }

    int filter_id;
    struct can_filter filter;

    /* Filter 1: E-STOP (highest priority, exact match) */
    filter.id = E_STOP;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add E-STOP filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added E-STOP filter (id=%d)", filter_id);

    /* Filter 2: HALT command */
    filter.id = HALT;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add HALT filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added HALT filter (id=%d)", filter_id);

    /* Filter 3: RESTART command */
    filter.id = RESTART;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add RESTART filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added RESTART filter (id=%d)", filter_id);

    /* Filter 4: HEARTBEAT */
    filter.id = HEARTBEAT;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add HEARTBEAT filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added HEARTBEAT filter (id=%d)", filter_id);

    /* Filter 5: Motor commands (0x3000 - 0x30FF) */
    filter.id = MOTOR_COMMAND;
    filter.mask = 0x7F00;  /* Match 0x30xx */
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add MOTOR filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added MOTOR_COMMAND filter (id=%d)", filter_id);

    /* Filter 6: Servo commands (0x4000 - 0x40FF) */
    filter.id = SERVO_CONTROL;
    filter.mask = 0x7F00;  /* Match 0x40xx */
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_ERR("Failed to add SERVO filter (err %d)", filter_id);
        return false;
    }
    LOG_DBG("Added SERVO_CONTROL filter (id=%d)", filter_id);

    /* Filter 7: Query and configuration commands */
    filter.id = QUERY;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_WRN("Failed to add QUERY filter (err %d)", filter_id);
    }

    /* Filter 8: ASSIGN_ID */
    filter.id = ASSIGN_ID;
    filter.mask = CAN_MASK_EXACT;
    filter.flags = CAN_FILTER_DATA;
    
    filter_id = can_add_rx_filter_msgq(can_dev, msgq, &filter);
    if (filter_id < 0) {
        LOG_WRN("Failed to add ASSIGN_ID filter (err %d)", filter_id);
    }

    LOG_INF("CAN filters configured successfully");
    return true;
}

/* ================================================================
 *   MESSAGE SENDING
 * ================================================================ */

bool send_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_fd)
{
    if (!can_dev) {
        LOG_ERR("CAN device not initialized");
        return false;
    }

    struct can_frame tx_frame = {0};
    tx_frame.id = id;
    tx_frame.dlc = len;
    
    if (is_fd) {
        tx_frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;
    }
    
    if (data && len > 0) {
        memcpy(tx_frame.data, data, len);
    }

    int ret = can_send(can_dev, &tx_frame, K_MSEC(100), NULL, NULL);
    if (ret < 0) {
        LOG_ERR("Failed to send CAN frame id=0x%03X (err %d)", id, ret);
        return false;
    }
    
    return true;
}

bool send_heartbeat(void)
{
    if (!can_dev) {
        LOG_WRN("Heartbeat: CAN device not available");
        return false;
    }

    uint8_t data[8] = {0};
    data[0] = g_node_id;
    data[1] = g_estop_active ? 0x01 : 0x00;  /* Status byte */
    
    /* Add timestamp (lower 4 bytes of uptime in ms) */
    uint32_t uptime_ms = (uint32_t)k_uptime_get();
    data[2] = (uptime_ms >> 0) & 0xFF;
    data[3] = (uptime_ms >> 8) & 0xFF;
    data[4] = (uptime_ms >> 16) & 0xFF;
    data[5] = (uptime_ms >> 24) & 0xFF;

    bool result = send_can_frame(HEARTBEAT, data, 8, false);
    if (result) {
        LOG_DBG("Heartbeat sent (node=%d)", g_node_id);
    }
    return result;
}

bool send_motor_status(uint8_t motor_id, uint8_t state, int16_t velocity, 
                       int16_t position, uint8_t fault, uint8_t temp)
{
    uint8_t data[8];
    data[0] = motor_id;
    data[1] = state;
    data[2] = (velocity >> 0) & 0xFF;
    data[3] = (velocity >> 8) & 0xFF;
    data[4] = (position >> 0) & 0xFF;
    data[5] = (position >> 8) & 0xFF;
    data[6] = fault;
    data[7] = temp;
    
    return send_can_frame(MOTOR_STATUS | motor_id, data, 8, false);
}

bool send_servo_status(uint8_t servo_id, uint8_t state, int16_t current_angle,
                       int16_t target_angle, uint8_t load, uint8_t fault)
{
    uint8_t data[8];
    data[0] = servo_id;
    data[1] = state;
    data[2] = (current_angle >> 0) & 0xFF;
    data[3] = (current_angle >> 8) & 0xFF;
    data[4] = (target_angle >> 0) & 0xFF;
    data[5] = (target_angle >> 8) & 0xFF;
    data[6] = load;
    data[7] = fault;
    
    return send_can_frame(SERVO_STATUS | servo_id, data, 8, false);
}

/* ================================================================
 *   MESSAGE HANDLING
 * ================================================================ */

bool handle_incoming_frame(struct can_frame *frame)
{
    if (!frame) {
        return false;
    }

    LOG_DBG("RX frame: id=0x%03X dlc=%d", frame->id, frame->dlc);

    /* Handle E-STOP with highest priority */
    if (frame->id == E_STOP) {
        LOG_WRN("E_STOP command received!");
        g_estop_active = true;
        
        /* Dispatch to motor handler for emergency stop */
        if (motor_handler) {
            motor_handler(frame);
        }
        /* Dispatch to servo handler for emergency stop */
        if (servo_handler) {
            servo_handler(frame);
        }
        return true;
    }
    
    /* Handle HALT */
    if (frame->id == HALT) {
        LOG_WRN("HALT command received!");
        g_estop_active = true;
        return true;
    }
    
    /* Handle RESTART - clears E-STOP state */
    if (frame->id == RESTART) {
        LOG_INF("RESTART command received - clearing E-STOP");
        g_estop_active = false;
        return true;
    }
    
    /* Handle HEARTBEAT (from other nodes) */
    if (frame->id == HEARTBEAT) {
        if (frame->dlc >= 1) {
            uint8_t sender_id = frame->data[0];
            LOG_DBG("Received heartbeat from node %d", sender_id);
        }
        return true;
    }
    
    /* Handle QUERY */
    if (frame->id == QUERY) {
        LOG_INF("QUERY command received");
        /* Respond with device info */
        send_heartbeat();
        return true;
    }
    
    /* Handle ASSIGN_ID */
    if (frame->id == ASSIGN_ID) {
        if (frame->dlc >= 1) {
            g_node_id = frame->data[0];
            LOG_INF("Node ID assigned: %d", g_node_id);
        }
        return true;
    }
    
    /* Handle FIRMWARE */
    if (frame->id == FIRMWARE) {
        LOG_INF("FIRMWARE command received");
        /* Firmware update handling would go here */
        return true;
    }
    
    /* Handle Motor commands (0x3000 - 0x30FF) */
    if ((frame->id & 0xFF00) == MOTOR_COMMAND) {
        LOG_DBG("MOTOR_COMMAND received (id=0x%03X)", frame->id);
        
        if (g_estop_active) {
            LOG_WRN("Motor command ignored - E-STOP active");
            return false;
        }
        
        if (motor_handler) {
            motor_handler(frame);
        } else {
            LOG_WRN("No motor handler registered");
        }
        return true;
    }
    
    /* Handle Servo commands (0x4000 - 0x40FF) */
    if ((frame->id & 0xFF00) == SERVO_CONTROL) {
        LOG_DBG("SERVO_CONTROL received (id=0x%03X)", frame->id);
        
        if (g_estop_active) {
            LOG_WRN("Servo command ignored - E-STOP active");
            return false;
        }
        
        if (servo_handler) {
            servo_handler(frame);
        } else {
            LOG_WRN("No servo handler registered");
        }
        return true;
    }
    
    /* Handle DIO */
    if (frame->id == DIO) {
        LOG_INF("DIO command received");
        return true;
    }
    
    /* Handle SENSORS */
    if (frame->id == SENSORS) {
        LOG_INF("SENSORS command received");
        return true;
    }
    
    /* Handle error/warning messages */
    if (frame->id == FATAL) {
        LOG_ERR("FATAL error received from CAN!");
        g_estop_active = true;
        return true;
    }
    
    if (frame->id == ERROR) {
        LOG_ERR("ERROR message received");
        return true;
    }
    
    if (frame->id == WARNINGS) {
        LOG_WRN("WARNINGS message received");
        return true;
    }
    
    if (frame->id == LOGS) {
        LOG_INF("LOGS message received");
        return true;
    }

    LOG_DBG("Unhandled frame id=0x%03X", frame->id);
    return false;
}

/* ================================================================
 *   DEVICE INFO
 * ================================================================ */

const char *getDeviceInfo(void)
{
    if (can_dev) {
        return can_dev->name;
    }
    return "CAN device not initialized";
}


