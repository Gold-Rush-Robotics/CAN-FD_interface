#include "../include/grr_can.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <string.h>

LOG_MODULE_REGISTER(grr_can, LOG_LEVEL_DBG);

/* ================================================================
 *   GLOBALS
 * ================================================================ */

static const struct device *can_dev = NULL;

/* ================================================================
 *   INITIALIZATION
 * ================================================================ */

void init_can(void){

    can_dev = DEVICE_DT_GET(DT_NODELABEL(flexcan3));

    if (!can_dev || !device_is_ready(can_dev))
    {
        LOG_WRN("CAN device not ready or not found");
        can_dev = NULL;
        
    }
    LOG_INF("CAN device initialized: %s", can_dev->name);
}


void send_heartbeat(void)
{
    if (!can_dev)
    {
        LOG_INF("Heartbeat: (no CAN device) - would send HEARTBEAT message");
        return;
    }

    struct can_frame tx_frame = {0};
    tx_frame.id = HEARTBEAT;
    tx_frame.dlc = 8;
    memset(tx_frame.data, 0, sizeof(tx_frame.data));

    int ret = can_send(can_dev, &tx_frame, K_MSEC(100), NULL, NULL);
    if (ret < 0)
    {
        LOG_ERR("Failed to send heartbeat (%d)", ret);
    }
    else
    {
        LOG_DBG("Heartbeat sent");
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

