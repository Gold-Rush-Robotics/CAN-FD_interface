#include "../include/grr_can.h"
#include <logging/log.h>
#include <drivers/can.h>

LOG_MODULE_REGISTER(grr_can, LOG_LEVEL_DBG);

static const struct device *can_dev = NULL;

void init_can(void)
{
  can_dev = device_get_binding(DT_LABEL(DT_NODELABEL(can0)));
  if (!can_dev) {
    LOG_WRN("CAN device not found (device_get_binding returned NULL)");
  } else {
    LOG_INF("CAN device initialized: %p", can_dev);
  }
}

void send_heartbeat(void)
{
  if (!can_dev) {
    LOG_INF("Heartbeat: (no CAN device) - would send HEARTBEAT message");
    return;
  }

  /* Build a zcan_frame and send - simplified example. Real code needs to
   * follow your message layout and use can_send() with proper params.
   */
  struct zcan_frame tx_frame = {0};
  tx_frame.id = HEARTBEAT;
  tx_frame.dlc = 8;
  for (int i = 0; i < 8; ++i) {
    tx_frame.data[i] = 0;
  }

  int ret = can_send(can_dev, &tx_frame, K_MSEC(100), NULL, NULL);
  if (ret < 0) {
    LOG_ERR("Failed to send heartbeat (%d)", ret);
  } else {
    LOG_DBG("Heartbeat sent");
  }
}

bool handle_incoming_frame(void *frame)
{
  if (!frame) return false;
  struct zcan_frame *f = (struct zcan_frame *)frame;
  LOG_DBG("Received frame id=0x%X dlc=%d", f->id, f->dlc);
  /* Implement parsing similar to your original code here. For now, just
   * return true to indicate a frame was handled.
   */
  return true;
}
