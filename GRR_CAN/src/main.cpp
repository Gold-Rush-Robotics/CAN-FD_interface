#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include "../include/grr_can.h"

LOG_MODULE_REGISTER(grr_main, LOG_LEVEL_INF);

static struct k_timer hb_timer;

void hb_timeout(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);
	send_heartbeat();
}

void main(void)
{
	LOG_INF("Starting GRR_CAN Zephyr app");
	init_can();

	k_timer_init(&hb_timer, hb_timeout, NULL);
	k_timer_start(&hb_timer, K_SECONDS(1), K_SECONDS(1));

	while (1) {
		k_sleep(K_SECONDS(10));
	}
}
