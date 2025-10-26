#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "../include/grr_can.h"

LOG_MODULE_REGISTER(grr_main);

static struct k_timer hb_timer;
static struct k_timer led_timer;

// LED configuration
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void led_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    static bool led_state = false;
    gpio_pin_set_dt(&led, led_state);
    led_state = !led_state;
}

void hb_timeout(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    LOG_DBG("hb_timeout fired");
    bool sent = send_heartbeat();
	if (!sent) {
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_SECONDS(10));
		gpio_pin_set_dt(&led, 0);
		LOG_ERR("Failed to send heartbeat");

	} else {
		LOG_DBG("Heartbeat sent successfully");
	}
}

int main()
{
    // Add delay for USB CDC initialization
    k_sleep(K_MSEC(1000));
    
    // Explicitly flush any pending messages
    printk("\n\n\r");
    
    LOG_INF("Starting GRR_CAN Zephyr app");
    // Initialize LED
    if (!device_is_ready(led.port)) {
        LOG_ERR("LED device not ready");
        return 1;
    }
    
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("LED pin configuration failed");
        //turn led on for 10 seconds to indicate error
		return 1;
    }
    bool configured = init_can();
	if (!configured) {
		LOG_ERR("CAN initialization failed");
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_SECONDS(10));
		gpio_pin_set_dt(&led, 0);
		return 1;
	}
	else {
		LOG_INF("CAN initialized successfully");
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_SECONDS(1));
		gpio_pin_set_dt(&led, 0);

	}


    k_timer_init(&hb_timer, hb_timeout, NULL);
    k_timer_start(&hb_timer, K_SECONDS(1), K_SECONDS(1));

    // Initialize and start LED timer
    k_timer_init(&led_timer, led_timeout, NULL);
    k_timer_start(&led_timer, K_MSEC(500), K_MSEC(500));  // Blink every 500ms

    while (1) {
        k_sleep(K_SECONDS(10));
		printk("Main loop is running...\n");
    }
}