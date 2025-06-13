#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(lora_gateway, LOG_LEVEL_INF);

/* LoRa Configuration */
#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
#define MAX_NODES 16
#define NODE_TIMEOUT_MS 30000  // 30 seconds timeout

/* LED Configuration */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0});
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0});

/* Global Variables */
static const struct device *const lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
static uint32_t total_messages_received = 0;

/* LoRa Configuration */
static struct lora_modem_config config = {
    .frequency = 434000000,
    .bandwidth = BW_125_KHZ,
    .datarate = SF_10,
    .preamble_len = 8,
    .coding_rate = CR_4_5,
    .tx_power = 14,
    .tx = true,
};

/* Sensor Message Structure (must match sensor node) */
struct sensor_message {
    uint8_t node_id;
    uint8_t msg_type;
    uint16_t seq_num;
    float temperature;
    float humidity;
    uint16_t battery_mv;
    uint16_t crc;
};

/* Node tracking structure */
struct node_info {
    uint8_t node_id;
    uint32_t last_seen;
    uint32_t message_count;
    int16_t last_rssi;
    int8_t last_snr;
    float last_temperature;
    float last_humidity;
    uint16_t last_battery;
    bool active;
};

static struct node_info nodes[MAX_NODES];
static uint8_t active_node_count = 0;

/* Function Prototypes */
static void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size,
                           int16_t rssi, int8_t snr, void *user_data);
static void process_sensor_data(struct sensor_message *msg, int16_t rssi, int8_t snr);
static void send_acknowledgment(uint8_t node_id, uint16_t seq_num);
static uint16_t calculate_crc(uint8_t *data, uint16_t len);
static void update_node_info(struct sensor_message *msg, int16_t rssi, int8_t snr);
static void print_gateway_status(void);
static void led_activity_indication(void);

/* Calculate CRC (same as sensor node) */
static uint16_t calculate_crc(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* LED Activity Indication */
static void led_activity_indication(void)
{
    if (gpio_is_ready_dt(&led0)) {
        gpio_pin_set_dt(&led0, 1);
        k_msleep(100);
        gpio_pin_set_dt(&led0, 0);
    }
}

/* Update Node Information */
static void update_node_info(struct sensor_message *msg, int16_t rssi, int8_t snr)
{
    struct node_info *node = NULL;
    
    // Find existing node or create new one
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].node_id == msg->node_id && nodes[i].active) {
            node = &nodes[i];
            break;
        }
    }
    
    // If not found, create new node entry
    if (!node) {
        for (int i = 0; i < MAX_NODES; i++) {
            if (!nodes[i].active) {
                node = &nodes[i];
                node->node_id = msg->node_id;
                node->active = true;
                node->message_count = 0;
                active_node_count++;
                LOG_INF("New node registered: 0x%02X", msg->node_id);
                break;
            }
        }
    }
    
    if (node) {
        node->last_seen = k_uptime_get_32();
        node->message_count++;
        node->last_rssi = rssi;
        node->last_snr = snr;
        node->last_temperature = msg->temperature;
        node->last_humidity = msg->humidity;
        node->last_battery = msg->battery_mv;
    }
}

/* Send Acknowledgment */
static void send_acknowledgment(uint8_t node_id, uint16_t seq_num)
{
    uint8_t ack_msg[4] = {0xAA, node_id, (seq_num >> 8) & 0xFF, seq_num & 0xFF};
    
    int ret = lora_send(lora_dev, ack_msg, sizeof(ack_msg));
    if (ret < 0) {
        LOG_WRN("ACK send failed: %d", ret);
    } else {
        LOG_DBG("ACK sent to node 0x%02X, seq %u", node_id, seq_num);
    }
}

/* Process Sensor Data */
static void process_sensor_data(struct sensor_message *msg, int16_t rssi, int8_t snr)
{
    // Verify CRC
    uint16_t calculated_crc = calculate_crc((uint8_t*)msg, sizeof(*msg) - sizeof(msg->crc));
    if (calculated_crc != msg->crc) {
        LOG_WRN("CRC mismatch for node 0x%02X", msg->node_id);
        return;
    }
    
    // Update node information
    update_node_info(msg, rssi, snr);
    
    // Log received data
    LOG_INF("Node 0x%02X [%u]: T=%.1f°C H=%.1f%% B=%umV RSSI=%d SNR=%d",
            msg->node_id, msg->seq_num, 
            msg->temperature, msg->humidity, msg->battery_mv,
            rssi, snr);
    
    // Format as JSON for external systems
    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer),
        "{\"node_id\":\"0x%02X\",\"seq\":%u,\"temp\":%.1f,\"humidity\":%.1f,"
        "\"battery\":%u,\"rssi\":%d,\"snr\":%d,\"timestamp\":%u}",
        msg->node_id, msg->seq_num, msg->temperature, msg->humidity,
        msg->battery_mv, rssi, snr, k_uptime_get_32());
    
    LOG_INF("JSON: %s", json_buffer);
    
    // Send acknowledgment
    send_acknowledgment(msg->node_id, msg->seq_num);
    
    total_messages_received++;
}

/* LoRa Receive Callback */
static void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size,
                           int16_t rssi, int8_t snr, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    
    led_activity_indication();
    
    if (size == sizeof(struct sensor_message)) {
        struct sensor_message *msg = (struct sensor_message *)data;
        if (msg->msg_type == 0x01) {  // Sensor data
            process_sensor_data(msg, rssi, snr);
        }
    } else {
        LOG_WRN("Unexpected message size: %u", size);
    }
}

/* Print Gateway Status */
static void print_gateway_status(void)
{
    uint32_t uptime = k_uptime_get_32();
    
    LOG_INF("=== Gateway Status ===");
    LOG_INF("Uptime: %u ms", uptime);
    LOG_INF("Active nodes: %u", active_node_count);
    LOG_INF("Total messages: %u", total_messages_received);
    
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].active) {
            uint32_t age = uptime - nodes[i].last_seen;
            LOG_INF("Node 0x%02X: %u msgs, age %u ms, T=%.1f°C, H=%.1f%%, B=%umV, RSSI=%d",
                    nodes[i].node_id, nodes[i].message_count, age,
                    nodes[i].last_temperature, nodes[i].last_humidity,
                    nodes[i].last_battery, nodes[i].last_rssi);
        }
    }
    LOG_INF("=====================");
}

/* Initialize LoRa */
static int init_lora(void)
{
    int ret;

    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
        return -ENODEV;
    }

    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed: %d", ret);
        return ret;
    }

    ret = lora_recv_async(lora_dev, lora_receive_cb, NULL);
    if (ret < 0) {
        LOG_ERR("LoRa RX setup failed: %d", ret);
        return ret;
    }

    LOG_INF("LoRa gateway initialized successfully");
    return 0;
}

/* Initialize LEDs */
static int init_leds(void)
{
    int ret = 0;

    if (gpio_is_ready_dt(&led0)) {
        ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("LED0 config failed: %d", ret);
            return ret;
        }
    }

    if (gpio_is_ready_dt(&led1)) {
        ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("LED1 config failed: %d", ret);
        }
    }

    if (gpio_is_ready_dt(&led2)) {
        ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("LED2 config failed: %d", ret);
        }
    }

    LOG_INF("LEDs initialized");
    return 0;
}

/* Status Thread */
static void status_thread(void)
{
    while (1) {
        k_msleep(30000);  // Print status every 30 seconds
        print_gateway_status();
    }
}

K_THREAD_DEFINE(status_tid, 1024, status_thread, NULL, NULL, NULL, 7, 0, 0);

/* Main Function */
int main(void)
{
    int ret;

    LOG_INF("LoRa Gateway Starting...");

    /* Initialize LEDs */
    ret = init_leds();
    if (ret < 0) {
        LOG_ERR("LED init failed: %d", ret);
        return ret;
    }

    /* Initialize LoRa */
    ret = init_lora();
    if (ret < 0) {
        LOG_ERR("LoRa init failed: %d", ret);
        return ret;
    }

    /* Startup LED sequence */
    for (int i = 0; i < 3; i++) {
        if (gpio_is_ready_dt(&led0)) gpio_pin_set_dt(&led0, 1);
        if (gpio_is_ready_dt(&led1)) gpio_pin_set_dt(&led1, 1);
        if (gpio_is_ready_dt(&led2)) gpio_pin_set_dt(&led2, 1);
        k_msleep(200);
        if (gpio_is_ready_dt(&led0)) gpio_pin_set_dt(&led0, 0);
        if (gpio_is_ready_dt(&led1)) gpio_pin_set_dt(&led1, 0);
        if (gpio_is_ready_dt(&led2)) gpio_pin_set_dt(&led2, 0);
        k_msleep(200);
    }

    LOG_INF("Gateway ready - listening for sensor nodes");

    /* Main loop */
    while (1) {
        k_msleep(1000);
        
        // Check for node timeouts
        uint32_t current_time = k_uptime_get_32();
        for (int i = 0; i < MAX_NODES; i++) {
            if (nodes[i].active && 
                (current_time - nodes[i].last_seen) > NODE_TIMEOUT_MS) {
                LOG_WRN("Node 0x%02X timeout", nodes[i].node_id);
                nodes[i].active = false;
                active_node_count--;
            }
        }
    }

    return 0;
}