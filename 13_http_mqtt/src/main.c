#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_ip.h>

LOG_MODULE_REGISTER(mqtt_temp_humid, LOG_LEVEL_DBG);

/* MQTT broker configuration */
#define MQTT_BROKER_ADDR "74.225.192.138"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "nucleo_f746zg_client"

/* Topics for publishing */
#define TOPIC_TEMPERATURE "sensor/temperature"
#define TOPIC_HUMIDITY "sensor/humidity"

/* Network and MQTT client variables */
static struct mqtt_client client_ctx;
static uint8_t rx_buffer[128];
static uint8_t tx_buffer[128];
static struct sockaddr_storage broker;
static bool connected = false;



/* Network management callback */
static struct net_mgmt_event_callback mgmt_cb;

static void mqtt_evt_handler(struct mqtt_client *const client,
                           const struct mqtt_evt *evt)
{
    switch (evt->type) {
    case MQTT_EVT_CONNACK:
        if (evt->result != 0) {
            LOG_ERR("MQTT connect failed %d", evt->result);
            connected = false;
            break;
        }
        connected = true;
        LOG_INF("MQTT client connected!");
        break;

    case MQTT_EVT_DISCONNECT:
        LOG_INF("MQTT client disconnected %d", evt->result);
        connected = false;
        break;

    case MQTT_EVT_PUBLISH:
        LOG_INF("MQTT publish event");
        break;

    case MQTT_EVT_SUBACK:
        LOG_INF("MQTT subscribe acknowledged");
        break;

    case MQTT_EVT_UNSUBACK:
        LOG_INF("MQTT unsubscribe acknowledged");
        break;

    default:
        LOG_INF("MQTT event type: %d", evt->type);
        break;
    }
}

static void broker_init(void)
{
    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

    broker4->sin_family = AF_INET;
    broker4->sin_port = htons(MQTT_BROKER_PORT);
    
    /* Manually set IP address: 74.225.192.138 */
    broker4->sin_addr.s_addr = htonl((74UL << 24) | (225UL << 16) | (192UL << 8) | 138UL);
}

static void client_init(struct mqtt_client *client)
{
    mqtt_client_init(client);

    broker_init();

    /* MQTT client configuration */
    client->broker = &broker;
    client->evt_cb = mqtt_evt_handler;
    client->client_id.utf8 = (uint8_t *)MQTT_CLIENT_ID;
    client->client_id.size = strlen(MQTT_CLIENT_ID);
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    /* MQTT buffers configuration */
    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    /* Set keep alive to 60 seconds */
    client->keepalive = 60U;
}

static int publish_data(struct mqtt_client *client, const char *topic, const char *data)
{
    struct mqtt_publish_param param;
    
    param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
    param.message.topic.topic.utf8 = (uint8_t *)topic;
    param.message.topic.topic.size = strlen(topic);
    param.message.payload.data = (uint8_t *)data;
    param.message.payload.len = strlen(data);
    param.message_id = sys_rand32_get();
    param.dup_flag = 0U;
    param.retain_flag = 0U;

    return mqtt_publish(client, &param);
}

static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                 uint32_t mgmt_event,
                                 struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_IPV4_DHCP_BOUND:
        LOG_INF("IPv4 DHCP bound");
        break;
    default:
        break;
    }
}

static int wait_for_network(void)
{
    struct net_if *iface;
    struct net_if_config *cfg;
    int i = 0;

    iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface found");
        return -1;
    }

    cfg = net_if_get_config(iface);
    if (!cfg) {
        LOG_ERR("No network configuration found");
        return -1;
    }

    /* Wait for network to be ready */
    while (!net_if_is_up(iface) && i < 100) {
        k_sleep(K_MSEC(100));
        i++;
    }

    if (!net_if_is_up(iface)) {
        LOG_ERR("Network interface not ready");
        return -1;
    }

    /* Start DHCP if not already started */
    net_dhcpv4_start(iface);

    /* Wait for IP address */
    i = 0;
    while (!net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) && i < 100) {
        k_sleep(K_MSEC(500));
        i++;
    }

    if (!net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)) {
        LOG_ERR("Failed to get IP address");
        return -1;
    }

    LOG_INF("Network ready!");
    return 0;
}

int main(void)
{
    int rc;
    char temp_str[32];
    char humid_str[32];
    int temperature, humidity;
    
    LOG_INF("Starting MQTT Temperature/Humidity Publisher");

    /* Initialize network management callback */
    net_mgmt_init_event_callback(&mgmt_cb, net_mgmt_event_handler,
                                NET_EVENT_IPV4_DHCP_BOUND);
    net_mgmt_add_event_callback(&mgmt_cb);

    /* Wait for network to be ready */
    if (wait_for_network() < 0) {
        LOG_ERR("Network initialization failed");
        return -1;
    }

    /* Initialize MQTT client */
    client_init(&client_ctx);

    while (1) {
        if (!connected) {
            LOG_INF("Attempting to connect to MQTT broker...");
            rc = mqtt_connect(&client_ctx);
            if (rc != 0) {
                LOG_ERR("Failed to connect to MQTT broker: %d", rc);
                k_sleep(K_SECONDS(5));
                continue;
            }

            /* Process MQTT events and wait for connection */
            for (int i = 0; i < 50; i++) {  // Wait up to 5 seconds
                mqtt_input(&client_ctx);
                if (connected) {
                    break;
                }
                k_sleep(K_MSEC(100));
            }
            
            if (!connected) {
                LOG_ERR("MQTT connection timeout");
                /* Reset client for next attempt */
                client_init(&client_ctx);
                k_sleep(K_SECONDS(5));
                continue;
            }
        }

        /* Generate random temperature (15-35°C) and humidity (30-80%) */
        temperature = 15 + (sys_rand32_get() % 21);
        humidity = 30 + (sys_rand32_get() % 51);
        
        /* Format data as JSON */
        snprintf(temp_str, sizeof(temp_str), "{\"temperature\": %d}", temperature);
        snprintf(humid_str, sizeof(humid_str), "{\"humidity\": %d}", humidity);

        /* Publish temperature */
        rc = publish_data(&client_ctx, TOPIC_TEMPERATURE, temp_str);
        if (rc != 0) {
            LOG_ERR("Failed to publish temperature: %d", rc);
            connected = false;  // Force reconnection
        } else {
            LOG_INF("Published temperature: %s", temp_str);
        }

        k_sleep(K_MSEC(500));

        /* Publish humidity */
        rc = publish_data(&client_ctx, TOPIC_HUMIDITY, humid_str);
        if (rc != 0) {
            LOG_ERR("Failed to publish humidity: %d", rc);
            connected = false;  // Force reconnection
        } else {
            LOG_INF("Published humidity: %s", humid_str);
        }

        /* Process incoming MQTT messages */
        mqtt_input(&client_ctx);

        /* Keep MQTT connection alive */
        mqtt_live(&client_ctx);

        /* Wait 10 seconds before next publish */
        k_sleep(K_SECONDS(10));
    }

    return 0;
}