

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_if.h>
#include <zephyr/random/random.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(MAIN);

#define NET_EVENT_WIFI_MASK                                                                        \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT |                        \
	 NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |                      \
	 NET_EVENT_WIFI_AP_STA_CONNECTED | NET_EVENT_WIFI_AP_STA_DISCONNECTED)

#define NET_EVENT_IPV4_MASK                                                                     \
	(NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL | NET_EVENT_IPV4_DHCP_BOUND)

/* STA Mode Configuration */
#define WIFI_SSID "IIIOT-INFOTECH_2.4G"
#define WIFI_PSK  "Automation@7076"

/* WebSocket Configuration */
#define SERVER_ADDR "192.168.1.31"
#define SERVER_PORT 8180
#define CHARGE_POINT_ID "ESP32CP001"
#define ENDPOINT "/steve/websocket/CentralSystemService/" CHARGE_POINT_ID

/* OCPP Configuration */
#define HEARTBEAT_INTERVAL_MS (300000)  // 5 minutes
#define PING_INTERVAL_MS (15000)        // 15 seconds

static struct net_if *sta_iface;
static struct wifi_connect_req_params sta_config;
static struct net_mgmt_event_callback cb;
static bool wifi_connected = false;
static bool dhcp_bound = false;
static int message_id = 1;
static int websocket_fd = -1;
static bool websocket_connected = false;

static int get_message_id(void)
{
	return message_id++;
}

static void get_current_time_string(char *buffer, size_t buffer_size)
{
	time_t now;
	struct tm *timeinfo;
	
	time(&now);
	timeinfo = gmtime(&now);
	strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
}

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			       struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		LOG_INF("WiFi connect result event received");
		wifi_connected = true;
		break;
	}
	case NET_EVENT_WIFI_DISCONNECT_RESULT: {
		LOG_INF("Disconnected from WiFi: %s", WIFI_SSID);
		wifi_connected = false;
		dhcp_bound = false;
		websocket_connected = false;
		break;
	}
	case NET_EVENT_IPV4_ADDR_ADD: {
		LOG_INF("IPv4 address added - checking if it's from DHCP");
		if (!wifi_connected) {
			LOG_INF("Assuming WiFi is connected based on IP address assignment");
			wifi_connected = true;
		}
		break;
	}
	case NET_EVENT_IPV4_DHCP_BOUND: {
		LOG_INF("DHCP bound - IP address assigned");
		dhcp_bound = true;
		if (!wifi_connected) {
			LOG_INF("WiFi connection confirmed by DHCP success");
			wifi_connected = true;
		}
		break;
	}
	default:
		LOG_DBG("Unhandled network event: 0x%08x", mgmt_event);
		break;
	}
}

static bool check_network_ready(void)
{
	int test_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (test_sock < 0) {
		LOG_DBG("Socket creation failed, network not ready");
		return false;
	}
	
	close(test_sock);
	LOG_DBG("Network appears to be ready");
	return true;
}

static int connect_to_wifi(void)
{
	if (!sta_iface) {
		LOG_ERR("STA: interface not initialized");
		return -EIO;
	}

	memset(&sta_config, 0, sizeof(sta_config));
	sta_config.ssid = (const uint8_t *)WIFI_SSID;
	sta_config.ssid_length = strlen(WIFI_SSID);
	sta_config.psk = (const uint8_t *)WIFI_PSK;
	sta_config.psk_length = strlen(WIFI_PSK);
	sta_config.security = WIFI_SECURITY_TYPE_PSK;
	sta_config.channel = WIFI_CHANNEL_ANY;
	sta_config.band = WIFI_FREQ_BAND_2_4_GHZ;

	LOG_INF("Connecting to SSID: %s", WIFI_SSID);

	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &sta_config,
			   sizeof(struct wifi_connect_req_params));
	if (ret) {
		LOG_ERR("Unable to connect to WiFi (%s), error: %d", WIFI_SSID, ret);
		return ret;
	}

	int timeout = 30;
	bool connection_detected = false;
	
	while (timeout > 0 && !connection_detected) {
		k_sleep(K_SECONDS(1));
		timeout--;
		
		if (wifi_connected || dhcp_bound || check_network_ready()) {
			connection_detected = true;
			LOG_INF("Connection detected! (wifi_connected=%d, dhcp_bound=%d)", 
				wifi_connected, dhcp_bound);
			break;
		}
		
		if (timeout % 5 == 0) {
			LOG_INF("Waiting for network connection... (%d seconds left)", timeout);
		}
	}

	if (!connection_detected) {
		LOG_ERR("Network connection timeout");
		return -ETIMEDOUT;
	}

	LOG_INF("Network connection established!");
	k_sleep(K_SECONDS(3));
	
	if (!check_network_ready()) {
		LOG_WRN("Network check failed, but proceeding anyway");
	}

	LOG_INF("WiFi connection and DHCP successful");
	return 0;
}

static int websocket_send_frame(int sock, const char *data)
{
	uint8_t frame[1024];
	int data_len = strlen(data);
	int frame_len = 0;
	
	/* WebSocket frame header */
	frame[0] = 0x81; /* FIN=1, opcode=1 (text) */
	
	if (data_len < 126) {
		frame[1] = 0x80 | data_len; /* MASK=1, payload length */
		frame_len = 2;
	} else {
		frame[1] = 0x80 | 126;
		frame[2] = (data_len >> 8) & 0xFF;
		frame[3] = data_len & 0xFF;
		frame_len = 4;
	}
	
	/* Add masking key */
	uint32_t mask_key = sys_rand32_get();
	uint8_t *mask = (uint8_t *)&mask_key;
	memcpy(&frame[frame_len], mask, 4);
	frame_len += 4;
	
	/* Add masked payload */
	for (int i = 0; i < data_len; i++) {
		frame[frame_len + i] = data[i] ^ mask[i % 4];
	}
	frame_len += data_len;

	int ret = send(sock, frame, frame_len, 0);
	if (ret < 0) {
		LOG_ERR("Failed to send WebSocket frame: %d", ret);
		return ret;
	}

	LOG_INF("Sent WebSocket frame (%d bytes)", ret);
	return 0;
}

static int websocket_handshake(int sock)
{
	char request[512];
	char response[1024];
	
	int len = snprintf(request, sizeof(request),
		"GET %s HTTP/1.1\r\n"
		"Host: %s:%d\r\n"
		"Upgrade: websocket\r\n"
		"Connection: Upgrade\r\n"
		"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
		"Sec-WebSocket-Protocol: ocpp1.6\r\n"
		"Sec-WebSocket-Version: 13\r\n"
		"\r\n",
		ENDPOINT, SERVER_ADDR, SERVER_PORT);

	LOG_INF("Sending WebSocket handshake");
	int ret = send(sock, request, len, 0);
	if (ret < 0) {
		LOG_ERR("Failed to send WebSocket handshake: %d", ret);
		return ret;
	}

	ret = recv(sock, response, sizeof(response) - 1, 0);
	if (ret < 0) {
		LOG_ERR("Failed to receive WebSocket response: %d", ret);
		return ret;
	}

	response[ret] = '\0';
	LOG_INF("WebSocket handshake response received");

	if (strstr(response, "101") && strstr(response, "Switching Protocols")) {
		LOG_INF("WebSocket handshake successful");
		return 0;
	} else {
		LOG_ERR("WebSocket handshake failed");
		return -1;
	}
}

static int connect_websocket(void)
{
	struct sockaddr_in addr;

	if (websocket_fd >= 0) {
		close(websocket_fd);
		websocket_fd = -1;
	}

	websocket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (websocket_fd < 0) {
		LOG_ERR("Failed to create WebSocket socket: %d", websocket_fd);
		return websocket_fd;
	}

	struct timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	setsockopt(websocket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	setsockopt(websocket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(SERVER_PORT);
	
	if (inet_pton(AF_INET, SERVER_ADDR, &addr.sin_addr) <= 0) {
		LOG_ERR("Invalid server address: %s", SERVER_ADDR);
		close(websocket_fd);
		websocket_fd = -1;
		return -EINVAL;
	}

	LOG_INF("Connecting to WebSocket server %s:%d...", SERVER_ADDR, SERVER_PORT);
	if (connect(websocket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("WebSocket connection failed");
		close(websocket_fd);
		websocket_fd = -1;
		return -ECONNREFUSED;
	}

	LOG_INF("TCP connection established");

	if (websocket_handshake(websocket_fd) < 0) {
		close(websocket_fd);
		websocket_fd = -1;
		return -1;
	}

	websocket_connected = true;
	LOG_INF("WebSocket connection established successfully");
	return 0;
}

static void send_boot_notification(void)
{
	char message[512];
	char timestamp[32];
	
	get_current_time_string(timestamp, sizeof(timestamp));
	
	snprintf(message, sizeof(message),
		"[2,\"%d\",\"BootNotification\","
		"{\"chargePointModel\":\"ESP32-OCPP-Client\","
		"\"chargePointVendor\":\"MyVendor\"}]",
		get_message_id());

	LOG_INF("Sending BootNotification: %s", message);
	
	if (websocket_connected && websocket_fd >= 0) {
		websocket_send_frame(websocket_fd, message);
	}
}

static void send_heartbeat(void)
{
	char message[256];
	
	snprintf(message, sizeof(message),
		"[2,\"%d\",\"Heartbeat\",{}]",
		get_message_id());

	LOG_INF("Sending Heartbeat: %s", message);
	
	if (websocket_connected && websocket_fd >= 0) {
		websocket_send_frame(websocket_fd, message);
	}
}

static void send_status_notification(const char *status)
{
	char message[512];
	char timestamp[32];
	
	get_current_time_string(timestamp, sizeof(timestamp));
	
	snprintf(message, sizeof(message),
		"[2,\"%d\",\"StatusNotification\","
		"{\"connectorId\":1,\"status\":\"%s\","
		"\"errorCode\":\"NoError\",\"timestamp\":\"%s\"}]",
		get_message_id(), status, timestamp);

	LOG_INF("Sending StatusNotification: %s", message);
	
	if (websocket_connected && websocket_fd >= 0) {
		websocket_send_frame(websocket_fd, message);
	}
}

static void handle_websocket_messages(void)
{
	if (!websocket_connected || websocket_fd < 0) {
		return;
	}

	char buffer[1024];
	int ret = recv(websocket_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
	
	if (ret > 0) {
		buffer[ret] = '\0';
		LOG_INF("Received WebSocket message (%d bytes)", ret);
		
		/* Print first few bytes as hex for debugging */
		for (int i = 0; i < ret && i < 20; i++) {
			printk("%02X ", (uint8_t)buffer[i]);
		}
		printk("\n");
		
		/* Simple WebSocket frame parsing */
		if (ret >= 6) { /* Minimum frame size */
			uint8_t *frame = (uint8_t *)buffer;
			bool fin = (frame[0] & 0x80) != 0;
			uint8_t opcode = frame[0] & 0x0F;
			bool masked = (frame[1] & 0x80) != 0;
			uint8_t payload_len = frame[1] & 0x7F;
			
			if (opcode == 1 && fin) { /* Text frame */
				int header_len = 2;
				if (payload_len == 126) {
					header_len = 4;
					payload_len = (frame[2] << 8) | frame[3];
				}
				
				if (masked) {
					header_len += 4; /* Skip mask */
				}
				
				if (ret > header_len) {
					char *payload = buffer + header_len;
					int actual_payload_len = ret - header_len;
					
					if (masked) {
						uint8_t *mask = frame + header_len - 4;
						for (int i = 0; i < actual_payload_len; i++) {
							payload[i] ^= mask[i % 4];
						}
					}
					
					payload[actual_payload_len] = '\0';
					LOG_INF("Received OCPP message: %s", payload);
				}
			}
		}
	} else if (ret == 0) {
		LOG_WRN("WebSocket connection closed by server");
		websocket_connected = false;
		close(websocket_fd);
		websocket_fd = -1;
	}
}

int main(void)
{
	LOG_INF("ESP32 OCPP Client starting...");
	k_sleep(K_SECONDS(2));

	/* Register WiFi event handler */
	net_mgmt_init_event_callback(&cb, wifi_event_handler, 
		NET_EVENT_WIFI_MASK | NET_EVENT_IPV4_MASK);
	net_mgmt_add_event_callback(&cb);

	/* Get STA interface */
	sta_iface = net_if_get_wifi_sta();
	if (!sta_iface) {
		LOG_ERR("WiFi STA interface not found");
		return -1;
	}

	/* Connect to WiFi */
	if (connect_to_wifi() != 0) {
		LOG_ERR("Failed to connect to WiFi");
		return -1;
	}

	k_sleep(K_SECONDS(2));

	/* Initialize time (simplified - just set to some base time) */
	struct timespec ts = {
		.tv_sec = 1640995200, /* 2022-01-01 00:00:00 UTC */
		.tv_nsec = 0
	};
	clock_settime(CLOCK_REALTIME, &ts);

	LOG_INF("Starting OCPP communication...");

	uint64_t last_heartbeat = 0;
	uint64_t last_ping = 0;
	bool initial_messages_sent = false;

	while (1) {
		uint64_t now = k_uptime_get();

		/* Maintain WebSocket connection */
		if (!websocket_connected) {
			LOG_INF("Attempting to connect to WebSocket...");
			if (connect_websocket() == 0) {
				/* Send initial messages after connection */
				k_sleep(K_MSEC(500));
				send_boot_notification();
				k_sleep(K_MSEC(500));
				send_status_notification("Available");
				initial_messages_sent = true;
				last_heartbeat = now;
				last_ping = now;
			} else {
				LOG_ERR("Failed to connect WebSocket, retrying in 10 seconds");
				k_sleep(K_SECONDS(10));
				continue;
			}
		}

		/* Handle incoming messages */
		handle_websocket_messages();

		/* Send heartbeat every 5 minutes */
		if (websocket_connected && (now - last_heartbeat) >= HEARTBEAT_INTERVAL_MS) {
			send_heartbeat();
			last_heartbeat = now;
		}

		/* Simple keep-alive ping every 15 seconds */
		if (websocket_connected && (now - last_ping) >= PING_INTERVAL_MS) {
			/* Send a simple ping frame */
			uint8_t ping_frame[] = {0x89, 0x80, 0x12, 0x34, 0x56, 0x78}; /* Ping with mask */
			send(websocket_fd, ping_frame, sizeof(ping_frame), 0);
			LOG_DBG("Sent WebSocket ping");
			last_ping = now;
		}

		k_sleep(K_MSEC(100)); /* Small delay to prevent tight loop */
	}

	return 0;
}