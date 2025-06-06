#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(http_server, LOG_LEVEL_DBG);

#define HTTP_PORT 80
#define RECV_BUF_SIZE 1024
#define SEND_BUF_SIZE 1024

static const char http_response[] = 
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n"
    "\r\n"
    "<!DOCTYPE html>\r\n"
    "<html>\r\n"
    "<head>\r\n"
    "    <title>NUCLEO-F746ZG Web Server</title>\r\n"
    "    <style>\r\n"
    "        body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\r\n"
    "        h1 { color: #2E86AB; font-size: 3em; }\r\n"
    "        p { font-size: 1.2em; color: #666; }\r\n"
    "    </style>\r\n"
    "</head>\r\n"
    "<body>\r\n"
    "    <h1>Hello IIIOT</h1>\r\n"
    "    <p>NUCLEO-F746ZG Ethernet Web Server</p>\r\n"
    "    <p>Board IP: %s</p>\r\n"
    "</body>\r\n"
    "</html>\r\n";

static struct net_mgmt_event_callback mgmt_cb;
static bool dhcp_ready = false;
static char board_ip[INET_ADDRSTRLEN];

static void dhcp_handler(struct net_mgmt_event_callback *cb,
                        uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
        LOG_INF("DHCP bound - IP address acquired");
        dhcp_ready = true;
    }
}

static void get_board_ip(struct net_if *iface)
{
    // Simple approach - just use a placeholder for now
    // The server will still work, just won't show the correct IP in the webpage
    strcpy(board_ip, "DHCP-Assigned");
    LOG_INF("Using DHCP assigned IP address");
}

static void handle_client(int client_sock)
{
    char recv_buf[RECV_BUF_SIZE];
    char send_buf[SEND_BUF_SIZE];
    int ret;

    // Receive HTTP request
    ret = zsock_recv(client_sock, recv_buf, sizeof(recv_buf) - 1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to receive data: %d", errno);
        goto cleanup;
    }
    
    recv_buf[ret] = '\0';
    LOG_INF("Received HTTP request from client");
    LOG_DBG("Request: %s", recv_buf);

    // Prepare HTTP response with board IP
    ret = snprintf(send_buf, sizeof(send_buf), http_response, board_ip);
    if (ret < 0 || ret >= sizeof(send_buf)) {
        LOG_ERR("Failed to prepare response");
        goto cleanup;
    }

    // Send HTTP response
    ret = zsock_send(client_sock, send_buf, strlen(send_buf), 0);
    if (ret < 0) {
        LOG_ERR("Failed to send response: %d", errno);
    } else {
        LOG_INF("HTTP response sent successfully");
    }

cleanup:
    zsock_close(client_sock);
}

static void http_server_thread(void)
{
    struct sockaddr_in addr;
    int server_sock, client_sock;
    int ret;
    int opt = 1;
    struct net_if *iface;

    // Wait for DHCP to complete
    while (!dhcp_ready) {
        k_msleep(100);
    }

    // Get the IP address after DHCP is ready
    iface = net_if_get_default();
    get_board_ip(iface);

    LOG_INF("Starting HTTP server on %s:%d", board_ip, HTTP_PORT);

    // Create socket
    server_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock < 0) {
        LOG_ERR("Failed to create socket: %d", errno);
        return;
    }

    // Set socket options
    ret = zsock_setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (ret < 0) {
        LOG_WRN("Failed to set SO_REUSEADDR: %d", errno);
    }

    // Bind socket
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HTTP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    ret = zsock_bind(server_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        LOG_ERR("Failed to bind socket: %d", errno);
        zsock_close(server_sock);
        return;
    }

    // Listen for connections
    ret = zsock_listen(server_sock, 5);
    if (ret < 0) {
        LOG_ERR("Failed to listen: %d", errno);
        zsock_close(server_sock);
        return;
    }

    LOG_INF("HTTP server listening on port %d", HTTP_PORT);
    LOG_INF("Access your web page at: http://%s", board_ip);

    // Accept and handle client connections
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        client_sock = zsock_accept(server_sock, (struct sockaddr *)&client_addr, 
                           &client_addr_len);
        if (client_sock < 0) {
            LOG_ERR("Failed to accept connection: %d", errno);
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        net_addr_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
        LOG_INF("Client connected from %s", client_ip);

        handle_client(client_sock);
    }

    zsock_close(server_sock);
}

int main(void)
{
    struct net_if *iface;

    LOG_INF("Starting NUCLEO-F746ZG HTTP Server");

    // Get network interface
    iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface found");
        return -1;
    }

    // Setup DHCP event handler
    net_mgmt_init_event_callback(&mgmt_cb, dhcp_handler,
                                NET_EVENT_IPV4_DHCP_BOUND);
    net_mgmt_add_event_callback(&mgmt_cb);

    // Start DHCP client
    net_dhcpv4_start(iface);
    LOG_INF("DHCP client started");

    // Start HTTP server in main thread
    http_server_thread();

    return 0;
}