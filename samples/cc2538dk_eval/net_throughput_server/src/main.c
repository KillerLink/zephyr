#include <logging/log.h>
LOG_MODULE_REGISTER(net_throughput_server, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <net/socket.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>
#include <linker/sections.h>
#include "../../common/common.h"

static int sock;
static char recv_buffer[DATA_LENGTH+1];


void panic(void) {
	NET_ERR("panic spin");
	EVAL_LED4_ON;
	while(1);
}


void init(void) {
	EVAL_LED1_OUTPUT;
	EVAL_LED2_OUTPUT;
	EVAL_LED4_OUTPUT;
	EVAL_LED1_OFF;
	EVAL_LED2_OFF;
	EVAL_LED4_OFF;
}


void recv_data() {
	int ret = 0;
	int flags = 0;
	int bytesReceived = 0;
	int errorCount = 0;
	struct sockaddr client_addr;
	socklen_t client_addr_len;
	client_addr_len = sizeof(client_addr);
	LOG_DBG("recvfrom(%d, %p, %d, %d, %p, %p);", sock, recv_buffer, DATA_LENGTH, flags, &client_addr, &client_addr_len);
	while (bytesReceived < DATA_TOTAL) {
		ret = recvfrom(sock, recv_buffer, DATA_LENGTH, flags , &client_addr, &client_addr_len);
		if (ret >= 0) {
			EVAL_LED2_TOGGLE;
//			LOG_DBG("ret=%d", ret);
			bytesReceived+=ret;
		} else {
			errorCount++;
		}
//		LOG_DBG("(progress) bytesReceived = %d", bytesReceived);
	}
	EVAL_LED1_TOGGLE;
	printk("rx: %d, err: %d\n", bytesReceived, errorCount);
}


int start_udp(void) {
	int ret = 0;
	struct sockaddr_in6 addr6 = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(SERVER_PORT),
		.sin6_addr = IN6ADDR_ANY_INIT,
	};
	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		NET_ERR("Failed to create socket (%d)", errno);
		return -errno;
	}
	errno = 0;
	ret = bind(sock, (const struct sockaddr *)&addr6, sizeof(addr6));
	if (ret < 0) {
		NET_ERR("Cannot bind socket (%d)", errno);
		return -errno;
	}
	return 0;
}


void main(void) {
	int ret;
	init();
	ret = start_udp();
	if (ret < 0) {
		NET_ERR("start_udp() failed: %d", ret);
		panic();
	}
	while (1) {
		recv_data();
	}
}

