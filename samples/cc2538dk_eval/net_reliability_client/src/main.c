#include <logging/log.h>
LOG_MODULE_REGISTER(net_reliability_client, LOG_LEVEL_DBG);

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

static const char lorem_ipsum[] =
"Lorem ipsum dolor sit amet, consetetur sadipscing elitr,"
" sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua.\n"
"At vero eos et accusam et justo duo dolores et ea rebum.\n"
"Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.\n"
"Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor"
" invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua.\n"
"At vero eos et accusam et justo duo dolores et ea rebum.\n"
"Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.\n"
"Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor"
" invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua.\n"
"At vero eos et accusam et justo duo dolores et ea rebum."
"Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.\n"
"\n"
"Duis autem vel eum iriure dolor in hendrerit in vulputate velit esse molestie consequat,"
" vel illum dolore eu feugiat nulla facilisis at vero eros et accumsan et iusto odio dignissim"
" qui blandit praesent luptatum zzril delenit augue duis dolore te feugait nulla facilisi.\n"
"Lorem ipsum dolor sit amet, consectetuer adipiscing elit, sed diam nonummy nibh euismod"
" tincidunt ut laoreet dolore magna aliquam erat volutpat.\n"
"\n"
"Ut wisi enim ad minim veniam, quis nostrud exerci tation ullamcorper suscipit lobortis"
" nisl ut aliquip ex ea commodo consequat.\n"
"Duis autem vel eum iriure dolor in hendrerit in vulputate velit esse molestie consequat,"
" vel illum dolore eu feugiat nulla facilisis at vero eros et accumsan et iusto odio"
" dignissim qui blandit praesent luptatum zzril delenit augue duis dolore te feugait nulla facilisi.\n"
"\n"
"Nam liber tempor cum soluta nobis eleifend option congue nihil imperdiet doming"
" id quod mazim placerat facer possim assum.\n"
"Lorem ipsum dolor sit amet, consectetuer adipiscing elit,"
" sed diam nonummy nibh euismod tincidunt ut laoreet dolore magna aliquam erat volutpat.\n"
;


static int sock;


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


void send_data() {
	int ret = 0;
	int bytesSent = 0;
	int errorCount = 0;
	printk("send(%d, %p, %d, 0);\n", sock, lorem_ipsum, ret);
	while (bytesSent < DATA_TOTAL) {
		ret = send(sock, lorem_ipsum, DATA_LENGTH, 0);
		if (ret < 0) {
			errorCount++;
		} else {
			EVAL_LED2_TOGGLE;
			bytesSent+=ret;
		}
	}
	EVAL_LED1_TOGGLE;
	printk("bytesSent = %d, errorCount = %d\n", bytesSent, errorCount);
}


int start_udp(void) {
	int ret = 0;
	struct sockaddr_in6 addr6;
	addr6.sin6_family = AF_INET6;
	addr6.sin6_port = htons(SERVER_PORT);
	inet_pton(AF_INET6, IPV6_SERVER_ADDR, &addr6.sin6_addr);
	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		NET_ERR("Failed to create socket: %d", errno);
		return -errno;
	}
	ret = connect(sock, (const struct sockaddr *)&addr6, sizeof(addr6));
	if (ret < 0) {
		NET_ERR("Cannot connect to UDP remote: %d", errno);
		return -errno;
	}
	return 0;
}


void main(void) {
	int ret = 0;
	init();
	ret = start_udp();
	if (ret < 0) {
		NET_ERR("start_udp() failed: %d", ret);
		panic();
	}
	send_data();
	while (1) {
		//nothing, end
	}
}

