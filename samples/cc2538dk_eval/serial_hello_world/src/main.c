#include <logging/log.h>
LOG_MODULE_REGISTER(serial_hello_world, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <stdlib.h>
#include <stdio.h>
#include <linker/sections.h>
#include "../../common/common.h"


void main(void) {
	while (1) {
		printf("Hello World!\n");
		k_sleep(1*1000);
	}
}

