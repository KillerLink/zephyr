#include <stdio.h>
#include <stdint.h>
//gcc main.c -o main -Werror -Wenum-compare -Wconversion -Wall -Wextra -pedantic

enum e1_t { E1_OK, E1_ON, E1_OFF };
enum e2_t { E2_OK, E2_RUN, E2_STOP };

enum e1_t e1 = E1_ON;
enum e2_t e2 = E2_RUN;
enum e1_t e1f = E2_RUN;
enum e2_t e2f = E1_ON;

struct kv_pair_t {
	union {
		uint32_t key;
		uint32_t* address;
	};
	uint32_t val;
};

static const struct kv_pair_t kv_init[] = {
    (struct kv_pair_t) {.key = (uint32_t)0x123, .val=0x456},
    (struct kv_pair_t) {.key = 0x678, .val=0x9ab},
    (struct kv_pair_t) {.key = 0x12123434, .val=01010202}
};


struct kv_output_power_t {
	int16_t dbm;
	uint8_t hw_cfg;
};

#define ABS_DELTA(a,b) (a>b?(a-b):(b-a))

static const struct kv_output_power_t output_power_lut[] = {
	{.dbm = 7, .hw_cfg = 0xFF },
	{.dbm = 5, .hw_cfg = 0xED },
	{.dbm = 3, .hw_cfg = 0xD5 },
	{.dbm = 1, .hw_cfg = 0xC5 },
	{.dbm = 0, .hw_cfg = 0xB6 },
	{.dbm = -1, .hw_cfg = 0xB0 },
	{.dbm = -3, .hw_cfg = 0xA1 },
	{.dbm = -5, .hw_cfg = 0x91 },
	{.dbm = -7, .hw_cfg = 0x88 },
	{.dbm = -9, .hw_cfg = 0x72 },
	{.dbm = -11, .hw_cfg = 0x62 },
	{.dbm = -13, .hw_cfg = 0x58 },
	{.dbm = -15, .hw_cfg = 0x42 },
	{.dbm = -24, .hw_cfg = 0x00 }
};

void output_power_best_match(int16_t dbm) {
	int opt_index = -1;
	int opt_delta = 0;

	for (int i=0; i<sizeof(output_power_lut)/sizeof(output_power_lut[0]); i++) {
		int delta = ABS_DELTA(dbm,output_power_lut[i].dbm);
		if ((opt_index == -1) || (delta<opt_delta) ) {
			opt_index = i;
			opt_delta = delta;
		}
	}
	printf("closest to desired %d dbm is %d with config 0x%2x\n",
		dbm,
		output_power_lut[opt_index].dbm,
		output_power_lut[opt_index].hw_cfg
	);
}




int main( int argc, char** argv) {
	for (int i=0; i<sizeof(kv_init)/sizeof(kv_init[0]); i++) {
		printf("%08x : %08x (%p)\n", kv_init[i].key, kv_init[i].val, kv_init[i].address);
	}
	printf("============================\n");
	output_power_best_match(-80);
	output_power_best_match(-24);
	output_power_best_match(-20);
	output_power_best_match(-7);
	output_power_best_match(-6);
	output_power_best_match(-5);
	output_power_best_match(0);
	output_power_best_match(2);
	output_power_best_match(3);
	output_power_best_match(4);
	output_power_best_match(5);
	output_power_best_match(8);
	output_power_best_match(16);

	printf("%d %d %d %d\n", e1, e2, e1f, e2f);
}
