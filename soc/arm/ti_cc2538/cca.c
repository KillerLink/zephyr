// (Created by [TheM])
/*
 * The CC2538 has a special section on the end of its flash
 * which configures some of its behaviour. This section is
 * referred to as CCA (Customer Configuration Area).
 * (For additional info to the bootloader see the chart
 * at the end of the this file)
 * Subject to this configuration are:
 *
 * -) bootloader backdoor
 *  The CC2538 features a bootloader in ROM.
 *  If the CCA indicates no valid image in flash,
 *  it will start the bootloader, if it is not explicitly
 *  disabled by the CCA. If configured, the bootloader
 *  can be entered externally on reset when a CCA
 *  configurable condition (selectable logic level
 *  on selectable pin) is met.
 *  If the bootloader is not available to flash,
 *  it might be deactivated, a complete device erease
 *  can make it available again.
 *
 * -) flash image validity
 *  Indicator for the ROM bootloader whether or not
 *  a valid image is present in flash.
 *
 * -) flash lock bits
 *  The flash memory is seperated into pages of
 *  size 2048 which can be individually locked
 *  to prevent write/erase.
 *  Since the bits in flash memory can only be
 *  "set to 0", the only way of clearing a set lockbit
 *  is a complete device erease.
 *
 * -) debug access
 *  Debugging capabilities over CM3 DAP can be allowed
 *  or blocked.
 *
 *
 * Note that:
 * CCA_CFG_* denote configuration choices
 * CCA_OPT_* denote configuration options for choices
*/

#include <toolchain/gcc.h>
#include <init.h>
#include <sys_io.h>
#include <stdint.h>
#include "cca.h"

/* Settings */

//specify bootloader backdoor configuration
#define CCA_CFG_BACKDOOR_ENABLE (CCA_OPT_BACKDOOR_ENABLED)
#define CCA_CFG_BACKDOOR_LEVEL (CCA_OPT_BACKDOOR_ACTIVELOW)
#define CCA_CFG_BACKDOOR_PIN (CCA_OPT_BACKDOOR_PIN3)

// debug access configuration
#define CCA_CFG_DEBUG_ACCESS (CCA_OPT_DEBUG_ACCESS_ALLOWED)

//flash lock configuration
#define CCA_CFG_LOCKBITS_DEFAULT (CCA_OPT_LOCKBITS_UNLOCKED)

//image valid configuration
#define CCA_CFG_IMAGE_VALID (CCA_OPT_IMAGE_VALID)

/* End Settings */

extern void* _vector_start;

struct cca_t {
	uint8_t reserved[3];
	uint8_t backdoor_config;
	uint8_t image_valid[4];
	void* vector_table;
	uint8_t lockbits[32];
};

const struct cca_t __ti_cca_section cca = {
	.reserved = {
		CCA_OPT_RESERVED,
		CCA_OPT_RESERVED,
		CCA_OPT_RESERVED
		 },
	.backdoor_config = (
		0xE0 //bits [7:5] are unused and should be 0b111 //TODO: difference between 0xC0 to 0xE0?
		| ( (CCA_CFG_BACKDOOR_ENABLE & 0x01) << 4 )
		| ( (CCA_CFG_BACKDOOR_LEVEL & 0x01) << 3 )
		| ( (CCA_CFG_BACKDOOR_PIN & 0x07) << 0)
		),
	.image_valid = {
		CCA_CFG_IMAGE_VALID,
		CCA_CFG_IMAGE_VALID,
		CCA_CFG_IMAGE_VALID,
		CCA_CFG_IMAGE_VALID
		},
	.vector_table = (void*)(&_vector_start),
	.lockbits = {
		//Pages [0:31]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [32:63]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [64:95]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [96:127]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [128:159]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [160:191]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [192:223]
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		//Pages [223:254] + Debug Access
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		CCA_CFG_LOCKBITS_DEFAULT,
		( 0x00
		| ((CCA_CFG_LOCKBITS_DEFAULT&0x7F)<<0)
		| ((CCA_CFG_DEBUG_ACCESS&0x01)<<7)
		)
	}
};


/*

[Power-On-Reset]
    |
    |
    v
[CCFG parameter: Flash image valid?] --no-->--------------v
    |                                                     |
    | yes                                                 |
    v                                                     |
[CCFG parameter: BL backdoor enabled? --no-->----------v  |
    |                                                  |  |
    | yes                                              |  |
    v                                                  |  |
[CCFG parameters: PinNumber.level == Level?] --yes-->--+--v
    |                                                  |  |
    | no                                               |  |
    v                                                  |  |
(Enter FLASH application) --<--------------------------<  |
                                                          |
(Enter Bootloader, Serves commands if BL enabled) --<-----<


*/
