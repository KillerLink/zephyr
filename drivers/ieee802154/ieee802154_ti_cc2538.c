// (Created by [TheM])
/* ieee802154_cc2538.c - TI CC2538 driver */

/*
======== NOTES ========
STXON()
	aborts running tx and forces calibration
STXONCCA()
	aborts running tx and rx and forces calibration

192us after STXON[CCA]() from rx to tx
192us from tx to rx

TX_DONE_FRM after successfull transmission
RSSI_VALID can tell if CCA is valid
CCA is updated 4 system clock cycles after RSSI_VALID

SSAMPLECCA and ISTXONCCA update SAMPLED_CCA
Every new RSSI reading updates CCA
*/

#define LOG_LEVEL 3
#define LOG_MODULE_NAME ti_cc2538_rf
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <atomic.h>
#include <kernel.h>
#include <arch/cpu.h>
//#include <board.h>
#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
#include <misc/byteorder.h>
#include <string.h>
#include <net/ieee802154_radio.h>
#include "ieee802154_ti_cc2538.h"


// ================ ================ ================ ================
// Forward Declarations
// ================ ================ ================ ================

static int ti_cc2538_rf_init(struct device *dev);
static enum ieee802154_hw_caps ti_cc2538_rf_get_capabilities(struct device *dev);
static int ti_cc2538_rf_cca(struct device *dev);
static int ti_cc2538_rf_set_channel(struct device *dev, u16_t channel);
static int ti_cc2538_rf_set_txpower(struct device *dev, s16_t dbm);
static int ti_cc2538_rf_start(struct device *dev);
static int ti_cc2538_rf_stop(struct device *dev);

static int ti_cc2538_rf_init(struct device *dev);
static void ti_cc2538_rf_iface_init(struct net_if *iface);
static enum ieee802154_hw_caps ti_cc2538_rf_get_capabilities(struct device *dev);
static int ti_cc2538_rf_start(struct device *dev);
static int ti_cc2538_rf_stop(struct device *dev);
static int ti_cc2538_rf_set_channel(struct device *dev, u16_t channel);
static int ti_cc2538_rf_set_txpower(struct device *dev, s16_t dbm);
static int ti_cc2538_rf_cca(struct device *dev);
static int ti_cc2538_rf_set_filter(
	struct device *dev,
	bool set,
	enum ieee802154_filter_type type,
	const struct ieee802154_filter *filter
);
static int ti_cc2538_rf_tx(
	struct device *dev,
	struct net_pkt *pkt,
	struct net_buf *frag
);


static void ti_cc2538_rf_rx(int arg1, int arg2, int arg3);
static void rf_handler(void* arg);
static void rferr_handler(void* arg);


// ================ ================ ================ ================
// Data Structures
// ================ ================ ================ ================
//m:dstruct

struct kv_register_init_t {
	union {
		uint32_t key;
		uint32_t* address;
	};
	uint32_t val;
};

struct kv_power_config_t {
	int16_t dbm;
	uint8_t hw_cfg;
};

// ================ ================ ================ ================
// Global Data
// ================ ================ ================ ================
//m:gdata

static const struct kv_register_init_t kv_init_table[] = {
	// ========= NOTE: recommendations made by the cc2538 datasheet ========
	{.key = RFCORE_XREG_AGCCTRL1, .val=0x15}, //adjusts automatic gain control target
	{.key = RFCORE_XREG_FSCTRL, .val=0x5A}, //set tx antialiasing filter to appropriate bandwidth
	{.key = ANA_REGS_BASE+ANA_REGS_O_IVCTRL, .val=0x0B}, //control bias currents
	{.key = RFCORE_XREG_FSCAL1, .val=0x01}, //tune frequency calibration
	// ======== NOTE: custom configuration ========
	{.key = RFCORE_XREG_FRMFILT0, .val=0
		|(0x3<<RFCORE_XREG_FRMFILT0_MAX_FRAME_VERSION_S) //all frame versions
		|(1<<RFCORE_XREG_FRMFILT0_PAN_COORDINATOR_S) //a pan coordinator //TODO: without?
		|(1<<RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN_S) //frame filtering enabled
	},
	{.key = RFCORE_XREG_FRMFILT1, .val=0
		|(1<<RFCORE_XREG_FRMFILT1_ACCEPT_FT_3_MAC_CMD_S) //accept command frames
		|(1<<RFCORE_XREG_FRMFILT1_ACCEPT_FT_2_ACK_S) //accept ack frames
		|(1<<RFCORE_XREG_FRMFILT1_ACCEPT_FT_1_DATA_S) //accept data frames
		|(1<<RFCORE_XREG_FRMFILT1_ACCEPT_FT_0_BEACON_S) //accept beacon frames
		|(0x0<<RFCORE_XREG_FRMFILT1_MODIFY_FT_FILTER_S) //do not modify frame type
	},
	{.key = RFCORE_XREG_SRCMATCH, .val=0
		|(0<<RFCORE_XREG_SRCMATCH_PEND_DATAREQ_ONLY_S) //(only relevant if autopend)
		|(0<<RFCORE_XREG_SRCMATCH_AUTOPEND_S) //no autopend
		|(0<<RFCORE_XREG_SRCMATCH_SRC_MATCH_EN_S) //no source address matching
	},
	{.key = RFCORE_XREG_SRCSHORTEN0, .val=0}, //no autopend for shortadresses 0 to 7
	{.key = RFCORE_XREG_SRCSHORTEN1, .val=0}, //no autopend for shortadresses 8 to 15
	{.key = RFCORE_XREG_SRCSHORTEN2, .val=0}, //no autopend for shortadresses 16 to 23
	{.key = RFCORE_XREG_SRCEXTEN0, .val=0}, //no autopend for extadresses 0 to 7
	{.key = RFCORE_XREG_SRCEXTEN1, .val=0}, //no autopend for extadresses 8 to 15
	{.key = RFCORE_XREG_SRCEXTEN2, .val=0}, //no autopend for extadresses 16 to 23
	{.key = RFCORE_XREG_FRMCTRL0, .val=0
		|(0<<RFCORE_XREG_FRMCTRL0_APPEND_DATA_MODE_S) //append rssi,crc,correlation
		|(1<<RFCORE_XREG_FRMCTRL0_AUTOCRC_S) //enable automatic crc
		|(1<<RFCORE_XREG_FRMCTRL0_AUTOACK_S) //enable autoack
		|(0<<RFCORE_XREG_FRMCTRL0_ENERGY_SCAN_S) //use most recent signal strength
		|(0x0<<RFCORE_XREG_FRMCTRL0_RX_MODE_S) //normal rx mode with fifo
		|(0x0<<RFCORE_XREG_FRMCTRL0_TX_MODE_S) //normal tx mode with fifo
	},
	{.key = RFCORE_XREG_FRMCTRL1, .val=0
		|(0<<RFCORE_XREG_FRMCTRL1_PENDING_OR_S) //state machine and filtering controls data pending bit
		|(0<<RFCORE_XREG_FRMCTRL1_IGNORE_TX_UNDERF_S) //does not ignore tx underflows
		|(0<<RFCORE_XREG_FRMCTRL1_SET_RXENMASK_ON_TX_S) //do not enable rx after tx
	},
	//{.key = RFCORE_XREG_RXENABLE, .val=0}, //unmodified
	//{.key = RFCORE_XREG_RXMASKSET, .val=0}, //unmodified
	//{.key = RFCORE_XREG_RXMASKCLR, .val=0}, //unmodified
	//{.key = RFCORE_XREG_FREQTUNE, .val=0}, //unmodified
	//{.key = RFCORE_XREG_FREQCTRL, .val=0}, //unmodified
	//{.key = RFCORE_XREG_TXPOWER, .val=0}, //unmodified
	//{.key = RFCORE_XREG_TXCTRL, .val=0}, //unmodified
	//{.key = RFCORE_XREG_FSMSTAT0, .val=0}, //read-only
	//{.key = RFCORE_XREG_FSMSTAT1, .val=0}, //read-only
	//{.key = RFCORE_XREG_FIFOPCTRL, .val=0}, //unmodified
	{.key = RFCORE_XREG_FSMCTRL, .val=0
		|(0<<RFCORE_XREG_FSMCTRL_SLOTTED_ACK_S) //ack frames are sent 12-symbol-periods after received frame
		|(1<<RFCORE_XREG_FSMCTRL_RX2RX_TIME_OFF_S) //use 12-symbol-period timeout after frame reception
	},
	{.key = RFCORE_XREG_CCACTRL0, .val=0
		|(0x68<<RFCORE_XREG_CCACTRL0_CCA_THR_S)	//threshold value indicating clear channel, -81dBm
												//must not be lower than (CCA_HYST-128)!
	},
	{.key = RFCORE_XREG_CCACTRL1, .val=0
		|(0x3<<RFCORE_XREG_CCACTRL1_CCA_MODE_S) //cca=1 IF RSSI<CCA_THR-CCA_HYST && !recieving_frame
		|(0x2<<RFCORE_XREG_CCACTRL1_CCA_HYST_S) //cca hysteresis, 2dBm
	},
	//{.key = RFCORE_XREG_RSSI, .val=0}, //read-only
	//{.key = RFCORE_XREG_RSSISTAT, .val=0}, //read-only
	//{.key = RFCORE_XREG_RXFIRST, .val=0}, //read-only
	//{.key = RFCORE_XREG_RXFIFOCNT, .val=0}, //read-only
	//{.key = RFCORE_XREG_TXFIFOCNT, .val=0}, //read-only
	//{.key = RFCORE_XREG_RXFIRST_PTR, .val=0}, //read-only
	//{.key = RFCORE_XREG_RXLAST_PTR, .val=0}, //read-only
	//{.key = RFCORE_XREG_RSSISTAT, .val=0}, //read-only
	//{.key = RFCORE_XREG_TXFIRST_PTR, .val=0}, //read-only
	//{.key = RFCORE_XREG_TXLAST_PTR, .val=0}, //read-only
	{.key = RFCORE_XREG_RFIRQM0, .val=0
		|(0<<RFIRQM0_RXMASKZERO_S) //
		|(0<<RFIRQM0_RXPKTDONE_S) //
		|(0<<RFIRQM0_FRAME_ACCEPTED_S) //
		|(0<<RFIRQM0_SRC_MATCH_FOUND_S) //
		|(0<<RFIRQM0_SRC_MATCH_DONE_S) //
		|(0<<RFIRQM0_FIFOP_S) //
		|(0<<RFIRQM0_SFD_S) //
	},
	{.key = RFCORE_XREG_RFIRQM1, .val=0
		|(0<<RFIRQM1_CSP_WAIT_S) //
		|(0<<RFIRQM1_CSP_STOP_S) //
		|(0<<RFIRQM1_CSP_MANINT_S) //
		|(0<<RFIRQM1_RF_IDLE_S) //
		|(0<<RFIRQM1_TXDONE_S) //
		|(0<<RFIRQM1_TXACKDONE_S) //
	},
	{.key = RFCORE_XREG_RFERRM, .val=0
		|(0<<RFERRM_STROBE_ERR_S) //
		|(0<<RFERRM_TXOVERF_S) //
		|(0<<RFERRM_TXUNDERF_S) //
		|(0<<RFERRM_RXOVERF_S) //
		|(0<<RFERRM_RXUNDERF_S) //
		|(0<<RFERRM_RXABO_S) //
		|(0<<RFERRM_NLOCK_S) //
	},
	//{.key = RFCORE_XREG_RFRND, .val=0}, //read-only
	{.key = RFCORE_XREG_MDMCTRL0, .val=0
		|(0x2<<RFCORE_XREG_MDMCTRL0_DEM_NUM_ZEROS_S) //expect 2 leading zeros
		|(0<<RFCORE_XREG_MDMCTRL0_DEMOD_AVG_MODE_S) //frequency offset calibration per frame
		|(0x2<<RFCORE_XREG_MDMCTRL0_PREAMBLE_LENGTH_S) //send 2 leading zeros
		|(1<<RFCORE_XREG_MDMCTRL0_TX_FILTER_S) //perform extra filtering to reduce out-of-band emission
	},
	{.key = RFCORE_XREG_MDMCTRL1, .val=0
		|(0<<RFCORE_XREG_MDMCTRL1_CORR_THR_SFD_S) //correlation of only one of the leading zeros must be above threshold
		|(0x14<<RFCORE_XREG_MDMCTRL1_CORR_THR_S) //correlation threshold, default value
	},
	//{.key = RFCORE_XREG_FREQEST, .val=0}, //read-only
	//{.key = RFCORE_XREG_RXCTRL, .val=0}, //unused
	//{.key = RFCORE_XREG_FSCTRL, .val=0}, //unused
	//{.key = RFCORE_XREG_FSCAL0, .val=0}, //unused
	//{.key = RFCORE_XREG_FSCAL1, .val=0}, //unused
	//{.key = RFCORE_XREG_FSCAL2, .val=0}, //unused
	//{.key = RFCORE_XREG_FSCAL3, .val=0}, //unused
	//{.key = RFCORE_XREG_AGCCTRL0, .val=0}, //unused
	//{.key = RFCORE_XREG_AGCCTRL1, .val=0}, //unused, already configured
	//{.key = RFCORE_XREG_AGCCTRL2, .val=0}, //unused
	//{.key = RFCORE_XREG_AGCCTRL3, .val=0}, //unused
	//{.key = RFCORE_XREG_ADCTEST0, .val=0}, //unused
	//{.key = RFCORE_XREG_ADCTEST1, .val=0}, //unused
	//{.key = RFCORE_XREG_ADCTEST1, .val=0}, //unused
	//{.key = RFCORE_XREG_MDMTEST0, .val=0}, //unused
	//{.key = RFCORE_XREG_MDMTEST1, .val=0}, //unused
	//{.key = RFCORE_XREG_DACTEST0, .val=0}, //unused
	//{.key = RFCORE_XREG_DACTEST1, .val=0}, //unused
	//{.key = RFCORE_XREG_DACTEST2, .val=0}, //unused
	//{.key = RFCORE_XREG_ATEST, .val=0}, //unused
	//{.key = RFCORE_XREG_PTEST0, .val=0}, //unused
	//{.key = RFCORE_XREG_PTEST1, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPPROG__x, .val=0}, //unused for x from 0 to 23
	//{.key = RFCORE_XREG_CSPCTRL, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPSTAT, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPX, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPY, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPZ, .val=0}, //unused
	//{.key = RFCORE_XREG_CSPT, .val=0}, //unused
	//{.key = RFCORE_XREG_RFC_OBS_CTRL0, .val=0}, //unused
	//{.key = RFCORE_XREG_RFC_OBS_CTRL1, .val=0}, //unused
	//{.key = RFCORE_XREG_RFC_OBS_CTRL2, .val=0}, //unused
	//{.key = RFCORE_XREG_TXFILTCFG, .val=0}, //unused
};

static const struct kv_power_config_t power_config_lut[] = {
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


// ================ ================ ================ ================
// Debug
// ================ ================ ================ ================
//m:debug

static void hal_dbg_RFERRF(void) {
	LOG_ERR("RFCORE_SFR_RFERRF: 0x%08x (%s,%s,%s,%s,%s,%s,%s)", HWREG(RFCORE_SFR_RFERRF),
		(HWREG(RFCORE_SFR_RFERRF)&(1<<0))?"NOLOCK":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<1))?"RXABO":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<2))?"RXOVERF":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<3))?"RXUNDERF":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<4))?"TXOVERF":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<5))?"TXUNDERF":" ",
		(HWREG(RFCORE_SFR_RFERRF)&(1<<6))?"STROBEERR":" "
		);
}
static void hal_dbg_RFIRQF0(void) {
	LOG_ERR("RFCORE_SFR_RFIRQF0: 0x%08x (%s,%s,%s,%s,%s,%s,%s,%s)", HWREG(RFCORE_SFR_RFIRQF0),
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<0))?"UNUSED":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<1))?"SFD":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<2))?"FIFOP":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<3))?"SRC_MATCH_DONE":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<4))?"SRC_MATCH_FOUND":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<5))?"FRAME_ACCEPTED":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<6))?"RXPKTDONE":" ",
		(HWREG(RFCORE_SFR_RFIRQF0)&(1<<7))?"RXMASKZERO":" "
	);
}

static void hal_dbg_RFIRQF1(void) {
	LOG_ERR("RFCORE_SFR_RFIRQF1: 0x%08x (%s,%s,%s,%s,%s,%s)", HWREG(RFCORE_SFR_RFIRQF1),
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<0))?"TXACKDONE":" ",
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<1))?"TXDONE":" ",
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<2))?"RFIDLE":" ",
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<3))?"CSP_MANINT":" ",
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<4))?"CSP_STOP":" ",
		(HWREG(RFCORE_SFR_RFIRQF1)&(1<<5))?"CSP_WAIT":" "
	);
}

static void hal_dbg_hwinfo(void) {
	LOG_DBG("DIECFG1 0x%8x", (unsigned int)(HWREG(FLASH_CTRL_DIECFG1)));
	LOG_DBG("DIECFG2 0x%8x", (unsigned int)(HWREG(FLASH_CTRL_DIECFG2)));
	LOG_INF("RFCORE HW Version: %u.%u", (HWREG(FLASH_CTRL_DIECFG2) >> 12) & 0x0F, (HWREG(FLASH_CTRL_DIECFG2) >> 8) & 0x0F );
}

static inline void hal_dbg_irqinfo(void) {
	hal_dbg_RFERRF();
	hal_dbg_RFIRQF0();
	hal_dbg_RFIRQF1();
}

// ================ ================ ================ ================
// HAL functions
// ================ ================ ================ ================
//m:hal


// ================ ================
// ================ Command Strobes
// ================ ================
//m:csp

#define st(x)			do { x } while (__LINE__ == -1)
#define ISRXON()		st(HWREG(RFCORE_SFR_RFST) = 0x000000E3;) //enable rx and calibrate frequency synthesizer
#define ISTXON()		st(HWREG(RFCORE_SFR_RFST) = 0x000000E9;) //enable tx after calibration
#define ISTXONCCA()		st(HWREG(RFCORE_SFR_RFST) = 0x000000EA;) //enable tx after calibration, IF Channel Clear
#define ISRFOFF()		st(HWREG(RFCORE_SFR_RFST) = 0x000000EF;) //disable rx, tx & frequency synthesizer
#define ISFLUSHRX()		st(HWREG(RFCORE_SFR_RFST) = 0x000000ED;) //flush rx buffer and reset demodulator
#define ISFLUSHTX()		st(HWREG(RFCORE_SFR_RFST) = 0x000000EE;) //flush tx buffer
#define ISSAMPLECCA()	st(HWREG(RFCORE_SFR_RFST) = 0x000000EB;) //issue cca, register in xreg


// ================ ================
// ================ Signal Flags
// ================ ================
//m:sigf

static inline bool hal_sigf_fifo(void) {
	return  (bool)(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_FIFO_M);
}

static inline bool hal_sigf_fifop(void) {
	return  (bool)(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_FIFOP_M);
}


// ================ ================
// ================ IRQ Flags
// ================ ================
//m:irqf

static inline bool hal_irqf_get_txdone(void) {
	return ( HWREG(RFCORE_SFR_RFIRQF1) & RFCORE_SFR_RFIRQF1_TXDONE_M );
}
static inline void hal_irqf_clear_txdone(void) {
	HWREG(RFCORE_SFR_RFIRQF1) &= ~RFCORE_SFR_RFIRQF1_TXDONE_M;
}

static inline uint32_t hal_irqf0(void) {
	return HWREG(RFCORE_SFR_RFIRQF0);
}
static inline void hal_irqf0_clear(void) {
	HWREG(RFCORE_SFR_RFIRQF0) = 0;
}
static inline void hal_irqf0_clear_enabled(void) {
	HWREG(RFCORE_SFR_RFIRQF0) &= ~(HWREG(RFCORE_XREG_RFIRQM0));
}

static inline uint32_t hal_irqf1(void) {
	return HWREG(RFCORE_SFR_RFIRQF1);
}
static inline void hal_irqf1_clear(void) {
	HWREG(RFCORE_SFR_RFIRQF1) = 0;
}
static inline void hal_irqf1_clear_enabled(void) {
	HWREG(RFCORE_SFR_RFIRQF1) &= ~(HWREG(RFCORE_XREG_RFIRQM1));
}


// ================ ================
// ================ Error Flags
// ================ ================
//m:errf

static inline bool hal_errf_get_rx_overflow(void) {
	return ( HWREG(RFCORE_SFR_RFERRF) & RFCORE_SFR_RFERRF_RXOVERF_M );
}
static inline void hal_errf_clear_rx_overflow(void) {
	HWREG(RFCORE_SFR_RFERRF) &= ~ RFCORE_SFR_RFERRF_RXOVERF_M;
}


static inline bool hal_errf_get_rx_underflow(void) {
	return ( HWREG(RFCORE_SFR_RFERRF) & RFCORE_SFR_RFERRF_RXUNDERF_M );
}
static inline void hal_errf_clear_rx_underflow(void) {
	HWREG(RFCORE_SFR_RFERRF) &= ~ RFCORE_SFR_RFERRF_RXUNDERF_M;
}


static inline bool hal_errf_get_tx_overflow(void) {
	return ( HWREG(RFCORE_SFR_RFERRF) & RFCORE_SFR_RFERRF_TXOVERF_M );
}
static inline void hal_errf_clear_tx_overflow(void) {
	HWREG(RFCORE_SFR_RFERRF) &= ~RFCORE_SFR_RFERRF_TXOVERF_M;
}


static inline bool hal_errf_get_tx_underflow(void) {
	return ( HWREG(RFCORE_SFR_RFERRF) & RFCORE_SFR_RFERRF_TXUNDERF_M );
}
static inline void hal_errf_clear_tx_underflow(void) {
	HWREG(RFCORE_SFR_RFERRF) &= ~RFCORE_SFR_RFERRF_TXUNDERF_M;
}


static inline uint32_t hal_errf(void) {
	return HWREG(RFCORE_SFR_RFERRF);
}
static inline void hal_errf_clear(void) {
	HWREG(RFCORE_SFR_RFERRF) = 0;
}
static inline void hal_errf_clear_enabled(void) {
	HWREG(RFCORE_SFR_RFERRF) &= ~(HWREG(RFCORE_XREG_RFERRM));
}


// ================ ================
// ================ Configuration
// ================ ================
//m:cfg

static inline void hal_cfg_set_fifop_threshold(uint8_t th) {
	NET_ASSERT(th < 128);
	HWREG(RFCORE_XREG_FIFOPCTRL) = (th & RFCORE_XREG_FIFOPCTRL_FIFOP_THR_M);
}

static inline void hal_cfg_set_pan_id(uint16_t pan_id) {
	LOG_DBG("set pan id: 0x%x", pan_id);
	pan_id = sys_le16_to_cpu(pan_id);
	HWREG(RFCORE_FFSM_PAN_ID0) = LO_UINT16(pan_id);
	HWREG(RFCORE_FFSM_PAN_ID1) = HI_UINT16(pan_id);
}

static inline void hal_cfg_set_output_power(uint8_t cfg) {
	LOG_DBG("set output power to: 0x%02x", cfg);
	HWREG(RFCORE_XREG_TXPOWER) = cfg;
}

static inline void hal_cfg_set_channel(uint16_t channel) {
	NET_ASSERT(channel>=MIN_CHANNEL);
	NET_ASSERT(channel<=MAX_CHANNEL);
	LOG_DBG("set channel to: %d", channel);
	HWREG(RFCORE_XREG_FREQCTRL) = (MIN_CHANNEL + (channel - MIN_CHANNEL) * CHANNEL_SPACING);
}

static inline void hal_cfg_set_short_addr(uint16_t short_addr) {
	short_addr = sys_le16_to_cpu(short_addr);
	LOG_DBG("set short address: 0x%04x", short_addr);
	HWREG(RFCORE_FFSM_SHORT_ADDR0) = LO_UINT16(short_addr);
	HWREG(RFCORE_FFSM_SHORT_ADDR1) = HI_UINT16(short_addr);
}

static inline void hal_cfg_set_ieee_addr(const uint8_t ieee_addr[8]) {
	LOG_DBG("set IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		    ieee_addr[0], ieee_addr[1], ieee_addr[2], ieee_addr[3],
		    ieee_addr[4], ieee_addr[5], ieee_addr[6], ieee_addr[7]);

	HWREG(RFCORE_FFSM_EXT_ADDR0) = ieee_addr[0];
	HWREG(RFCORE_FFSM_EXT_ADDR1) = ieee_addr[1];
	HWREG(RFCORE_FFSM_EXT_ADDR2) = ieee_addr[2];
	HWREG(RFCORE_FFSM_EXT_ADDR3) = ieee_addr[3];
	HWREG(RFCORE_FFSM_EXT_ADDR4) = ieee_addr[4];
	HWREG(RFCORE_FFSM_EXT_ADDR5) = ieee_addr[5];
	HWREG(RFCORE_FFSM_EXT_ADDR6) = ieee_addr[6];
	HWREG(RFCORE_FFSM_EXT_ADDR7) = ieee_addr[7];
}


// ================ ================
// ================ Data Access
// ================ ================
//m:dacc

static inline size_t hal_fifo_rx_count(void) {
	return (size_t) (HWREG(RFCORE_XREG_RXFIFOCNT)&RFCORE_XREG_RXFIFOCNT_RXFIFOCNT_M);
}

static inline size_t hal_fifo_tx_count(void) {
	return (size_t) (HWREG(RFCORE_XREG_TXFIFOCNT)&RFCORE_XREG_TXFIFOCNT_TXFIFOCNT_M);
}

static inline uint8_t hal_fifo_read_byte(void) {
	return (uint8_t)(HWREG(RFCORE_SFR_RFDATA));
}

static inline uint8_t hal_fifo_peek_byte(void) {
	return (uint8_t)(HWREG(RFCORE_XREG_RXFIRST)&0xFF);
}

static inline void hal_fifo_write_byte(uint8_t data) {
	HWREG(RFCORE_SFR_RFDATA) = data;
}

static inline void hal_fifo_read(uint8_t* dst, size_t cnt) {
	NET_ASSERT(cnt <= 128);
	for (size_t i = 0; i<cnt; i++) {
		dst[i] = (uint8_t)(HWREG(RFCORE_SFR_RFDATA));
	}
}

static inline void hal_fifo_write(uint8_t* src, size_t cnt) {
	NET_ASSERT(cnt <= 128);
	for (size_t i = 0; i<cnt; i++) {
		HWREG(RFCORE_SFR_RFDATA) = src[i];
	}
}


// ================ ================
// ================ Hardware Interaction
// ================ ================
//m:hwi

static inline void hal_rf_enable(void) {
	//soft enable, does not abort current transmissions
	HWREG(RFCORE_XREG_RXMASKSET) = 0x01;
}

static inline void hal_rf_disable(void) {
	//soft disable, does not abort current transmissions
	HWREG(RFCORE_XREG_RXMASKCLR) = 0x01;
}

static inline bool hal_is_rf_enabled(void) {
	return ( HWREG(RFCORE_XREG_RXENABLE) & RFCORE_XREG_RXENABLE_RXENMASK_M );
}

static inline bool hal_is_pll_stable(void) {
	return ( HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_LOCK_STATUS_M );
}

static inline bool hal_is_rssi_valid(void) {
	return ( HWREG(RFCORE_XREG_RSSISTAT) & RFCORE_XREG_RSSISTAT_RSSI_VALID_M );
}

static inline bool hal_cca(void) {
	return ( HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_CCA_M );
}


// ================ ================ ================ ================
// Driver API Declaration
// ================ ================ ================ ================
//m:api

static struct ti_cc2538_rf_context_t ti_cc2538_rf_context_data;

static struct ieee802154_radio_api ti_cc2538_rf_radio_api = {
	.iface_api.init		= ti_cc2538_rf_iface_init,
	.get_capabilities	= ti_cc2538_rf_get_capabilities,
	.cca				= ti_cc2538_rf_cca,
	.set_channel		= ti_cc2538_rf_set_channel,
	.filter				= ti_cc2538_rf_set_filter,
	.set_txpower		= ti_cc2538_rf_set_txpower,
	.start				= ti_cc2538_rf_start,
	.stop				= ti_cc2538_rf_stop,
	.tx					= ti_cc2538_rf_tx,
};


NET_DEVICE_INIT(
	ti_cc2538_rf_radio,
	CONFIG_IEEE802154_TI_CC2538_RF_DRV_NAME,
	ti_cc2538_rf_init,
	&ti_cc2538_rf_context_data,
	NULL,
	CONFIG_IEEE802154_TI_CC2538_RF_INIT_PRIO,
	&ti_cc2538_rf_radio_api,
	IEEE802154_L2,
	NET_L2_GET_CTX_TYPE(IEEE802154_L2),
	125
);

NET_STACK_INFO_ADDR(
	RX,
	ti_cc2538_rf_radio,
	CONFIG_IEEE802154_TI_CC2538_RF_RX_STACK_SIZE,
	CONFIG_IEEE802154_TI_CC2538_RF_RX_STACK_SIZE,
	ti_cc2538_rf_context_data.ti_cc2538_rf_rx_stack,
	0
);


// ================ ================ ================ ================
// Helper Functions
// ================ ================ ================ ================
//m:help

static void wait_rssi_valid(void) {
	if (!hal_is_rf_enabled()) {
		LOG_ERR("rssi can not become valid when rf disabled");
		return;
	}
	while (!hal_is_rssi_valid()) {
		LOG_DBG("waiting for rssi to become valid");
		k_sleep(1);
		//nothing
	}
}

static bool wait_pll_stable(uint32_t max_us) {
	uint32_t i=0;
	if (!hal_is_rf_enabled()) {
		LOG_ERR("pll can not become stable when rf disabled");
		return false;
	}
	if (max_us == 0) {
		LOG_WRN("possibly waiting forever until pll is stable");
		while (!hal_is_pll_stable()) {
			LOG_DBG("waiting for pll to become stable");
			k_sleep(1);
			//nothing
		}
		LOG_WRN("pll eventually became stable");
	}
	bool stable = false;
	while ( (i<max_us) && (!(stable=hal_is_pll_stable())) ) {
		i++;
		k_sleep(1);
	}
	return stable;
}

bool power_config_best_match(int16_t dbm, uint8_t* cfg) {
	int opt_index = -1;
	int opt_delta = 0;

	for (int i=0; i<sizeof(power_config_lut)/sizeof(power_config_lut[0]); i++) {
		int delta = ABS_DELTA(dbm,power_config_lut[i].dbm);
		if ((opt_index == -1) || (delta<opt_delta) ) {
			opt_index = i;
			opt_delta = delta;
		}
	}
	if (opt_index == -1) {
		LOG_ERR("failed to find a power configuration value");
		*cfg = DEFAULT_TX_POWER_CONFIG;
		return false;
	}
	LOG_INF("using power configuration 0x%02x for %d dBm, approximating %d dBm",
			power_config_lut[opt_index].hw_cfg,
			power_config_lut[opt_index].dbm,
			dbm
		);
	*cfg = power_config_lut[opt_index].hw_cfg;
	return true;
}

void initialize_registers(void) {
	for (int i=0; i<sizeof(kv_init_table)/sizeof(kv_init_table[0]); i++) {
		//dbg_print("hi %d", i);
		//LOG_DBG("Initializing %p to 0x%08x", kv_init_table[i].address, kv_init_table[i].val);
		HWREG(kv_init_table[i].address) = kv_init_table[i].val;
	}
}

void initialize_peripheral(void) {
	//IMPORTANT: Activate Clocks in all (Sleep)Modes. Otherwise strange behaviour might/will be observed.
	//Problem => Works only once after flashing, no more after either reset or manual on/off
	//Reason <= Debug Connection from TI-Flasher apparently changes those registers.
	HWREG(SYS_CTRL_RCGCRFC) = 1;
	HWREG(SYS_CTRL_SCGCRFC) = 1;
	HWREG(SYS_CTRL_DCGCRFC) = 1;
	SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC); //Enable Peripheral
}

static void force_disable(void) {
	ISRFOFF();
	SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_RFC); //Disable the peripheral
}

void generate_mac_address(uint8_t mac[8]) {
	//copy configured bytes
	mac[0] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,0);
	mac[1] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,1);
	mac[2] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,2);
	mac[3] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,3);
	mac[4] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,4);
	mac[5] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,5);
	mac[6] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,6);
	mac[7] = MAC_ADDR_BYTE(CONFIG_IEEE802154_TI_CC2538_RF_MAC,7);

	//randomize bytes
	for (int i=0; i<CONFIG_IEEE802154_TI_CC2538_RF_MAC_RANDOM_BYTES; i++) {
		mac[7-i] = (uint8_t)(sys_rand32_get()%256);
	}

	//TODO: introduce configuration option?
	//make sure this is a individual mac by clearing byte 0 bit 0
	mac[0] &= ~(1<<0);
	//make sure this is a locally administered address by setting byte 0 bit 1
	mac[0] |= (1<<1);

	LOG_INF("generated MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
}

void initialize_interrupts() {
	//install zephyr irq handlers
	IRQ_CONNECT(CC2538_RF_IRQ, CC2538_RF_PRIO, rf_handler, DEVICE_GET(ti_cc2538_rf_radio), CC2538_RF_FLAGS);
	IRQ_CONNECT(CC2538_RFERR_IRQ, CC2538_RFERR_PRIO, rferr_handler,	DEVICE_GET(ti_cc2538_rf_radio), CC2538_RFERR_FLAGS);
	//enable irqs
	irq_enable(CC2538_RF_IRQ);
	irq_enable(CC2538_RFERR_IRQ);

	//enable irq sources
	HWREG(RFCORE_XREG_RFIRQM0) |= 0
		| (0<<RFIRQM0_RXMASKZERO_S)
		| (0<<RFIRQM0_RXPKTDONE_S)
		| (1<<RFIRQM0_FRAME_ACCEPTED_S)
		| (0<<RFIRQM0_SRC_MATCH_FOUND_S)
		| (0<<RFIRQM0_SRC_MATCH_DONE_S)
		| (0<<RFIRQM0_FIFOP_S)
		| (0<<RFIRQM0_SFD_S)
	;
	HWREG(RFCORE_XREG_RFIRQM1) |= 0
		| (0<<RFIRQM1_CSP_WAIT_S)
		| (0<<RFIRQM1_CSP_STOP_S)
		| (0<<RFIRQM1_CSP_MANINT_S)
		| (0<<RFIRQM1_RF_IDLE_S)
		| (1<<RFIRQM1_TXDONE_S)
		| (0<<RFIRQM1_TXACKDONE_S)
	;
	HWREG(RFCORE_XREG_RFERRM) |= 0
		| (1<<RFERRM_STROBE_ERR_S)
		| (1<<RFERRM_TXUNDERF_S)
		| (1<<RFERRM_TXOVERF_S)
		| (1<<RFERRM_RXUNDERF_S)
		| (1<<RFERRM_RXOVERF_S)
		| (1<<RFERRM_RXABO_S)
		| (1<<RFERRM_NLOCK_S)
	;

	//enable zephyr irq
	IntEnable(INT_RFCORERTX);
	IntEnable(INT_RFCOREERR);
}

// ================ ================ ================ ================
// Radio API Implementation
// ================ ================ ================ ================
//m:impl

static int ti_cc2538_rf_init(struct device *dev) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	//print some debug info
	LOG_WRN("driver under development");
	hal_dbg_hwinfo();

	//disable if running
	LOG_INF("initialize peripheral disable");
	force_disable();

	//Setting Clock and Enabling Peripheral
	LOG_INF("initialize peripheral");
	initialize_peripheral();

	//initialize hardware configuration using (address,value) pairs
	LOG_INF("initialize registers");
	initialize_registers();

	//runtime configuration
	LOG_INF("initialize configuration");
	generate_mac_address(&cc2538rf->mac_addr[0]);
	uint8_t pwr_cfg = 0;
	power_config_best_match(CONFIG_NET_CONFIG_IEEE802154_RADIO_TX_POWER, &pwr_cfg);
	hal_cfg_set_pan_id(CONFIG_NET_CONFIG_IEEE802154_PAN_ID);
	hal_cfg_set_short_addr(CONFIG_IEEE802154_TI_CC2538_RF_SHORT_ADDR);
	hal_cfg_set_ieee_addr(&cc2538rf->mac_addr[0]);
	hal_cfg_set_channel(CONFIG_NET_CONFIG_IEEE802154_CHANNEL);
	hal_cfg_set_output_power(pwr_cfg);
	hal_cfg_set_fifop_threshold(0x20);

	//initialize device data
	LOG_INF("intialize device data");
	cc2538rf->rx_pkt = NULL;
	cc2538rf->rx_overflow = false;
	cc2538rf->rx_underflow = false;
	atomic_set(&cc2538rf->rf_on, 0);
	atomic_set(&cc2538rf->tx_pkt_do, 0);
	atomic_set(&cc2538rf->rx_pkt_avail, 0);
	k_sem_init(&cc2538rf->rf_access_lock, 0, 1);
	k_sem_init(&cc2538rf->tx_pkt_done, 0, 1);
	k_sem_init(&cc2538rf->rx_pkt_done, 0, UINT_MAX);

	//configure interrupts
	LOG_INF("initialize interrupts");
	initialize_interrupts();


	//initialize rx thread
	LOG_INF("initialize rx thread");
	k_thread_create(
		&cc2538rf->ti_cc2538_rf_rx_thread, //thread variable
		cc2538rf->ti_cc2538_rf_rx_stack, //thread stack
		CONFIG_IEEE802154_TI_CC2538_RF_RX_STACK_SIZE, //thread stack size
		(k_thread_entry_t) ti_cc2538_rf_rx, //thread function
		dev, NULL, NULL, //thread function arguments
		//K_PRIO_COOP(2), 0, 0 //priority, options, delay
		K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES-1), 0, 0 //priority, options, delay
	);

	k_sem_give(&cc2538rf->rf_access_lock);

	LOG_INF("initialization done");

	return 0;
}


static void ti_cc2538_rf_iface_init(struct net_if *iface) {
	struct device *dev = net_if_get_device(iface);
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	net_if_set_link_addr(iface, (uint8_t*)(&(cc2538rf->mac_addr[0])), 8, NET_LINK_IEEE802154);
	cc2538rf->iface = iface;
	ieee802154_init(iface);
}


static enum ieee802154_hw_caps ti_cc2538_rf_get_capabilities(struct device *dev) {
	//struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;
	return 0
		| IEEE802154_HW_FCS /* Frame Check-Sum supported */
		//| IEEE802154_HW_PROMISC /* Promiscuous mode supported */
		| IEEE802154_HW_FILTER /* Filters PAN ID, long/short addr */
		| IEEE802154_HW_CSMA /* CSMA-CA supported */
		| IEEE802154_HW_2_4_GHZ /* 2.4Ghz radio supported */
		//| IEEE802154_HW_TX_RX_ACK /* Handles ACK request on TX */
		//| IEEE802154_HW_SUB_GHZ /* Sub-GHz radio supported */
		| 0;
}


static int ti_cc2538_rf_start(struct device *dev) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (k_sem_take(&cc2538rf->rf_access_lock, DEFAULT_SEMAPHORE_TIMEOUT) != 0) {
		LOG_ERR("failed to acquire rf_access_lock");
		return -EIO;
	}

//_try:
	int r = -EIO;

	if (atomic_get(&cc2538rf->rf_on) != 0) {
		LOG_ERR("already started");
		r = -EIO;
		goto _finally;
	}

	hal_rf_enable();
	if (!wait_pll_stable(DEFAULT_PLL_TIMEOUT)) {
		LOG_ERR("pll did not become stable");
		hal_rf_disable();
		r = -EIO;
		goto _finally;
	}

	atomic_set(&cc2538rf->rf_on, 1);
	r = 0;

_finally:
	k_sem_give(&cc2538rf->rf_access_lock);
	return r;
}


static int ti_cc2538_rf_stop(struct device *dev) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (k_sem_take(&cc2538rf->rf_access_lock, DEFAULT_SEMAPHORE_TIMEOUT) != 0) {
		LOG_ERR("failed to acquire rf_access_lock");
		return -EIO;
	}

//_try:
	int r = -EIO;

	if (atomic_get(&cc2538rf->rf_on) != 1) {
		LOG_ERR("already started");
		r = -EIO;
		goto _finally;
	}

	atomic_set(&cc2538rf->rf_on, 0);
	hal_rf_disable();
	r = 0;

_finally:
	k_sem_give(&cc2538rf->rf_access_lock);
	return r;
}


static int ti_cc2538_rf_set_channel(struct device *dev, u16_t channel){

	NET_ASSERT(channel >= MIN_CHANNEL);
	NET_ASSERT(channel <= MAX_CHANNEL);

	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (k_sem_take(&cc2538rf->rf_access_lock, DEFAULT_SEMAPHORE_TIMEOUT) != 0) {
		LOG_ERR("failed to acquire rf_access_lock");
		return -EIO;
	}

//_try:
	int r = -EIO;

	if (atomic_get(&cc2538rf->rf_on) != 0) {
		LOG_ERR("modification while rf active not allowed");
		r = -EIO;
		goto _finally;
	}

	hal_cfg_set_channel(channel);
	r = 0;

_finally:
	k_sem_give(&cc2538rf->rf_access_lock);
	return r;
}


static int ti_cc2538_rf_set_filter(
		struct device *dev,
		bool set,
		enum ieee802154_filter_type type,
		const struct ieee802154_filter *filter) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (k_sem_take(&cc2538rf->rf_access_lock, DEFAULT_SEMAPHORE_TIMEOUT) != 0) {
		LOG_ERR("failed to acquire rf_access_lock");
		return -EIO;
	}

//_try:
	int r = -EIO;
	if (atomic_get(&cc2538rf->rf_on) != 0) {
		LOG_ERR("modification while rf active not allowed");
		r = -EIO;
		goto _finally;
	}

	if (set != true) {
		LOG_ERR("deactivating filters not supported");
		r = -ENOTSUP;
		goto _finally;
	}

	switch (type) {
		case IEEE802154_FILTER_TYPE_IEEE_ADDR: {
			LOG_DBG("applying IEEE_ADDR filter");
			hal_cfg_set_ieee_addr(filter->ieee_addr);
			r = 0;
			break;
		}
		case IEEE802154_FILTER_TYPE_SHORT_ADDR: {
			LOG_DBG("applying SHORT_ADDR filter");
			hal_cfg_set_short_addr(filter->short_addr);
			r = 0;
			break;
		}
		case IEEE802154_FILTER_TYPE_PAN_ID: {
			LOG_DBG("applying PAN_ID filter");
			hal_cfg_set_pan_id(filter->pan_id);
			r = 0;
			break;
		}
		default: {
			LOG_ERR("filter type not supported");
			r = -ENOTSUP;
			goto _finally;
		}
	}

_finally:
	k_sem_give(&cc2538rf->rf_access_lock);
	return r;
}


static int ti_cc2538_rf_set_txpower(struct device *dev, s16_t dbm) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (k_sem_take(&cc2538rf->rf_access_lock, DEFAULT_SEMAPHORE_TIMEOUT) != 0) {
		LOG_ERR("failed to acquire rf_access_lock");
		return -EIO;
	}

//_try:
	int r = -EIO;

	uint8_t cfg=0;
	if (!power_config_best_match(dbm, &cfg)) {
		LOG_ERR("no suitable power configuration found");
		r = -ENOTSUP;
		goto _finally;
	}

	hal_cfg_set_output_power(cfg);
	r=0;

_finally:
	k_sem_give(&cc2538rf->rf_access_lock);
	return r;
}


static int ti_cc2538_rf_cca(struct device *dev) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	if (!atomic_get(&cc2538rf->rf_on)) {
		LOG_ERR("rf core not started");
		return -EIO;
	}
	wait_rssi_valid();
	if (!hal_cca()) {
		return -EBUSY;
	}
	return 0;
}


static void ti_cc2538_rf_rx(int arg1, int arg2, int arg3) {
	struct device *dev = INT_TO_POINTER(arg1);
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;
	struct net_buf *pkt_frag = NULL;
	struct net_pkt *pkt = NULL;
	uint8_t pkt_len;
	bool rx_abort;
	bool rx_drop;
static uint64_t cnt=0;
	while (1) {
		pkt = NULL;
		rx_abort = false;
		rx_drop = false;
		k_sem_take(&cc2538rf->rx_pkt_done, K_FOREVER);

		if (cc2538rf->rx_overflow || cc2538rf->rx_underflow) {
			LOG_ERR("rx over-/underflow detected, flushing.");
			goto flush;
		}

		pkt_len = hal_fifo_read_byte();
cnt+=pkt_len;
		LOG_DBG("rx length: %u", pkt_len);

		if (pkt_len < 2) {
			LOG_ERR("Invalid rx fifo content: length less than 2");
			goto flush;
		}

		if (pkt_len > 127) {
			LOG_ERR("Invalid rx fifo content: length greater than 125");
			goto flush;
		}

		pkt = net_pkt_get_reserve_rx(K_NO_WAIT);

		if (!pkt) {
			LOG_WRN("No pkt available, dropping");
			LOG_ERR("No pkt available, dropping");
			rx_drop = true;
			goto drop;
		}

		pkt_frag = net_pkt_get_frag(pkt, K_NO_WAIT);

		if (!pkt_frag) {
			LOG_WRN("No pkt_frag available");
			LOG_ERR("No pkt_frag available");
			rx_drop = true;
			goto drop;
		}
		net_pkt_frag_insert(pkt, pkt_frag);

		uint8_t* pkt_data = pkt_frag->data;

		for (uint8_t i=0; i<pkt_len-CC2538_FCS_LENGTH; i++) {
			//simple, but inefficient approach to avoid underflows
			while (hal_fifo_rx_count()<1) {
				if (cc2538rf->rx_overflow || cc2538rf->rx_underflow) {
					goto flush;
				}
			}
			pkt_data[i] = hal_fifo_read_byte();
		}
		while (hal_fifo_rx_count()<CC2538_FCS_LENGTH) {
				if (cc2538rf->rx_overflow || cc2538rf->rx_underflow) {
					goto flush;
				}
		}

		uint8_t meta0 = hal_fifo_read_byte(); //rssi
		uint8_t meta1 = hal_fifo_read_byte(); //crc_ok, correlation

		net_buf_add(pkt_frag, pkt_len-CC2538_FCS_LENGTH);

		if (!(meta1&CC2538_FCS_CRC_OK)) {
			LOG_ERR("Bad packet CRC");
			goto out;
		}

		net_pkt_set_ieee802154_rssi(pkt, meta0);

		uint8_t lqi = meta1 & CC2538_FCS_CORRELATION;
		if (lqi <= 50) {
			lqi = 0;
		} else if (lqi >= 110) {
			lqi = 255;
		} else {
			lqi = (lqi - 50) << 2;
		}

		net_pkt_set_ieee802154_lqi(pkt, lqi);

		if (ieee802154_radio_handle_ack(cc2538rf->iface, pkt) == NET_OK) {
			LOG_DBG("ACK packet handled");
			goto out;
		}

		if (net_recv_data(cc2538rf->iface, pkt) < 0) {
			LOG_DBG("Packet dropped by NET stack");
			LOG_ERR("Packet dropped by NET stack");
			goto out;
		}

/*
		net_analyze_stack("CC2538 RX fiber stack",
				K_THREAD_STACK_BUFFER(cc2538rf->ti_cc2538_rf_rx_stack),
				K_THREAD_STACK_SIZEOF(cc2538rf->ti_cc2538_rf_rx_stack)
		);
*/
		continue;
drop:
		for (int i=0; i<pkt_len; i++) {
			while (hal_fifo_rx_count()<1) {
				//avoid underflows
			}
			(void)(hal_fifo_read_byte());
		}
		goto out;
flush:
		hal_dbg_irqinfo();
		ISFLUSHRX();	//flush rx buffer
		cc2538rf->rx_overflow = false; //reset flags
		cc2538rf->rx_underflow = false; //reset flags
		k_sem_init(&cc2538rf->rx_pkt_done, 0, 1); //reset in case something happen meanwhile
		if (atomic_get(&cc2538rf->rf_on)) { //restart if it should be online
			atomic_set(&cc2538rf->rf_on, 0);
			ti_cc2538_rf_start(dev);
		}
		goto out;
out:
		if (pkt) {
			net_pkt_unref(pkt);
		}
	}
/*
	while (1) {
		k_sem_take(&cc2538rf->rx_pkt_done, K_FOREVER);
		pkt = cc2538rf->rx_pkt;
		len = cc2538rf->rx_len;
		LOG_ERR("rx thread recieved pkt with length %d", len);

		if (!(cc2538rf->rx_fcs1&CC2538_FCS_CRC_OK)) {
			LOG_ERR("Bad packet CRC");
			goto done;
		}

		net_pkt_set_ieee802154_rssi(pkt, cc2538rf->rx_fcs0);

		uint8_t lqi = cc2538rf->rx_fcs1 & CC2538_FCS_CORRELATION;
		if (lqi <= 50) {
			lqi = 0;
		} else if (lqi >= 110) {
			lqi = 255;
		} else {
			lqi = (lqi - 50) << 2;
		}

		net_pkt_set_ieee802154_lqi(pkt, lqi);

		if (ieee802154_radio_handle_ack(cc2538rf->iface, pkt) == NET_OK) {
			LOG_DBG("ACK packet handled");
			goto done;
		}

		if (net_recv_data(cc2538rf->iface, pkt) < 0) {
			LOG_DBG("Packet dropped by NET stack");
			goto done;
		}

		net_analyze_stack(
			"CC2538 RX fiber stack",
			K_THREAD_STACK_BUFFER(cc2538rf->ti_cc2538_rf_rx_stack),
			K_THREAD_STACK_SIZEOF(cc2538rf->ti_cc2538_rf_rx_stack)
		);

		pkt = NULL;
		done:
			atomic_set(&cc2538rf->rx_pkt_avail, 0);
			if (pkt!=NULL) {
				net_pkt_unref(pkt);
			}
	}
*/
}

static int ti_cc2538_rf_tx(struct device *dev,
	struct net_pkt *pkt,
	struct net_buf *frag) {
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;

	//uint8_t* frame = frag->data - net_pkt_ll_reserve(pkt); //pointer to payload with header before
	//uint8_t len = frag->len + net_pkt_ll_reserve(pkt); //length of payload with header before
	uint8_t* frame = frag->data; //pointer to payload with header before
	uint8_t len = frag->len; //length of payload with header before
	uint8_t flen = 0;

	ISFLUSHTX();
	while (hal_fifo_tx_count()!=0) {
		//wait until flush complete
		//otherwise errors seem to occur
	}
	hal_fifo_write_byte(len+CC2538_FCS_LENGTH); //write payload length to fifo (include 2 byte FCS added automatically)
	hal_fifo_write(frame,len); //write payload to fifo
	flen = hal_fifo_tx_count();
	if (flen != len+1) { //include length byte but not FCS, because FCS is added by hardware when transmitting
		LOG_ERR("failed to write to tx fifo (%d vs %d)", flen, len+1);
		return -EIO;
	}
	//NOTE: for debugging, if we want to wait until no TX ACTIVE
	//while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE) { k_sleep(1); }

	//NOTE: for debugging, if we want to wait until no RX ACTIVE
	//while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE) { k_sleep(1); }

	//NOTE: for debugging, if we want to wait until RX is finished
	//while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_SFD) { k_sleep(1); }

	//NOTE: for debugging, if we want to wait until it is available
	//while (ti_cc2538_rf_cca(dev)!=0) { _usleep(10); }

	// this Block starts transmission and verifies
	// that it did do indeed start transmittig
	// by waiting on a semaphore set by the sfd interrupt
	uint8_t retry = 2;
	bool status = false;
	//LOG_ERR("trying tx");
	do { // 1 retry is allowed here
		retry--;
		k_sem_init(&cc2538rf->tx_pkt_done, 0, UINT_MAX);
		atomic_set(&cc2538rf->tx_pkt_do, 1);
		ISSAMPLECCA();
		wait_rssi_valid();
		if (!hal_cca()) {
			continue;
		}
		ISTXON();
//		ISTXONCCA(); //problematic somehow
		if (k_sem_take(&cc2538rf->tx_pkt_done, 50) != 0) {
			LOG_DBG("tx did not complete");
			atomic_set(&cc2538rf->tx_pkt_do, 0);
			continue;
		}
/* xxx */
		status = true;
		atomic_set(&cc2538rf->tx_pkt_do, 0);
	} while (!status && retry);

	if (status) {
		LOG_DBG("sent TX frame");
		return 0;
	} else {
		LOG_ERR("error sending TX frame");
		atomic_set(&cc2538rf->tx_pkt_do, 0);
		return -EIO;
	}
}

volatile uint8_t thrash = 0;

static void rf_handler(void* arg) {
	struct device* dev = (struct device*)arg;
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;
	uint8_t irqf0 = hal_irqf0();
	uint8_t irqf1 = hal_irqf1();
	hal_irqf0_clear_enabled();
	hal_irqf1_clear_enabled();
	LOG_DBG("rf_handler");
	if (irqf1 & (1<<RFIRQM1_TXDONE_S)) {
		if (atomic_get(&cc2538rf->tx_pkt_do)) {
			////LOG_ERR("irqf1 giving semaphore!");
			k_sem_give(&cc2538rf->tx_pkt_done);
		} else {
			////LOG_ERR("TXDONE but not tx_pkt_do");
		}
	}
	if (irqf0 & (1<<RFIRQM0_FRAME_ACCEPTED_S)) {
		k_sem_give(&cc2538rf->rx_pkt_done);
	}
}

static void rferr_handler(void* arg) {
	struct device* dev = (struct device*)arg;
	struct ti_cc2538_rf_context_t* cc2538rf = dev->driver_data;
	uint8_t errf = hal_errf();
	hal_dbg_irqinfo();
	hal_errf_clear_enabled();
	if (errf & (1<<RFERRM_RXOVERF_S)) {
		hal_rf_disable();
		cc2538rf->rx_overflow = true;
		k_sem_give(&cc2538rf->rx_pkt_done);
	}
	if (errf & (1<<RFERRM_RXUNDERF_S)) {
		hal_rf_disable();
		cc2538rf->rx_underflow = true;
		k_sem_give(&cc2538rf->rx_pkt_done);
	}
}

/*
	if (errf & (1<<RFERRM_RXOVERF_S)) {
		LOG_INF("rxoverflow: buffer flush and rf rx restart");
		ISRFOFF();
		ISFLUSHRX();
		hal_rf_enable();
	}
*/

/*
	if (irqf0 & (1<<RFIRQM0_FIFOP_S)) {
		if (atomic_get(&cc2538rf->rx_pkt_avail) != 0) {
			//discard
			uint8_t pkt_len = hal_fifo_read_byte();
			LOG_ERR("discard %d", pkt_len);
			for (uint8_t i=0; i<pkt_len; i++) {
				while (hal_fifo_rx_count()<1) {
					//wait
				}
				thrash = hal_fifo_read_byte();
			}
		} else {
			//store
			struct net_pkt *pkt = NULL;
			struct net_buf *pkt_frag = NULL;
			pkt = net_pkt_get_reserve_rx(0, K_NO_WAIT);
			if (!pkt) {
				LOG_ERR("No pkt");
				goto rx_err;
			}
			pkt_frag = net_pkt_get_frag(pkt, K_NO_WAIT);
			if (!pkt_frag) {
				LOG_ERR("No pkt_frag");
				goto rx_err;
			}
			net_pkt_frag_insert(pkt, pkt_frag);
			uint8_t* pkt_data = pkt_frag->data;
			uint8_t pkt_len = hal_fifo_read_byte()-CC2538_FCS_LENGTH;
			for (uint8_t i=0; i<pkt_len; i++) {
				while (hal_fifo_rx_count()<1) {
					//wait
				}
				pkt_data[i] = hal_fifo_read_byte();
			}
			while (hal_fifo_rx_count()<CC2538_FCS_LENGTH) {
				//wait
			}
			cc2538rf->rx_fcs0 = hal_fifo_read_byte();
			cc2538rf->rx_fcs1 = hal_fifo_read_byte();
			net_buf_add(pkt_frag, pkt_len);
			cc2538rf->rx_pkt = pkt;
			pkt = NULL;
			cc2538rf->rx_len = pkt_len;
			atomic_set(&cc2538rf->rx_pkt_avail, 1);
			k_sem_give(&cc2538rf->rx_pkt_done);
			rx_err:
				if (pkt!=NULL) {
					net_pkt_unref(pkt);
				}
		}
	}
*/
