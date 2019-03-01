// (Created by [TheM])
/* ieee802154_cc2538.h - TI CC2538 driver */

#ifndef __IEEE802154_CC2538_H__
#define __IEEE802154_CC2538_H__

#include <atomic.h>
#include <inc/hw_rfcore_xreg.h>
#include <inc/hw_rfcore_ffsm.h>
#include <inc/hw_rfcore_sfr.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_sys_ctrl.h>
#include <inc/hw_ana_regs.h>
#include <inc/hw_flash_ctrl.h>
#include <inc/hw_memmap.h>
#include <driverlib/interrupt.h>
#include <driverlib/sys_ctrl.h>

//constants
#define MIN_CHANNEL (11)
#define MAX_CHANNEL (26)
#define CHANNEL_SPACING (5)
#define MIN_DBM (-24)
#define MAX_DBM (7)

#define CC2538_FCS_LENGTH (2)
#define CC2538_FCS_CRC_OK		(0x80)
#define CC2538_FCS_CORRELATION 	(0x7F)

#define RFIRQM0_RXMASKZERO_S (7)
#define RFIRQM0_RXPKTDONE_S (6)
#define RFIRQM0_FRAME_ACCEPTED_S (5)
#define RFIRQM0_SRC_MATCH_FOUND_S (4)
#define RFIRQM0_SRC_MATCH_DONE_S (3)
#define RFIRQM0_FIFOP_S (2)
#define RFIRQM0_SFD_S (1)

#define RFIRQM1_CSP_WAIT_S (5)
#define RFIRQM1_CSP_STOP_S (4)
#define RFIRQM1_CSP_MANINT_S (3)
#define RFIRQM1_RF_IDLE_S (2)
#define RFIRQM1_TXDONE_S (1)
#define RFIRQM1_TXACKDONE_S (0)

#define RFERRM_STROBE_ERR_S (6)
#define RFERRM_TXUNDERF_S (5)
#define RFERRM_TXOVERF_S (4)
#define RFERRM_RXUNDERF_S (3)
#define RFERRM_RXOVERF_S (2)
#define RFERRM_RXABO_S (1)
#define RFERRM_NLOCK_S (0)

#define CC2538_RF_IRQ		(INT_RFCORERTX-16)
//#define CC2538_RF_PRIO		(-CONFIG_NUM_COOP_PRIORITIES)
#define CC2538_RF_PRIO		(K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES-1))
#define CC2538_RF_FLAGS		0
#define CC2538_RFERR_IRQ	(INT_RFCOREERR-16)
#define CC2538_RFERR_PRIO 	1
#define CC2538_RFERR_FLAGS	0


//default configuration values
#define DEFAULT_SEMAPHORE_TIMEOUT (10) //in milliseconds
#define DEFAULT_PLL_TIMEOUT (1000) //in milliseconds
#define DEFAULT_TX_POWER_CONFIG (0x5F) //default by datasheet, returned if none found

//macros
#define HI_UINT32(a) ((uint16_t) (((uint32_t)(a)) >> 16))
#define HI_UINT16(a) (((uint16_t)(a) >> 8) & 0xFF)
#define HI_UINT8(a) (((uint8_t)(a) >> 4) & 0x0F)
#define LO_UINT32(a) ((uint16_t) ((uint32)(a)))
#define LO_UINT16(a) ((uint16_t)(a) & 0xFF)
#define LO_UINT8(a) ((uint8_t)(a) & 0x0F)
#define MAC_ADDR_BYTE(addr,byte) ((addr>>((7-byte)*8))&0xFF)
#define ABS_DELTA(a,b) (a>b?(a-b):(b-a))

struct ti_cc2538_rf_context_t {
	//api specific
	struct net_if *iface;
	struct k_thread ti_cc2538_rf_rx_thread;
	K_THREAD_STACK_MEMBER(ti_cc2538_rf_rx_stack, CONFIG_IEEE802154_TI_CC2538_RF_RX_STACK_SIZE);
	//general control
	struct k_sem rf_access_lock; //protects on access to the rf hardware
	atomic_t rf_on;
	atomic_t rf_active;
	//general data
	uint8_t mac_addr[8];
	//tx
	atomic_t tx_pkt_do;
	struct k_sem tx_pkt_done;
	//rx
	atomic_t rx_pkt_avail;
	struct k_sem rx_pkt_done;
	struct net_pkt *rx_pkt;
	uint8_t rx_len;
	uint8_t rx_fcs0;
	uint8_t rx_fcs1;
	bool 	rx_overflow;
	bool	rx_underflow;
};



#endif //__IEEE802154_CC2538_H__
