#ifndef HW_CAN_PCI_H
#define HW_CAN_PCI_H

#define PCI_MEM_SIZE				128
#define PCI_DEVICE_ID_CANBUS		0xbeef
#define PCI_REVISION_ID_CANBUS 		0x73 

#define MEM_BAR						0

// The max size for a message buffer, EFF and DLC=8, DS-p39
#define SJA_MSG_MAX_LEN				13
// The receive buffer size.
#define SJA_RCV_BUF_LEN				64


//#define DEBUG_CAN
#ifdef DEBUG_CAN
#define DPRINTF(fmt, ...) \
   do { fprintf(stderr, "[mycan]: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
   do {} while (0)
#endif



/* NOTE: the following two structures is copied from <linux/can.h>. */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error frame flag (0 = data frame, 1 = error frame)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

struct can_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* data length code: 0 .. 8 */
	__u8    data[8] __attribute__((aligned(8)));
};

/**
 * struct can_filter - CAN ID based filter in can_register().
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *          <received_can_id> & mask == can_id & mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error message frames (CAN_ERR_FLAG bit set in mask).
 */
struct can_filter {
	canid_t can_id;
	canid_t can_mask;
};

#define CAN_INV_FILTER 0x20000000U /* to be set in can_filter.can_id */




typedef struct CanState {
	/* Some registers ... */
	uint8_t			mode; 			// PeliCAN, addr 0, Mode register, DS-p26
									// PeliCAN, addr 1, Command register
	uint8_t			statusP; 		// PeliCAN, addr 2, Status register, p15
	uint8_t			interruptP;		// PeliCAN, addr 3, Interrupt register
	uint8_t			interrupt_en; 	// PeliCAN, addr 4, Interrupt Enable register
	uint8_t			rxmsg_cnt; 		// PeliCAN, addr 29, RX message counter. DS-p49
	uint8_t			rxbuf_start; 	// PeliCAN, addr 30, RX buffer start address register, DS-p49
	uint8_t			clock; 			// PeliCAN, addr 31, Clock Divider register, DS-p55  

	uint8_t			code_mask[8];	// PeliCAN, addr 16~23
	uint8_t			tx_buff[13];	// PeliCAN, addr 96~108, transmit buffer
									// BasicCAN, addr 10~19, transmit buffer

	uint8_t			rx_buff[SJA_RCV_BUF_LEN];  // 32~95, 64bytes
	int				rx_ptr;         // Count by bytes.
	int				rx_cnt; 		// Count by bytes.

	uint8_t			control;		// BasicCAN, addr 0, Control register
									// BasicCAN, addr 1, Command register
	uint8_t			statusB;		// BasicCAN, addr 2, Status register
	uint8_t			interruptB;		// BasicCAN, addr 3, Interrupt register
	uint8_t			code;			// BasicCAN, addr 4, Acceptance code register
	uint8_t			mask;			// BasicCAN, addr 5, Acceptance mask register


	struct can_filter	filter[4];

	QemuMutex		rx_lock;
    qemu_irq 		irq;
    CharDriverState *chr;
    MemoryRegion 	memio;
} CanState;

typedef struct PCICanState {
    PCIDevice 		dev;
    CanState 		state;
	char			*model;
} PCICanState;




#endif

