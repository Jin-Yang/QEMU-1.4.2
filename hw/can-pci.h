/*
 * CAN device (SJA1000) simulation based on PCI-bus
 *
 * Copyright (c) 2013 Jin Yang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
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
	char			*model; // The model that support, only SJA1000 now.
} PCICanState;




/* PeliCAN mode */
enum SJA1000_PeliCAN_regs {
	SJAMOD	= 0x00,
/// Command register
	SJACMR 	= 0x01,
/// Status register
	SJASR	= 0x02,
/// Interrupt register
	SJAIR	= 0x03,
/// Interrupt Enable
	SJAIER	= 0x04,
/// Bus Timing register 0
	SJABTR0 = 0x06,
/// Bus Timing register 1
	SJABTR1	= 0x07,
/// Output Control register
	SJAOCR	= 0x08,
/// Arbitration Lost Capture
	SJAALC	= 0x0b,
/// Error Code Capture
	SJAECC	= 0x0c,
/// Error Warning Limit
	SJAEWLR = 0x0d,
/// RX Error Counter
	SJARXERR = 0x0e,
/// TX Error Counter
	SJATXERR0 = 0x0e,
	SJATXERR1 = 0x0f,
/// Rx Message Counter (number of msgs. in RX FIFO
	SJARMC	= 0x1d,
/// Rx Buffer Start Addr. (address of current MSG)
	SJARBSA	= 0x1e,
/// Transmit Buffer (write) Receive Buffer (read) Frame Information
	SJAFRM = 0x10,
/// ID bytes (11 bits in 0 and 1 or 16 bits in 0,1 and 13 bits in 2,3 (extended))
	SJAID0 = 0x11, SJAID1 = 0x12,
/// ID cont. for extended frames
	SJAID2 = 0x13, SJAID3 = 0x14,
/// Data start standard frame
	SJADATS = 0x13,
/// Data start extended frame
	SJADATE = 0x15,
/// Acceptance Code (4 bytes) in RESET mode
	SJAACR0	= 0x10,
/// Acceptance Mask (4 bytes) in RESET mode
	SJAAMR0	= 0x14,
/// 4 bytes
	SJA_PeliCAN_AC_LEN = 4,
/// Clock Divider
	SJACDR = 0x1f
};


/* PeliCAN mode */
enum SJA1000_BasicCAN_regs {
	B_SJACTR	= 0x00,
/// Command register
	B_SJACMR 	= 0x01,
/// Status register
	B_SJASR 	= 0x02,
/// Interrupt register
	B_SJAIR 	= 0x03
};





#endif

