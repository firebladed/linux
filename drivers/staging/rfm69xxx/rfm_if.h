// SPDX-License-Identifier: GPL-2.0+
/*
 * userspace interface for rfm69 radio module
 *
 * rfm69 is a radio module created by HopeRf Module. 
 * Therefore inside of this
 * driver, you'll find an abstraction of the rf69 chip.
 *
 *
 *
 *
 * Based on pi433 Driver by
 *
 * Copyright (C) 2016 Wolf-Entwicklungen
 *	Marcus Wolf <linux@wolf-entwicklungen.de>
 */
 

#ifndef rfm69_H
#define rfm69_H

#include <linux/types.h>
#include "rf69_enum.h"

/*---------------------------------------------------------------------------*/

enum option_on_off {
	OPTION_OFF,
	OPTION_ON
};

/* IOCTL structs and commands */

/**
 * struct rfm69_tx_config
 * describes the configuration of the radio module for sending
 * @frequency:
 * @bit_rate:
 * @modulation:
 * @data_mode:
 * @preamble_length:
 * @sync_pattern:
 * @tx_start_condition:
 * @payload_length:
 * @repetitions:
 *
 * ATTENTION:
 * If the contents of 'rfm69_tx_config' ever change
 * incompatibly, then the ioctl number (see define below) must change.
 *
 * NOTE: struct layout is the same in 64bit and 32bit userspace.
 */
#define rfm69_TX_CFG_IOCTL_NR	0
struct rfm69_tx_cfg {
	__u32			frequency;
	__u16			bit_rate;
	__u32			dev_frequency;
	enum modulation		modulation;
	enum mod_shaping	mod_shaping;

	enum pa_ramp		pa_ramp;

	enum tx_start_condition	tx_start_condition;

	__u16			repetitions;

	/* packet format */
	enum option_on_off	enable_preamble;
	enum option_on_off	enable_sync;
	enum option_on_off	enable_length_byte;
	enum option_on_off	enable_address_byte;
	enum option_on_off	enable_crc;

	__u16			preamble_length;
	__u8			sync_length;
	__u8			fixed_message_length;

	__u8			sync_pattern[8];
	__u8			address_byte;
};

/**
 * struct rfm69_rx_config
 * describes the configuration of the radio module for sending
 * @frequency:
 * @bit_rate:
 * @modulation:
 * @data_mode:
 * @preamble_length:
 * @sync_pattern:
 * @tx_start_condition:
 * @payload_length:
 * @repetitions:
 *
 * ATTENTION:
 * If the contents of 'rfm69_rx_config' ever change
 * incompatibly, then the ioctl number (see define below) must change
 *
 * NOTE: struct layout is the same in 64bit and 32bit userspace.
 */
#define rfm69_RX_CFG_IOCTL_NR	1
struct rfm69_rx_cfg {
	__u32			frequency;
	__u16			bit_rate;
	__u32			dev_frequency;

	enum modulation		modulation;

	__u8			rssi_threshold;
	enum threshold_decrement threshold_decrement;
	enum antenna_impedance	antenna_impedance;
	enum lna_gain		lna_gain;
	enum mantisse		bw_mantisse;	/* normal: 0x50 */
	__u8			bw_exponent;	/* during AFC: 0x8b */
	enum dagc		dagc;

	/* packet format */
	enum option_on_off	enable_sync;
	enum option_on_off	enable_length_byte;	  /* should be used in combination with sync, only */
	enum address_filtering	enable_address_filtering; /* operational with sync, only */
	enum option_on_off	enable_crc;		  /* only operational, if sync on and fixed length or length byte is used */

	__u8			sync_length;
	__u8			fixed_message_length;
	__u32			bytes_to_drop;

	__u8			sync_pattern[8];
	__u8			node_address;
	__u8			broadcast_address;
};

#define rfm69_IOC_MAGIC			'r'

#define rfm69_IOC_RD_TX_CFG	_IOR(rfm69_IOC_MAGIC, rfm69_TX_CFG_IOCTL_NR, char[sizeof(struct rfm69_tx_cfg)])
#define rfm69_IOC_WR_TX_CFG	_IOW(rfm69_IOC_MAGIC, rfm69_TX_CFG_IOCTL_NR, char[sizeof(struct rfm69_tx_cfg)])

#define rfm69_IOC_RD_RX_CFG	_IOR(rfm69_IOC_MAGIC, rfm69_RX_CFG_IOCTL_NR, char[sizeof(struct rfm69_rx_cfg)])
#define rfm69_IOC_WR_RX_CFG	_IOW(rfm69_IOC_MAGIC, rfm69_RX_CFG_IOCTL_NR, char[sizeof(struct rfm69_rx_cfg)])

#endif /* rfm69_H */
