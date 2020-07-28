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



#undef DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/idr.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "rfm69_if.h"
#include "rf69.h"

#define N_rfm69_MINORS		BIT(MINORBITS) /*32*/	/* ... up to 256 */
#define MAX_MSG_SIZE		900	/* min: FIFO_SIZE! */
#define MSG_FIFO_SIZE		65536   /* 65536 = 2^16  */
#define NUM_DIO			2

static dev_t rfm69_dev;
static DEFINE_IDR(rfm69_idr);
static DEFINE_MUTEX(minor_lock); /* Protect idr accesses */

static struct class *rfm69_class; /* mainly for udev to create /dev/rfm69 */

/*
 * tx config is instance specific
 * so with each open a new tx config struct is needed
 */
/*
 * rx config is device specific
 * so we have just one rx config, ebedded in device struct
 */
struct rfm69_device {
	/* device handling related values */
	dev_t			devt;
	int			minor;
	struct device		*dev;
	struct cdev		*cdev;
	struct spi_device	*spi;

	/* irq related values */
	struct gpio_desc	*gpiod[NUM_DIO];
	int			irq_num[NUM_DIO];
	u8			irq_state[NUM_DIO];

	/* tx related values */
	STRUCT_KFIFO_REC_1(MSG_FIFO_SIZE) tx_fifo;
	struct mutex		tx_fifo_lock; /* serialize userspace writers */
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	u8			free_in_fifo;
	char			buffer[MAX_MSG_SIZE];

	/* rx related values */
	struct rfm69_rx_cfg	rx_cfg;
	u8			*rx_buffer;
	unsigned int		rx_buffer_size;
	u32			rx_bytes_to_drop;
	u32			rx_bytes_dropped;
	unsigned int		rx_position;
	struct mutex		rx_lock;
	wait_queue_head_t	rx_wait_queue;

	/* fifo wait queue */
	struct task_struct	*fifo_task_struct;
	wait_queue_head_t	fifo_wait_queue;

	/* flags */
	bool			rx_active;
	bool			tx_active;
	bool			interrupt_rx_allowed;
};

struct rfm69_instance {
	struct rfm69_device	*device;
	struct rfm69_tx_cfg	tx_cfg;
};

/*-------------------------------------------------------------------------*/

/* GPIO interrupt handlers */
static irqreturn_t DIO0_irq_handler(int irq, void *dev_id)
{
	struct rfm69_device *device = dev_id;

	if (device->irq_state[DIO0] == DIO_PACKET_SENT) {
		device->free_in_fifo = FIFO_SIZE;
		dev_dbg(device->dev, "DIO0 irq: Packet sent\n");
		wake_up_interruptible(&device->fifo_wait_queue);
	} else if (device->irq_state[DIO0] == DIO_RSSI_DIO0) {
		dev_dbg(device->dev, "DIO0 irq: RSSI level over threshold\n");
		wake_up_interruptible(&device->rx_wait_queue);
	} else if (device->irq_state[DIO0] == DIO_PAYLOAD_READY) {
		dev_dbg(device->dev, "DIO0 irq: Payload ready\n");
		device->free_in_fifo = 0;
		wake_up_interruptible(&device->fifo_wait_queue);
	}

	return IRQ_HANDLED;
}

static irqreturn_t DIO1_irq_handler(int irq, void *dev_id)
{
	struct rfm69_device *device = dev_id;

	if (device->irq_state[DIO1] == DIO_FIFO_NOT_EMPTY_DIO1) {
		device->free_in_fifo = FIFO_SIZE;
	} else if (device->irq_state[DIO1] == DIO_FIFO_LEVEL) {
		if (device->rx_active)
			device->free_in_fifo = FIFO_THRESHOLD - 1;
		else
			device->free_in_fifo = FIFO_SIZE - FIFO_THRESHOLD - 1;
	}
	dev_dbg(device->dev,
		"DIO1 irq: %d bytes free in fifo\n", device->free_in_fifo);
	wake_up_interruptible(&device->fifo_wait_queue);

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static int
rf69_set_rx_cfg(struct rfm69_device *dev, struct rfm69_rx_cfg *rx_cfg)
{
	int ret;
	int payload_length;

	/* receiver config */
	ret = rf69_set_frequency(dev->spi, rx_cfg->frequency);
	if (ret < 0)
		return ret;
	ret = rf69_set_bit_rate(dev->spi, rx_cfg->bit_rate);
	if (ret < 0)
		return ret;
	ret = rf69_set_modulation(dev->spi, rx_cfg->modulation);
	if (ret < 0)
		return ret;
	ret = rf69_set_antenna_impedance(dev->spi, rx_cfg->antenna_impedance);
	if (ret < 0)
		return ret;
	ret = rf69_set_rssi_threshold(dev->spi, rx_cfg->rssi_threshold);
	if (ret < 0)
		return ret;
	ret = rf69_set_ook_threshold_dec(dev->spi, rx_cfg->threshold_decrement);
	if (ret < 0)
		return ret;
	ret = rf69_set_bandwidth(dev->spi, rx_cfg->bw_mantisse,
				 rx_cfg->bw_exponent);
	if (ret < 0)
		return ret;
	ret = rf69_set_bandwidth_during_afc(dev->spi, rx_cfg->bw_mantisse,
					    rx_cfg->bw_exponent);
	if (ret < 0)
		return ret;
	ret = rf69_set_dagc(dev->spi, rx_cfg->dagc);
	if (ret < 0)
		return ret;

	dev->rx_bytes_to_drop = rx_cfg->bytes_to_drop;

	/* packet config */
	/* enable */
	if (rx_cfg->enable_sync == OPTION_ON) {
		ret = rf69_enable_sync(dev->spi);
		if (ret < 0)
			return ret;

		ret = rf69_set_fifo_fill_condition(dev->spi,
						   after_sync_interrupt);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_disable_sync(dev->spi);
		if (ret < 0)
			return ret;

		ret = rf69_set_fifo_fill_condition(dev->spi, always);
		if (ret < 0)
			return ret;
	}
	if (rx_cfg->enable_length_byte == OPTION_ON) {
		ret = rf69_set_packet_format(dev->spi, packet_length_var);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_set_packet_format(dev->spi, packet_length_fix);
		if (ret < 0)
			return ret;
	}
	ret = rf69_set_address_filtering(dev->spi,
					 rx_cfg->enable_address_filtering);
	if (ret < 0)
		return ret;

	if (rx_cfg->enable_crc == OPTION_ON) {
		ret = rf69_enable_crc(dev->spi);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_disable_crc(dev->spi);
		if (ret < 0)
			return ret;
	}

	/* lengths */
	ret = rf69_set_sync_size(dev->spi, rx_cfg->sync_length);
	if (ret < 0)
		return ret;
	if (rx_cfg->enable_length_byte == OPTION_ON) {
		ret = rf69_set_payload_length(dev->spi, 0xff);
		if (ret < 0)
			return ret;
	} else if (rx_cfg->fixed_message_length != 0) {
		payload_length = rx_cfg->fixed_message_length;
		if (rx_cfg->enable_length_byte  == OPTION_ON)
			payload_length++;
		if (rx_cfg->enable_address_filtering != filtering_off)
			payload_length++;
		ret = rf69_set_payload_length(dev->spi, payload_length);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_set_payload_length(dev->spi, 0);
		if (ret < 0)
			return ret;
	}

	/* values */
	if (rx_cfg->enable_sync == OPTION_ON) {
		ret = rf69_set_sync_values(dev->spi, rx_cfg->sync_pattern);
		if (ret < 0)
			return ret;
	}
	if (rx_cfg->enable_address_filtering != filtering_off) {
		ret = rf69_set_node_address(dev->spi, rx_cfg->node_address);
		if (ret < 0)
			return ret;
		ret = rf69_set_broadcast_address(dev->spi,
						 rx_cfg->broadcast_address);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int
rf69_set_tx_cfg(struct rfm69_device *dev, struct rfm69_tx_cfg *tx_cfg)
{
	int ret;

	ret = rf69_set_frequency(dev->spi, tx_cfg->frequency);
	if (ret < 0)
		return ret;
	ret = rf69_set_bit_rate(dev->spi, tx_cfg->bit_rate);
	if (ret < 0)
		return ret;
	ret = rf69_set_modulation(dev->spi, tx_cfg->modulation);
	if (ret < 0)
		return ret;
	ret = rf69_set_deviation(dev->spi, tx_cfg->dev_frequency);
	if (ret < 0)
		return ret;
	ret = rf69_set_pa_ramp(dev->spi, tx_cfg->pa_ramp);
	if (ret < 0)
		return ret;
	ret = rf69_set_modulation_shaping(dev->spi, tx_cfg->mod_shaping);
	if (ret < 0)
		return ret;
	ret = rf69_set_tx_start_condition(dev->spi, tx_cfg->tx_start_condition);
	if (ret < 0)
		return ret;

	/* packet format enable */
	if (tx_cfg->enable_preamble == OPTION_ON) {
		ret = rf69_set_preamble_length(dev->spi,
					       tx_cfg->preamble_length);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_set_preamble_length(dev->spi, 0);
		if (ret < 0)
			return ret;
	}

	if (tx_cfg->enable_sync == OPTION_ON) {
		ret = rf69_set_sync_size(dev->spi, tx_cfg->sync_length);
		if (ret < 0)
			return ret;
		ret = rf69_set_sync_values(dev->spi, tx_cfg->sync_pattern);
		if (ret < 0)
			return ret;
		ret = rf69_enable_sync(dev->spi);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_disable_sync(dev->spi);
		if (ret < 0)
			return ret;
	}

	if (tx_cfg->enable_length_byte == OPTION_ON) {
		ret = rf69_set_packet_format(dev->spi, packet_length_var);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_set_packet_format(dev->spi, packet_length_fix);
		if (ret < 0)
			return ret;
	}

	if (tx_cfg->enable_crc == OPTION_ON) {
		ret = rf69_enable_crc(dev->spi);
		if (ret < 0)
			return ret;
	} else {
		ret = rf69_disable_crc(dev->spi);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static int
rfm69_start_rx(struct rfm69_device *dev)
{
	int retval;

	/* return without action, if no pending read request */
	if (!dev->rx_active)
		return 0;

	/* setup for receiving */
	retval = rf69_set_rx_cfg(dev, &dev->rx_cfg);
	if (retval)
		return retval;

	/* setup rssi irq */
	retval = rf69_set_dio_mapping(dev->spi, DIO0, DIO_RSSI_DIO0);
	if (retval < 0)
		return retval;
	dev->irq_state[DIO0] = DIO_RSSI_DIO0;
	irq_set_irq_type(dev->irq_num[DIO0], IRQ_TYPE_EDGE_RISING);

	/* setup fifo level interrupt */
	retval = rf69_set_fifo_threshold(dev->spi, FIFO_SIZE - FIFO_THRESHOLD);
	if (retval < 0)
		return retval;
	retval = rf69_set_dio_mapping(dev->spi, DIO1, DIO_FIFO_LEVEL);
	if (retval < 0)
		return retval;
	dev->irq_state[DIO1] = DIO_FIFO_LEVEL;
	irq_set_irq_type(dev->irq_num[DIO1], IRQ_TYPE_EDGE_RISING);

	/* set module to receiving mode */
	retval = rf69_set_mode(dev->spi, receive);
	if (retval < 0)
		return retval;

	return 0;
}

/*-------------------------------------------------------------------------*/

static int
rfm69_receive(void *data)
{
	struct rfm69_device *dev = data;
	struct spi_device *spi = dev->spi;
	int bytes_to_read, bytes_total;
	int retval;

	dev->interrupt_rx_allowed = false;

	/* wait for any tx to finish */
	dev_dbg(dev->dev, "rx: going to wait for any tx to finish");
	retval = wait_event_interruptible(dev->rx_wait_queue, !dev->tx_active);
	if (retval) {
		/* wait was interrupted */
		dev->interrupt_rx_allowed = true;
		wake_up_interruptible(&dev->tx_wait_queue);
		return retval;
	}

	/* prepare status vars */
	dev->free_in_fifo = FIFO_SIZE;
	dev->rx_position = 0;
	dev->rx_bytes_dropped = 0;

	/* setup radio module to listen for something "in the air" */
	retval = rfm69_start_rx(dev);
	if (retval)
		return retval;

	/* now check RSSI, if low wait for getting high (RSSI interrupt) */
	while (!rf69_get_flag(dev->spi, rssi_exceeded_threshold)) {
		/* allow tx to interrupt us while waiting for high RSSI */
		dev->interrupt_rx_allowed = true;
		wake_up_interruptible(&dev->tx_wait_queue);

		/* wait for RSSI level to become high */
		dev_dbg(dev->dev, "rx: going to wait for high RSSI level");
		retval = wait_event_interruptible(dev->rx_wait_queue,
						  rf69_get_flag(dev->spi,
								rssi_exceeded_threshold));
		if (retval) /* wait was interrupted */
			goto abort;
		dev->interrupt_rx_allowed = false;

		/* cross check for ongoing tx */
		if (!dev->tx_active)
			break;
	}

	/* configure payload ready irq */
	retval = rf69_set_dio_mapping(spi, DIO0, DIO_PAYLOAD_READY);
	if (retval < 0)
		goto abort;
	dev->irq_state[DIO0] = DIO_PAYLOAD_READY;
	irq_set_irq_type(dev->irq_num[DIO0], IRQ_TYPE_EDGE_RISING);

	/* fixed or unlimited length? */
	if (dev->rx_cfg.fixed_message_length != 0) {
		if (dev->rx_cfg.fixed_message_length > dev->rx_buffer_size) {
			retval = -1;
			goto abort;
		}
		bytes_total = dev->rx_cfg.fixed_message_length;
		dev_dbg(dev->dev, "rx: msg len set to %d by fixed length",
			bytes_total);
	} else {
		bytes_total = dev->rx_buffer_size;
		dev_dbg(dev->dev, "rx: msg len set to %d as requested by read",
			bytes_total);
	}

	/* length byte enabled? */
	if (dev->rx_cfg.enable_length_byte == OPTION_ON) {
		retval = wait_event_interruptible(dev->fifo_wait_queue,
						  dev->free_in_fifo < FIFO_SIZE);
		if (retval) /* wait was interrupted */
			goto abort;

		rf69_read_fifo(spi, (u8 *)&bytes_total, 1);
		if (bytes_total > dev->rx_buffer_size) {
			retval = -1;
			goto abort;
		}
		dev->free_in_fifo++;
		dev_dbg(dev->dev, "rx: msg len reset to %d due to length byte",
			bytes_total);
	}

	/* address byte enabled? */
	if (dev->rx_cfg.enable_address_filtering != filtering_off) {
		u8 dummy;

		bytes_total--;

		retval = wait_event_interruptible(dev->fifo_wait_queue,
						  dev->free_in_fifo < FIFO_SIZE);
		if (retval) /* wait was interrupted */
			goto abort;

		rf69_read_fifo(spi, &dummy, 1);
		dev->free_in_fifo++;
		dev_dbg(dev->dev, "rx: address byte stripped off");
	}

	/* get payload */
	while (dev->rx_position < bytes_total) {
		if (!rf69_get_flag(dev->spi, payload_ready)) {
			retval = wait_event_interruptible(dev->fifo_wait_queue,
							  dev->free_in_fifo < FIFO_SIZE);
			if (retval) /* wait was interrupted */
				goto abort;
		}

		/* need to drop bytes or acquire? */
		if (dev->rx_bytes_to_drop > dev->rx_bytes_dropped)
			bytes_to_read = dev->rx_bytes_to_drop -
					dev->rx_bytes_dropped;
		else
			bytes_to_read = bytes_total - dev->rx_position;

		/* access the fifo */
		if (bytes_to_read > FIFO_SIZE - dev->free_in_fifo)
			bytes_to_read = FIFO_SIZE - dev->free_in_fifo;
		retval = rf69_read_fifo(spi,
					&dev->rx_buffer[dev->rx_position],
					bytes_to_read);
		if (retval) /* read failed */
			goto abort;

		dev->free_in_fifo += bytes_to_read;

		/* adjust status vars */
		if (dev->rx_bytes_to_drop > dev->rx_bytes_dropped)
			dev->rx_bytes_dropped += bytes_to_read;
		else
			dev->rx_position += bytes_to_read;
	}

	/* rx done, wait was interrupted or error occurred */
abort:
	dev->interrupt_rx_allowed = true;
	if (rf69_set_mode(dev->spi, standby))
		pr_err("rf69_set_mode(): radio module failed to go standby\n");
	wake_up_interruptible(&dev->tx_wait_queue);

	if (retval)
		return retval;
	else
		return bytes_total;
}

static int
rfm69_tx_thread(void *data)
{
	struct rfm69_device *device = data;
	struct spi_device *spi = device->spi;
	struct rfm69_tx_cfg tx_cfg;
	size_t size;
	bool   rx_interrupted = false;
	int    position, repetitions;
	int    retval;

	while (1) {
		/* wait for fifo to be populated or for request to terminate*/
		dev_dbg(device->dev, "thread: going to wait for new messages");
		wait_event_interruptible(device->tx_wait_queue,
					 (!kfifo_is_empty(&device->tx_fifo) ||
					  kthread_should_stop()));
		if (kthread_should_stop())
			return 0;

		/*
		 * get data from fifo in the following order:
		 * - tx_cfg
		 * - size of message
		 * - message
		 */
		retval = kfifo_out(&device->tx_fifo, &tx_cfg, sizeof(tx_cfg));
		if (retval != sizeof(tx_cfg)) {
			dev_dbg(device->dev,
				"reading tx_cfg from fifo failed: got %d byte(s), expected %d",
				retval, (unsigned int)sizeof(tx_cfg));
			continue;
		}

		retval = kfifo_out(&device->tx_fifo, &size, sizeof(size_t));
		if (retval != sizeof(size_t)) {
			dev_dbg(device->dev,
				"reading msg size from fifo failed: got %d, expected %d",
				retval, (unsigned int)sizeof(size_t));
			continue;
		}

		/* use fixed message length, if requested */
		if (tx_cfg.fixed_message_length != 0)
			size = tx_cfg.fixed_message_length;

		/* increase size, if len byte is requested */
		if (tx_cfg.enable_length_byte == OPTION_ON)
			size++;

		/* increase size, if adr byte is requested */
		if (tx_cfg.enable_address_byte == OPTION_ON)
			size++;

		/* prime buffer */
		memset(device->buffer, 0, size);
		position = 0;

		/* add length byte, if requested */
		if (tx_cfg.enable_length_byte  == OPTION_ON)
			/*
			 * according to spec, length byte itself must be
			 * excluded from the length calculation
			 */
			device->buffer[position++] = size - 1;

		/* add adr byte, if requested */
		if (tx_cfg.enable_address_byte == OPTION_ON)
			device->buffer[position++] = tx_cfg.address_byte;

		/* finally get message data from fifo */
		retval = kfifo_out(&device->tx_fifo, &device->buffer[position],
				   sizeof(device->buffer) - position);
		dev_dbg(device->dev,
			"read %d message byte(s) from fifo queue.", retval);

		/*
		 * if rx is active, we need to interrupt the waiting for
		 * incoming telegrams, to be able to send something.
		 * We are only allowed, if currently no reception takes
		 * place otherwise we need to  wait for the incoming telegram
		 * to finish
		 */
		wait_event_interruptible(device->tx_wait_queue,
					 !device->rx_active ||
					  device->interrupt_rx_allowed);

		/*
		 * prevent race conditions
		 * irq will be reenabled after tx config is set
		 */
		disable_irq(device->irq_num[DIO0]);
		device->tx_active = true;

		/* clear fifo, set fifo threshold, set payload length */
		retval = rf69_set_mode(spi, standby); /* this clears the fifo */
		if (retval < 0)
			return retval;

		if (device->rx_active && !rx_interrupted) {
			/*
			 * rx is currently waiting for a telegram;
			 * we need to set the radio module to standby
			 */
			rx_interrupted = true;
		}

		retval = rf69_set_fifo_threshold(spi, FIFO_THRESHOLD);
		if (retval < 0)
			return retval;
		if (tx_cfg.enable_length_byte == OPTION_ON) {
			retval = rf69_set_payload_length(spi, size * tx_cfg.repetitions);
			if (retval < 0)
				return retval;
		} else {
			retval = rf69_set_payload_length(spi, 0);
			if (retval < 0)
				return retval;
		}

		/* configure the rf chip */
		retval = rf69_set_tx_cfg(device, &tx_cfg);
		if (retval < 0)
			return retval;

		/* enable fifo level interrupt */
		retval = rf69_set_dio_mapping(spi, DIO1, DIO_FIFO_LEVEL);
		if (retval < 0)
			return retval;
		device->irq_state[DIO1] = DIO_FIFO_LEVEL;
		irq_set_irq_type(device->irq_num[DIO1], IRQ_TYPE_EDGE_FALLING);

		/* enable packet sent interrupt */
		retval = rf69_set_dio_mapping(spi, DIO0, DIO_PACKET_SENT);
		if (retval < 0)
			return retval;
		device->irq_state[DIO0] = DIO_PACKET_SENT;
		irq_set_irq_type(device->irq_num[DIO0], IRQ_TYPE_EDGE_RISING);
		enable_irq(device->irq_num[DIO0]); /* was disabled by rx active check */

		/* enable transmission */
		retval = rf69_set_mode(spi, transmit);
		if (retval < 0)
			return retval;

		/* transfer this msg (and repetitions) to chip fifo */
		device->free_in_fifo = FIFO_SIZE;
		position = 0;
		repetitions = tx_cfg.repetitions;
		while ((repetitions > 0) && (size > position)) {
			if ((size - position) > device->free_in_fifo) {
				/* msg to big for fifo - take a part */
				int write_size = device->free_in_fifo;

				device->free_in_fifo = 0;
				rf69_write_fifo(spi,
						&device->buffer[position],
						write_size);
				position += write_size;
			} else {
				/* msg fits into fifo - take all */
				device->free_in_fifo -= size;
				repetitions--;
				rf69_write_fifo(spi,
						&device->buffer[position],
						(size - position));
				position = 0; /* reset for next repetition */
			}

			retval = wait_event_interruptible(device->fifo_wait_queue,
							  device->free_in_fifo > 0);
			if (retval) {
				dev_dbg(device->dev, "ABORT\n");
				goto abort;
			}
		}

		/* we are done. Wait for packet to get sent */
		dev_dbg(device->dev,
			"thread: wait for packet to get sent/fifo to be empty");
		wait_event_interruptible(device->fifo_wait_queue,
					 device->free_in_fifo == FIFO_SIZE ||
					 kthread_should_stop());
		if (kthread_should_stop())
			return 0;

		/* STOP_TRANSMISSION */
		dev_dbg(device->dev, "thread: Packet sent. Set mode to stby.");
		retval = rf69_set_mode(spi, standby);
		if (retval < 0)
			return retval;

		/* everything sent? */
		if (kfifo_is_empty(&device->tx_fifo)) {
abort:
			if (rx_interrupted) {
				rx_interrupted = false;
				rfm69_start_rx(device);
			}
			device->tx_active = false;
			wake_up_interruptible(&device->rx_wait_queue);
		}
	}
}

/*-------------------------------------------------------------------------*/

static ssize_t
rfm69_read(struct file *filp, char __user *buf, size_t size, loff_t *f_pos)
{
	struct rfm69_instance	*instance;
	struct rfm69_device	*device;
	int			bytes_received;
	ssize_t			retval;

	/* check, whether internal buffer is big enough for requested size */
	if (size > MAX_MSG_SIZE)
		return -EMSGSIZE;

	instance = filp->private_data;
	device = instance->device;

	/* just one read request at a time */
	mutex_lock(&device->rx_lock);
	if (device->rx_active) {
		mutex_unlock(&device->rx_lock);
		return -EAGAIN;
	}

	device->rx_active = true;
	mutex_unlock(&device->rx_lock);

	/* start receiving */
	/* will block until something was received*/
	device->rx_buffer_size = size;
	bytes_received = rfm69_receive(device);

	/* release rx */
	mutex_lock(&device->rx_lock);
	device->rx_active = false;
	mutex_unlock(&device->rx_lock);

	/* if read was successful copy to user space*/
	if (bytes_received > 0) {
		retval = copy_to_user(buf, device->rx_buffer, bytes_received);
		if (retval)
			return -EFAULT;
	}

	return bytes_received;
}

static ssize_t
rfm69_write(struct file *filp, const char __user *buf,
	    size_t count, loff_t *f_pos)
{
	struct rfm69_instance	*instance;
	struct rfm69_device	*device;
	int                     retval;
	unsigned int		required, available, copied;

	instance = filp->private_data;
	device = instance->device;

	/*
	 * check, whether internal buffer (tx thread) is big enough
	 * for requested size
	 */
	if (count > MAX_MSG_SIZE)
		return -EMSGSIZE;

	/*
	 * write the following sequence into fifo:
	 * - tx_cfg
	 * - size of message
	 * - message
	 */
	mutex_lock(&device->tx_fifo_lock);

	required = sizeof(instance->tx_cfg) + sizeof(size_t) + count;
	available = kfifo_avail(&device->tx_fifo);
	if (required > available) {
		dev_dbg(device->dev, "write to fifo failed: %d bytes required but %d available",
			required, available);
		mutex_unlock(&device->tx_fifo_lock);
		return -EAGAIN;
	}

	retval = kfifo_in(&device->tx_fifo, &instance->tx_cfg,
			  sizeof(instance->tx_cfg));
	if (retval != sizeof(instance->tx_cfg))
		goto abort;

	retval = kfifo_in(&device->tx_fifo, &count, sizeof(size_t));
	if (retval != sizeof(size_t))
		goto abort;

	retval = kfifo_from_user(&device->tx_fifo, buf, count, &copied);
	if (retval || copied != count)
		goto abort;

	mutex_unlock(&device->tx_fifo_lock);

	/* start transfer */
	wake_up_interruptible(&device->tx_wait_queue);
	dev_dbg(device->dev, "write: generated new msg with %d bytes.", copied);

	return copied;

abort:
	dev_warn(device->dev,
		 "write to fifo failed, non recoverable: 0x%x", retval);
	mutex_unlock(&device->tx_fifo_lock);
	return -EAGAIN;
}

static long
rfm69_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rfm69_instance	*instance;
	struct rfm69_device	*device;
	struct rfm69_tx_cfg	tx_cfg;
	void __user *argp = (void __user *)arg;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != rfm69_IOC_MAGIC)
		return -ENOTTY;

	instance = filp->private_data;
	device = instance->device;

	if (!device)
		return -ESHUTDOWN;

	switch (cmd) {
	case rfm69_IOC_RD_TX_CFG:
		if (copy_to_user(argp, &instance->tx_cfg,
				 sizeof(struct rfm69_tx_cfg)))
			return -EFAULT;
		break;
	case rfm69_IOC_WR_TX_CFG:
		if (copy_from_user(&tx_cfg, argp, sizeof(struct rfm69_tx_cfg)))
			return -EFAULT;
		mutex_lock(&device->tx_fifo_lock);
		memcpy(&instance->tx_cfg, &tx_cfg, sizeof(struct rfm69_tx_cfg));
		mutex_unlock(&device->tx_fifo_lock);
		break;
	case rfm69_IOC_RD_RX_CFG:
		if (copy_to_user(argp, &device->rx_cfg,
				 sizeof(struct rfm69_rx_cfg)))
			return -EFAULT;
		break;
	case rfm69_IOC_WR_RX_CFG:
		mutex_lock(&device->rx_lock);

		/* during pendig read request, change of config not allowed */
		if (device->rx_active) {
			mutex_unlock(&device->rx_lock);
			return -EAGAIN;
		}

		if (copy_from_user(&device->rx_cfg, argp,
				   sizeof(struct rfm69_rx_cfg))) {
			mutex_unlock(&device->rx_lock);
			return -EFAULT;
		}

		mutex_unlock(&device->rx_lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static int rfm69_open(struct inode *inode, struct file *filp)
{
	struct rfm69_device	*device;
	struct rfm69_instance	*instance;

	mutex_lock(&minor_lock);
	device = idr_find(&rfm69_idr, iminor(inode));
	mutex_unlock(&minor_lock);
	if (!device) {
		pr_debug("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance)
		return -ENOMEM;

	/* setup instance data*/
	instance->device = device;
	instance->tx_cfg.bit_rate = 4711;
	// TODO: fill instance->tx_cfg;

	/* instance data as context */
	filp->private_data = instance;
	stream_open(inode, filp);

	return 0;
}

static int rfm69_release(struct inode *inode, struct file *filp)
{
	struct rfm69_instance	*instance;

	instance = filp->private_data;
	kfree(instance);
	filp->private_data = NULL;

	return 0;
}

/*-------------------------------------------------------------------------*/

static int setup_gpio(struct rfm69_device *device)
{
	char	name[5];
	int	retval;
	int	i;
	const irq_handler_t DIO_irq_handler[NUM_DIO] = {
		DIO0_irq_handler,
		DIO1_irq_handler
	};

	for (i = 0; i < NUM_DIO; i++) {
		/* "construct" name and get the gpio descriptor */
		snprintf(name, sizeof(name), "DIO%d", i);
		device->gpiod[i] = gpiod_get(&device->spi->dev, name,
					     0 /*GPIOD_IN*/);

		if (device->gpiod[i] == ERR_PTR(-ENOENT)) {
			dev_dbg(&device->spi->dev,
				"Could not find entry for %s. Ignoring.", name);
			continue;
		}

		if (device->gpiod[i] == ERR_PTR(-EBUSY))
			dev_dbg(&device->spi->dev, "%s is busy.", name);

		if (IS_ERR(device->gpiod[i])) {
			retval = PTR_ERR(device->gpiod[i]);
			/* release already allocated gpios */
			for (i--; i >= 0; i--) {
				free_irq(device->irq_num[i], device);
				gpiod_put(device->gpiod[i]);
			}
			return retval;
		}

		/* configure the pin */
		gpiod_unexport(device->gpiod[i]);
		retval = gpiod_direction_input(device->gpiod[i]);
		if (retval)
			return retval;

		/* configure irq */
		device->irq_num[i] = gpiod_to_irq(device->gpiod[i]);
		if (device->irq_num[i] < 0) {
			device->gpiod[i] = ERR_PTR(-EINVAL);
			return device->irq_num[i];
		}
		retval = request_irq(device->irq_num[i],
				     DIO_irq_handler[i],
				     0, /* flags */
				     name,
				     device);

		if (retval)
			return retval;

		dev_dbg(&device->spi->dev, "%s successfully configured", name);
	}

	return 0;
}

static void free_gpio(struct rfm69_device *device)
{
	int i;

	for (i = 0; i < NUM_DIO; i++) {
		/* check if gpiod is valid */
		if (IS_ERR(device->gpiod[i]))
			continue;

		free_irq(device->irq_num[i], device);
		gpiod_put(device->gpiod[i]);
	}
}

static int rfm69_get_minor(struct rfm69_device *device)
{
	int retval = -ENOMEM;

	mutex_lock(&minor_lock);
	retval = idr_alloc(&rfm69_idr, device, 0, N_rfm69_MINORS, GFP_KERNEL);
	if (retval >= 0) {
		device->minor = retval;
		retval = 0;
	} else if (retval == -ENOSPC) {
		dev_err(&device->spi->dev, "too many rfm69 devices\n");
		retval = -EINVAL;
	}
	mutex_unlock(&minor_lock);
	return retval;
}

static void rfm69_free_minor(struct rfm69_device *dev)
{
	mutex_lock(&minor_lock);
	idr_remove(&rfm69_idr, dev->minor);
	mutex_unlock(&minor_lock);
}

/*-------------------------------------------------------------------------*/

static const struct file_operations rfm69_fops = {
	.owner =	THIS_MODULE,
	/*
	 * REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	rfm69_write,
	.read =		rfm69_read,
	.unlocked_ioctl = rfm69_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open =		rfm69_open,
	.release =	rfm69_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

static int rfm69_probe(struct spi_device *spi)
{
	struct rfm69_device	*device;
	int			retval;

	/* setup spi parameters */
	spi->mode = 0x00;
	spi->bits_per_word = 8;
	/*
	 * spi->max_speed_hz = 10000000;
	 * 1MHz already set by device tree overlay
	 */

	retval = spi_setup(spi);
	if (retval) {
		dev_dbg(&spi->dev, "configuration of SPI interface failed!\n");
		return retval;
	}

	dev_dbg(&spi->dev,
		"spi interface setup: mode 0x%2x, %d bits per word, %dhz max speed",
		spi->mode, spi->bits_per_word, spi->max_speed_hz);

	/* Ping the chip by reading the version register */
	retval = spi_w8r8(spi, 0x10);
	if (retval < 0)
		return retval;

	switch (retval) {
	case 0x24:
		dev_dbg(&spi->dev, "found rfm69 (ver. 0x%x)", retval);
		break;
	default:
		dev_dbg(&spi->dev, "unknown chip version: 0x%x", retval);
		return -ENODEV;
	}

	/* Allocate driver data */
	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device)
		return -ENOMEM;

	/* Initialize the driver data */
	device->spi = spi;
	device->rx_active = false;
	device->tx_active = false;
	device->interrupt_rx_allowed = false;

	/* init rx buffer */
	device->rx_buffer = kmalloc(MAX_MSG_SIZE, GFP_KERNEL);
	if (!device->rx_buffer) {
		retval = -ENOMEM;
		goto RX_failed;
	}

	/* init wait queues */
	init_waitqueue_head(&device->tx_wait_queue);
	init_waitqueue_head(&device->rx_wait_queue);
	init_waitqueue_head(&device->fifo_wait_queue);

	/* init fifo */
	INIT_KFIFO(device->tx_fifo);

	/* init mutexes and locks */
	mutex_init(&device->tx_fifo_lock);
	mutex_init(&device->rx_lock);

	/* setup GPIO (including irq_handler) for the different DIOs */
	retval = setup_gpio(device);
	if (retval) {
		dev_dbg(&spi->dev, "setup of GPIOs failed");
		goto GPIO_failed;
	}

	/* setup the radio module */
	retval = rf69_set_mode(spi, standby);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_set_data_mode(spi, DATAMODUL_MODE_PACKET);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_enable_amplifier(spi, MASK_PALEVEL_PA0);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_disable_amplifier(spi, MASK_PALEVEL_PA1);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_disable_amplifier(spi, MASK_PALEVEL_PA2);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_set_output_power_level(spi, 13);
	if (retval < 0)
		goto minor_failed;
	retval = rf69_set_antenna_impedance(spi, fifty_ohm);
	if (retval < 0)
		goto minor_failed;

	/* determ minor number */
	retval = rfm69_get_minor(device);
	if (retval) {
		dev_dbg(&spi->dev, "get of minor number failed");
		goto minor_failed;
	}

	/* create device */
	device->devt = MKDEV(MAJOR(rfm69_dev), device->minor);
	device->dev = device_create(rfm69_class,
				    &spi->dev,
				    device->devt,
				    device,
				    "rfm69.%d",
				    device->minor);
	if (IS_ERR(device->dev)) {
		pr_err("rfm69: device register failed\n");
		retval = PTR_ERR(device->dev);
		goto device_create_failed;
	} else {
		dev_dbg(device->dev,
			"created device for major %d, minor %d\n",
			MAJOR(rfm69_dev),
			device->minor);
	}

	/* start tx thread */
	device->tx_task_struct = kthread_run(rfm69_tx_thread,
					     device,
					     "rfm69.%d_tx_task",
					     device->minor);
	if (IS_ERR(device->tx_task_struct)) {
		dev_dbg(device->dev, "start of send thread failed");
		retval = PTR_ERR(device->tx_task_struct);
		goto send_thread_failed;
	}

	/* create cdev */
	device->cdev = cdev_alloc();
	if (!device->cdev) {
		dev_dbg(device->dev, "allocation of cdev failed");
		retval = -ENOMEM;
		goto cdev_failed;
	}
	device->cdev->owner = THIS_MODULE;
	cdev_init(device->cdev, &rfm69_fops);
	retval = cdev_add(device->cdev, device->devt, 1);
	if (retval) {
		dev_dbg(device->dev, "register of cdev failed");
		goto del_cdev;
	}

	/* spi setup */
	spi_set_drvdata(spi, device);

	return 0;

del_cdev:
	cdev_del(device->cdev);
cdev_failed:
	kthread_stop(device->tx_task_struct);
send_thread_failed:
	device_destroy(rfm69_class, device->devt);
device_create_failed:
	rfm69_free_minor(device);
minor_failed:
	free_gpio(device);
GPIO_failed:
	kfree(device->rx_buffer);
RX_failed:
	kfree(device);

	return retval;
}

static int rfm69_remove(struct spi_device *spi)
{
	struct rfm69_device	*device = spi_get_drvdata(spi);

	/* free GPIOs */
	free_gpio(device);

	/* make sure ops on existing fds can abort cleanly */
	device->spi = NULL;

	kthread_stop(device->tx_task_struct);

	device_destroy(rfm69_class, device->devt);

	cdev_del(device->cdev);

	rfm69_free_minor(device);

	kfree(device->rx_buffer);
	kfree(device);

	return 0;
}

/*
	hoperf,rfm69cw-315		#315MHZ,missing DIO4 
	hoperf,frm69hcw-315		#315MHZ,High Power 
	hoperf,rfm69hw-315		#315MHZ,High Power
	hoperf,rfm69w-315		#315MHZ 
	hoperf,rfm69cw-433		#433MHZ,missing DIO4 
	hoperf,frm69hcw-433		#433MHZ,High Power
	hoperf,rfm69hw-433		#433MHZ,High Power
	hoperf,rfm69w-433		#433MHZ
	hoperf,rfm69cw-868		#868MHZ,missing DIO4
	hoperf,frm69hcw-868		#868MHZ,High Power
	hoperf,rfm69hw-868		#868MHZ,High Power
	hoperf,rfm69w-868		#868MHZ
	hoperf,rfm69cw-915		#915MHZ,missing DIO4
	hoperf,frm69hcw-915		#915MHZ,High Power
	hoperf,rfm69hw-915		#915MHZ,High Power
	hoperf,rfm69w-915		#915MHZ
*/


// list of chip varients
enum rf69_type {
	rfm69_cw_315,
	rfm69_cw_433,
	rfm69_cw_868,
	rfm69_cw_915,
	rfm69_hcw_315,
	rfm69_hcw_433,
	rfm69_hcw_868,
	rfm69_hcw_915,
	rfm69_hw_315,
	rfm69_hw_433,
	rfm69_hw_868,
	rfm69_hw_915,
	rfm69_w_315,
	rfm69_w_433,
	rfm69_w_868,
	rfm69_w_915,

};

// structure to provide rfm69 chip varient data to driver
struct chip_desc {

	u32 freq_min;
	u32 freq_base;
	u32 freq_max;
	u8 pins_io; // binary io pinmap e.g dio4 (of 0-5) missing on rfm59cw
};



// array of chip varients
static const struct chip_desc chips[] = {
	[rfm69cw_315] = {
		.freq_min = 290000000,
		.freq_base = 315000000,
		.freq_max = 340000000,
		.pins_io = 0x2F,
	},
	[rfm69cw_433] = {
		.freq_min = 424000000,
		.freq_base = 433000000,
		.freq_max = 510000000,
		.pins_io = 0x2F,
	},
	[rfm69cw_868] = {
		.freq_min = 862000000,
		.freq_base = 868000000,
		.freq_max = 890000000,
		.pins_io = 0x2F,
	},
	[rfm69cw_915] = {
		.freq_min = 890000000,
		.freq_base = 915000000,
		.freq_max = 1020000000,
		.pins_io = 0x2F,
	},
	[rfm69hcw_315] = {
		.freq_min = 290000000,
		.freq_base = 315000000,
		.freq_max = 340000000,
		.pins_io = 0x3F,
	},
	[rfm69hcw_433] = {
		.freq_min = 424000000,
		.freq_base = 433000000,
		.freq_max = 510000000,
		.pins_io = 0x3F,
	},
	[rfm69hcw_868] = {
		.freq_min = 862000000,
		.freq_base = 868000000,
		.freq_max = 890000000,
		.pins_io = 0x3F,
	},
	[rfm69hcw_915] = {
		.freq_min = 890000000,
		.freq_base = 915000000,
		.freq_max = 1020000000,
		.pins_io = 0x3F,
	},
	[rfm69hw_315] = {
		.freq_min = 290000000,
		.freq_base = 315000000,
		.freq_max = 340000000,
		.pins_io = 0x3F,
	},
	[rfm69hw_433] = {
		.freq_min = 424000000,
		.freq_base = 433000000,
		.freq_max = 510000000,
		.pins_io = 0x3F,
	},
	[rfm69hw_868] = {
		.freq_min = 862000000,
		.freq_base = 868000000,
		.freq_max = 890000000,
		.pins_io = 0x3F,
	},
	[rfm69hw_915] = {
		.freq_min = 890000000,
		.freq_base = 915000000,
		.freq_max = 1020000000,
		.pins_io = 0x3F,
	},
	[rfm69w_315] = {
		.freq_min = 290000000,
		.freq_base = 315000000,
		.freq_max = 340000000,
		.pins_io = 0x3F,
	},
	[rfm69w_433] = {
		.freq_min = 424000000,
		.freq_base = 433000000,
		.freq_max = 510000000,
		.pins_io = 0x3F,
	},
	[rfm69w_868] = {
		.freq_min = 862000000,
		.freq_base = 868000000,
		.freq_max = 890000000,
		.pins_io = 0x3F,
	},
	[rfm69w_915] = {
		.freq_min = 890000000,
		.freq_base = 915000000,
		.freq_max = 1020000000,
		.pins_io = 0x3F,
	},
}


static const struct platform_device_id rfm68_dt_ids[] = {
	{
		.name		= "rfm69cw-315",
		.driver_data	= rfm69cw_315,
	}, {
		.name		= "rfm69cw-433",
		.driver_data	= rfm69cw_433,
	}, {
		.name		= "rfm69cw-868",
		.driver_data	= rfm69cw_868,
	}, {
		.name		= "rfm69cw-915",
		.driver_data	= rfm69cw_915,
	}, {
		.name		= "rfm69hcw-315",
		.driver_data	= rfm69hcw_315,
	}, {
		.name		= "rfm69hcw-433",
		.driver_data	= rfm69hcw_433,
	}, {
		.name		= "rfm69hcw-868",
		.driver_data	= rfm69hcw_868,
	}, {
		.name		= "rfm69hcw-915",
		.driver_data	= rfm69hcw_915,
	}, {
		.name		= "rfm69hw-315",
		.driver_data	= rfm69hw_315,
	}, {
		.name		= "rfm69hw-433",
		.driver_data	= rfm69hw_433,
	}, {
		.name		= "rfm69hw-868",
		.driver_data	= rfm69hw_868,
	}, {
		.name		= "rfm69hw-915",
		.driver_data	= rfm69hw_915,
	}, {
		.name		= "rfm69w-315",
		.driver_data	= rfm69w_315,
	}, {
		.name		= "rfm69w-433",
		.driver_data	= rfm69w_433,
	}, {
		.name		= "rfm69w-868",
		.driver_data	= rfm69w_868,
	}, {
		.name		= "rfm69w-915",
		.driver_data	= rfm69w_915,
	},
	{ },
};



static const struct of_device_id rfm69_of_match[] = {

	{ .compatible = "hoperf,rfm69cw-315",.data = &chips[rfm69cw_315] },
	{ .compatible = "hoperf,rfm69cw-433",.data = &chips[rfm69cw_433] },
	{ .compatible = "hoperf,rfm69cw-868",.data = &chips[rfm69cw_868] },
	{ .compatible = "hoperf,rfm69hcw-915",.data = &chips[rfm69hcw_915] },
	{ .compatible = "hoperf,rfm69hcw-315",.data = &chips[rfm69hcw_315] },
	{ .compatible = "hoperf,rfm69hcw-433",.data = &chips[rfm69hcw_433] },
	{ .compatible = "hoperf,rfm69hcw-868",.data = &chips[rfm69hcw_868] },
	{ .compatible = "hoperf,rfm69hw-915",.data = &chips[rfm69hw_915] },
	{ .compatible = "hoperf,rfm69hw-315",.data = &chips[rfm69hw_315] },
	{ .compatible = "hoperf,rfm69hw-433",.data = &chips[rfm69hw_433] },
	{ .compatible = "hoperf,rfm69hw-868",.data = &chips[rfm69hw_868] },
	{ .compatible = "hoperf,rfm69w-915",.data = &chips[rfm69w_915] },
	{ .compatible = "hoperf,rfm69w-315",.data = &chips[rfm69w_315] },
	{ .compatible = "hoperf,rfm69w-433",.data = &chips[rfm69w_433] },
	{ .compatible = "hoperf,rfm69w-868",.data = &chips[rfm69w_868] },
	{ .compatible = "hoperf,rfm69w-915",.data = &chips[rfm69w_915] },
	
	{},
};

MODULE_DEVICE_TABLE(of, rfm69_dt_match);

static struct spi_driver rfm69_spi_driver = {
	.driver = {
		.name =		"rfm69xxx",
		.owner =	THIS_MODULE,
		.of_match_table = rfm69_of_match,
	},
	.probe =	rfm69_probe,
	.remove =	rfm69_remove,
	.id_table = rfm69_dt_ids

	/*
	 * NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init rfm69_init(void)
{
	int status;

	/*
	 * If MAX_MSG_SIZE is smaller then FIFO_SIZE, the driver won't
	 * work stable - risk of buffer overflow
	 */
	if (MAX_MSG_SIZE < FIFO_SIZE)
		return -EINVAL;

	/*
	 * Claim device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * Last, register the driver which manages those device numbers.
	 */
	status = alloc_chrdev_region(&rfm69_dev, 0, N_rfm69_MINORS, "rfm69");
	if (status < 0)
		return status;

	rfm69_class = class_create(THIS_MODULE, "rfm69");
	if (IS_ERR(rfm69_class)) {
		unregister_chrdev(MAJOR(rfm69_dev),
				  rfm69_spi_driver.driver.name);
		return PTR_ERR(rfm69_class);
	}

	status = spi_register_driver(&rfm69_spi_driver);
	if (status < 0) {
		class_destroy(rfm69_class);
		unregister_chrdev(MAJOR(rfm69_dev),
				  rfm69_spi_driver.driver.name);
	}

	return status;
}

module_init(rfm69_init);

static void __exit rfm69_exit(void)
{
	spi_unregister_driver(&rfm69_spi_driver);
	class_destroy(rfm69_class);
	unregister_chrdev(MAJOR(rfm69_dev), rfm69_spi_driver.driver.name);
}
module_exit(rfm69_exit);

MODULE_AUTHOR("Marcus Wolf, <linux@wolf-entwicklungen.de>");
MODULE_DESCRIPTION("Driver for rfm69");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:rfm69");
