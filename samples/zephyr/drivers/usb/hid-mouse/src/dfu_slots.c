/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Two-slot USB DFU backend built directly on the flash map.
 *
 * The in-tree backend (CONFIG_USBD_DFU_FLASH) writes through flash_img_*,
 * which resolves to a single compile-time area ID, so the alternate setting
 * selected by the host is ignored on download. This backend uses flash_area_*
 * per image instead, so each alternate setting gets its own base and size
 * straight from its partition:
 *
 *   alt 0 -> dfu_slot_a
 *   alt 1 -> dfu_slot_b
 *
 * The alternate setting number is the position of the image in the linker
 * section, which is sorted by the first USBD_DFU_DEFINE_IMG() argument, hence
 * the "a"/"b" suffixes.
 */

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_dfu.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dfu_slots, LOG_LEVEL_INF);

struct dfu_slot_data {
	const uint8_t area_id;
	uint32_t last_block;
	/* Bytes read or written so far, relative to the partition start. */
	uint32_t offset;
	/* Bytes erased so far, relative to the partition start. */
	uint32_t erased;
};

static struct dfu_slot_data slot_a_data = {
	.area_id = PARTITION_ID(dfu_slot_a),
};

static struct dfu_slot_data slot_b_data = {
	.area_id = PARTITION_ID(dfu_slot_b),
};

/* Erase pages lazily, up to the byte the current download needs. */
static int slot_erase_upto(const struct flash_area *const fa,
			   struct dfu_slot_data *const data, const uint32_t needed)
{
	while (data->erased < needed) {
		struct flash_pages_info info;
		int ret;

		ret = flash_get_page_info_by_offs(fa->fa_dev, fa->fa_off + data->erased,
						  &info);
		if (ret) {
			return ret;
		}

		ret = flash_area_erase(fa, data->erased, info.size);
		if (ret) {
			return ret;
		}

		data->erased += info.size;
	}

	return 0;
}

static int slot_read(void *const priv, const uint32_t block, const uint16_t size,
		     uint8_t buf[static CONFIG_USBD_DFU_TRANSFER_SIZE])
{
	struct dfu_slot_data *const data = priv;
	const struct flash_area *fa;
	uint32_t len;
	int ret;

	if (size == 0) {
		/* There is nothing to upload */
		return 0;
	}

	if (block == 0) {
		data->last_block = 0;
		data->offset = 0;
	} else if (data->last_block + 1U != block) {
		return -EINVAL;
	}

	ret = flash_area_open(data->area_id, &fa);
	if (ret) {
		return ret;
	}

	len = MIN(size, fa->fa_size - data->offset);
	if (len == 0) {
		/* Partition fully uploaded, a short packet ends the transfer */
		flash_area_close(fa);
		return 0;
	}

	ret = flash_area_read(fa, data->offset, buf, len);
	flash_area_close(fa);
	if (ret) {
		LOG_ERR("Failed to read area %u at 0x%x (%d)", data->area_id,
			data->offset, ret);
		return ret;
	}

	data->last_block = block;
	data->offset += len;
	LOG_DBG("area %u block %u uploaded %u", data->area_id, block, data->offset);

	return len;
}

static int slot_write(void *const priv, const uint32_t block, const uint16_t size,
		      const uint8_t buf[static CONFIG_USBD_DFU_TRANSFER_SIZE])
{
	struct dfu_slot_data *const data = priv;
	const struct flash_area *fa;
	int ret;

	if (block == 0) {
		data->last_block = 0;
		data->offset = 0;
		data->erased = 0;
	} else if (data->last_block + 1U != block) {
		return -EINVAL;
	}

	if (size == 0) {
		/* Zero length download ends the transfer, nothing is buffered */
		LOG_INF("area %u download finished, %u bytes", data->area_id,
			data->offset);
		return 0;
	}

	ret = flash_area_open(data->area_id, &fa);
	if (ret) {
		return ret;
	}

	if (data->offset + size > fa->fa_size) {
		LOG_ERR("Image does not fit into area %u (%u bytes)", data->area_id,
			fa->fa_size);
		flash_area_close(fa);
		return -ENOSPC;
	}

	ret = slot_erase_upto(fa, data, data->offset + size);
	if (ret) {
		LOG_ERR("Failed to erase area %u (%d)", data->area_id, ret);
		flash_area_close(fa);
		return ret;
	}

	/* Both Bee flash drivers advertise write-block-size 1, so a short last
	 * block needs no padding.
	 */
	ret = flash_area_write(fa, data->offset, buf, size);
	flash_area_close(fa);
	if (ret) {
		LOG_ERR("Failed to write area %u at 0x%x (%d)", data->area_id,
			data->offset, ret);
		return ret;
	}

	data->last_block = block;
	data->offset += size;
	LOG_DBG("area %u block %u downloaded %u", data->area_id, block, data->offset);

	return 0;
}

USBD_DFU_DEFINE_IMG(dfu_slot_a, "slot-a", &slot_a_data, slot_read, slot_write, NULL);
USBD_DFU_DEFINE_IMG(dfu_slot_b, "slot-b", &slot_b_data, slot_read, slot_write, NULL);
