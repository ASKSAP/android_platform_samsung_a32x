/**
 * f2fs_format_utils.c
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * Dual licensed under the GPL or LGPL version 2 licenses.
 */
#ifndef _LARGEFILE_SOURCE
#define _LARGEFILE_SOURCE
#endif
#ifndef _LARGEFILE64_SOURCE
#define _LARGEFILE64_SOURCE
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS 64
#endif

#include <f2fs_fs.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#ifndef ANDROID_WINDOWS_HOST
#include <sys/ioctl.h>
#endif
#include <sys/stat.h>
#include <fcntl.h>

#ifdef HAVE_LINUX_FS_H
#include <linux/fs.h>
#endif
#ifdef HAVE_LINUX_FALLOC_H
#include <linux/falloc.h>
#endif

#ifndef BLKDISCARD
#define BLKDISCARD	_IO(0x12,119)
#endif
#ifndef BLKSECDISCARD
#define BLKSECDISCARD	_IO(0x12,125)
#endif

#define SEC_MAGIC_START	(8192)

enum TRIM_STEP_MAGIC {
	TRIM_STEP_MAGIC_1 = 0,
	TRIM_STEP_MAGIC_2,
	TRIM_STEP_MAGIC_MAX,
};

static const char trim_magic_1[16] = "BEFORE_DISCARD";
static const char trim_magic_2[16] = "AFTER_DISCARD";

static int trim_update_magic(int step)
{
	u_int8_t *buf;
	int ret = 0;

	buf = calloc(F2FS_BLKSIZE, 1);
	if (!buf) {
		MSG(0, "\tError: Failed to alloc memory for magic!!!\n");
		return -1;
	}

	if (step == TRIM_STEP_MAGIC_1) {
		memcpy(buf, trim_magic_1, sizeof(trim_magic_1));
	} else if (step == TRIM_STEP_MAGIC_2) {
		memcpy(buf, trim_magic_2, sizeof(trim_magic_2));
	} else {
		MSG(0, "\tSkip update magic: step is wrong!!!\n");
		goto exit;
	}

	if (dev_write_block(buf, 0) < 0) {
		MSG(0, "\tError: Failed to write magic!!!\n");
		ret = -1;
		goto exit;
	}

	if (f2fs_fsync_device() < 0) {
		MSG(0, "\tError: Failed to fsync for magic!!!\n");
		ret = -1;
		goto exit;
	}
exit:
	free(buf);
	return ret;
}

static int trim_device(int i)
{
#ifndef ANDROID_WINDOWS_HOST
	unsigned long long range[2];
	struct stat *stat_buf;
	struct device_info *dev = c.devices + i;
	u_int64_t bytes = dev->total_sectors * dev->sector_size;
	int fd = dev->fd;

	stat_buf = malloc(sizeof(struct stat));
	if (stat_buf == NULL) {
		MSG(1, "\tError: Malloc Failed for trim_stat_buf!!!\n");
		return -1;
	}

	if (fstat(fd, stat_buf) < 0 ) {
		MSG(1, "\tError: Failed to get the device stat!!!\n");
		free(stat_buf);
		return -1;
	}

	range[0] = (i == 0) ? SEC_MAGIC_START : 0;
	range[1] = bytes - range[0];

#if defined(WITH_BLKDISCARD) && defined(BLKDISCARD)
	MSG(0, "Info: [%s] Discarding device\n", dev->path);
	if (S_ISREG(stat_buf->st_mode)) {
#if defined(HAVE_FALLOCATE) && defined(FALLOC_FL_PUNCH_HOLE)
		if (fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
				range[0], range[1]) < 0) {
			MSG(0, "Info: fallocate(PUNCH_HOLE|KEEP_SIZE) is failed\n");
		}
#endif
		free(stat_buf);
		return 0;
	} else if (S_ISBLK(stat_buf->st_mode)) {
		if (dev->zoned_model != F2FS_ZONED_NONE) {
			free(stat_buf);
			return f2fs_reset_zones(i);
		}
#ifdef BLKSECDISCARD
		if (ioctl(fd, BLKSECDISCARD, &range) < 0) {
			MSG(0, "Info: This device doesn't support BLKSECDISCARD\n");
		} else {
			MSG(0, "Info: Secure Discarded %lu MB\n",
					(unsigned long)stat_buf->st_size >> 20);
			free(stat_buf);
			return 0;
		}
#endif
		if (ioctl(fd, BLKDISCARD, &range) < 0) {
			MSG(0, "Info: This device doesn't support BLKDISCARD\n");
		} else {
			MSG(0, "Info: Discarded %llu MB\n", range[1] >> 20);
		}
	} else {
		free(stat_buf);
		return -1;
	}
#endif
	free(stat_buf);
#endif
	return 0;
}

static bool is_wiped_device(__attribute__((unused)) int i)
{
	return false;
/* We always want to trim device without condition */
#if 0
#ifdef WITH_ANDROID
	struct device_info *dev = c.devices + i;
	int fd = dev->fd;
	char *buf, *zero_buf;
	bool wiped = true;
	int nblocks = 4096;	/* 16MB size */
	int j;

	buf = malloc(F2FS_BLKSIZE);
	if (buf == NULL) {
		MSG(1, "\tError: Malloc Failed for buf!!!\n");
		return false;
	}
	zero_buf = calloc(1, F2FS_BLKSIZE);
	if (zero_buf == NULL) {
		MSG(1, "\tError: Calloc Failed for zero buf!!!\n");
		free(buf);
		return false;
	}

	if (lseek(fd, 0, SEEK_SET) < 0) {
		free(zero_buf);
		free(buf);
		return false;
	}

	/* check first n blocks */
	for (j = 0; j < nblocks; j++) {
		if (read(fd, buf, F2FS_BLKSIZE) != F2FS_BLKSIZE ||
				memcmp(buf, zero_buf, F2FS_BLKSIZE)) {
			wiped = false;
			break;
		}
	}
	free(zero_buf);
	free(buf);

	if (wiped)
		MSG(0, "Info: Found all zeros in first %d blocks\n", nblocks);
	return wiped;
#else
	return false;
#endif
#endif
}

int f2fs_trim_devices(void)
{
	int i, err = 0;

	err = trim_update_magic(TRIM_STEP_MAGIC_1);
	if (err < 0) {
		MSG(0, "\tError: Failed to update magic1!!!\n");
		return -1;
	}

	for (i = 0; i < c.ndevs; i++) {
		if (!is_wiped_device(i) && trim_device(i))
			return -1;
	}

	err = trim_update_magic(TRIM_STEP_MAGIC_2);
	if (err < 0) {
		MSG(0, "\tError: Failed to update magic2!!!\n");
		return -1;
	}

	c.trimmed = 1;
	return 0;
}
