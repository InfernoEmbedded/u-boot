// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * load_file.c
 *
 * Copyright (c) 2007-2024 Allwinnertech Co., Ltd.
 */

#include <common.h>
#include <malloc.h>
#include <asm/cache.h>
#include <command.h>
#include <linux/string.h>
#include <gzip.h>
#include "load_file.h"
#include "boot_bmp.h"

static int __unload_file(struct file_info_t *file)
{
	if (file) {
		free(file->name);
		free(file->path);
		free(file->file_addr);
		free(file);
		return 0;
	}
	return -1;
}

static int __print_file_info(struct file_info_t *file)
{
	pr_err("File name:%s\n", file->name);
	pr_err("Partition name:%s\n", file->path);
	pr_err("File size:%u\n", file->file_size);
	pr_err("File addr:%p\n", file->file_addr);
	return 0;
}

void *decompress_boot_bmp(void)
{
	unsigned long uncomp_len = 256 * 1024;
	void *uncomp_buf = malloc(uncomp_len);

	if (!uncomp_buf) {
		printf("malloc failed!\n");
		return NULL;
	}

	if (gunzip(uncomp_buf, uncomp_len,
		   boot_bmp_gz, (long unsigned int *)&boot_bmp_gz_len) != 0) {
		printf("gunzip failed!\n");
		free(uncomp_buf);
		return NULL;
	}

	printf("boot.bmp decompressed OK\n");
	return uncomp_buf;
}

static struct file_info_t *create_boot_bmp_file(void)
{
	struct file_info_t *file = NULL;

	file = malloc(sizeof(struct file_info_t));
	if (!file) {
		pr_err("malloc failed\n");
		return NULL;
	}

	memset(file, 0, sizeof(struct file_info_t));

	file->file_addr = decompress_boot_bmp();
	file->file_size = 256 * 1024;

	file->name = strdup("boot.bmp");
	file->path = strdup("embedded_array");

	file->unload_file = __unload_file;
	file->print_file_info = __print_file_info;

	return file;
}

struct file_info_t *load_file(char *name, char *part_name)
{
	return create_boot_bmp_file();
}

int write_file(char *name, char *part_name, void *buf_addr, unsigned int buf_size)
{
	return 0;
}
