/*
 * (C) Copyright 2000-2009
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <bootm.h>
#include <command.h>
#include <image.h>
#include <libfdt.h>
#include <lmb.h>
#include <mapmem.h>
#include <stdlib.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

/* See Documentation/arm64/booting.txt in the Linux kernel */
struct Image_header {
	uint32_t	code0;		/* Executable code */
	uint32_t	code1;		/* Executable code */
	uint64_t	text_offset;	/* Image load offset, LE */
	uint64_t	image_size;	/* Effective Image size, LE */
	uint64_t	flags;		/* Kernel flags, LE */
	uint64_t	res2;		/* reserved */
	uint64_t	res3;		/* reserved */
	uint64_t	res4;		/* reserved */
	uint32_t	magic;		/* Magic number */
	uint32_t	res5;
};

#define LINUX_ARM64_IMAGE_MAGIC	0x644d5241

static int booti_setup(bootm_headers_t *images)
{
	struct Image_header *ih;
	uint64_t dst;
	uint64_t image_size, text_offset;

	ih = (struct Image_header *)map_sysmem(images->ep, 0);

	if (ih->magic != le32_to_cpu(LINUX_ARM64_IMAGE_MAGIC)) {
		puts("Bad Linux ARM64 Image magic!\n");
		return 1;
	}

	/*
	 * Prior to Linux commit a2c1d73b94ed, the text_offset field
	 * is of unknown endianness.  In these cases, the image_size
	 * field is zero, and we can assume a fixed value of 0x80000.
	 */
	if (ih->image_size == 0) {
		puts("Image lacks image_size field, assuming 16MiB\n");
		image_size = 16 << 20;
		text_offset = 0x80000;
	} else {
		image_size = le64_to_cpu(ih->image_size);
		text_offset = le64_to_cpu(ih->text_offset);
	}

	/*
	 * If bit 3 of the flags field is set, the 2MB aligned base of the
	 * kernel image can be anywhere in physical memory, so respect
	 * images->ep.  Otherwise, relocate the image to the base of RAM
	 * since memory below it is not accessible via the linear mapping.
	 */
	if (le64_to_cpu(ih->flags) & BIT(3))
		dst = images->ep - text_offset;
	else
		dst = gd->bd->bi_dram[0].start;

	dst = ALIGN(dst, SZ_2M) + text_offset;

	unmap_sysmem(ih);

	if (images->ep != dst) {
		void *src;

		debug("Moving Image from 0x%lx to 0x%llx\n", images->ep, dst);

		src = (void *)images->ep;
		images->ep = dst;
		memmove((void *)dst, src, image_size);
	}

	return 0;
}

/*
 * Image booting support
 */
static int booti_start(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[], bootm_headers_t *images)
{
	int ret;
	struct Image_header *ih;

	ret = do_bootm_states(cmdtp, flag, argc, argv, BOOTM_STATE_START,
			      images, 1);

	/* Setup Linux kernel Image entry point */
	if (!argc) {
		images->ep = load_addr;
		debug("*  kernel: default image load address = 0x%08lx\n",
				load_addr);
	} else {
		images->ep = simple_strtoul(argv[0], NULL, 16);
		debug("*  kernel: cmdline image address = 0x%08lx\n",
			images->ep);
	}

	ret = booti_setup(images);
	if (ret != 0)
		return 1;

	ih = (struct Image_header *)map_sysmem(images->ep, 0);

	lmb_reserve(&images->lmb, images->ep, le32_to_cpu(ih->image_size));

	unmap_sysmem(ih);

	/*
	 * Handle the BOOTM_STATE_FINDOTHER state ourselves as we do not
	 * have a header that provide this informaiton.
	 */
	if (bootm_find_images(flag, argc, argv))
		return 1;

	return 0;
}

int do_booti_a(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;

	/* Consume 'booti' */
	argc--; argv++;

	if (booti_start(cmdtp, flag, argc, argv, &images))
		return 1;

	/*
	 * We are doing the BOOTM_STATE_LOADOS state ourselves, so must
	 * disable interrupts ourselves
	 */
	bootm_disable_interrupts();

	images.os.os = IH_OS_LINUX;
	images.os.arch = IH_ARCH_ARM64;
	ret = do_bootm_states(cmdtp, flag, argc, argv,
#ifdef CONFIG_SYS_BOOT_RAMDISK_HIGH
			      BOOTM_STATE_RAMDISK |
#endif
			      BOOTM_STATE_OS_PREP | BOOTM_STATE_OS_FAKE_GO |
			      BOOTM_STATE_OS_GO,
			      &images, 1);

	return ret;
}

int do_booti(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct andr_img_hdr *hdr;
	ulong kernel_addr = 0;
	ulong kernel_len = 0;
	ulong ramdisk_addr = 0;
	ulong ramdisk_len = 0;
	ulong fdt_addr = 0;
	ulong fdt_len = 0;
	ulong ramdisk_addr_env = 0;
	ulong fdt_addr_env = 0;
	int i;

	if (argc == 4) {
		debug("normal %s %s %s %s\n", argv[0], argv[1], argv[2], argv[3]);
		return do_booti_a(cmdtp, flag, argc, argv);
	}

	debug("boot android arm64 bootimage\n");
	hdr = (struct andr_img_hdr *)simple_strtoul(argv[1], NULL, 16);
	if (android_image_check_header(hdr)) {
		printf("invalid android image\n");
		return -1;
	}

	android_image_get_kernel(hdr, false, &kernel_addr, &kernel_len);
	android_image_get_ramdisk(hdr, &ramdisk_addr, &ramdisk_len);
	android_image_get_second(hdr, &fdt_addr, &fdt_len);

	if (fdt_check_header((void*)fdt_addr)) {
		printf(" error: invalid fdt\n");
		return -1;
	}

	/* relocate ramdisk and fdt to the address defined by the environment variable.
	 * that means we'll ignore the load address of ramdisk and dtb defined in the
	 * abootimg, since it make more sense letting u-boot handling where to put what.
	 * kernel relocation will be handled in booti_setup
	 */
	ramdisk_addr_env = env_get_ulong("ramdisk_addr_r", 16, 0);;
	fdt_addr_env = env_get_ulong("fdt_addr_r", 16, 0);

	if (!ramdisk_addr_env) {
		printf(" error: didn't define ramdisk_addr_r\n");
		return -1;
	}
	memmove((void *)ramdisk_addr_env, (void *)ramdisk_addr, ramdisk_len);

	if (!fdt_addr_env) {
		printf(" error: didn't define fdt_addr_r\n");
		return -1;
	}
	memmove((void *)fdt_addr_env, (void *)fdt_addr, fdt_len);

	const int max_length = 40;
	const int new_argc = 4;
	char *new_argv[new_argc];

	for (i = 0; i < new_argc; i++) {
		new_argv[i] = (char*) malloc(max_length);
	}

	strcpy(new_argv[0], "booti");
	snprintf(new_argv[1], max_length, "0x%lx", kernel_addr);
	snprintf(new_argv[2], max_length, "0x%lx:%lx",
			ramdisk_addr_env, ramdisk_len);
	snprintf(new_argv[3], max_length, "0x%lx", fdt_addr_env);

	debug("android: %s %s %s %s\n", new_argv[0], new_argv[1], new_argv[2], new_argv[3]);

	int ret = do_booti_a(cmdtp, flag, new_argc, new_argv);

	for (i = 0; i < new_argc; i++) {
		free(new_argv[i]);
	}

	return ret;
}
#ifdef CONFIG_SYS_LONGHELP
static char booti_help_text[] =
	"[addr [initrd[:size]] [fdt]]\n"
	"    - boot arm64 Linux Image stored in memory\n"
	"\tThe argument 'initrd' is optional and specifies the address\n"
	"\tof an initrd in memory. The optional parameter ':size' allows\n"
	"\tspecifying the size of a RAW initrd.\n"
#if defined(CONFIG_OF_LIBFDT)
	"\tSince booting a Linux kernel requires a flat device-tree, a\n"
	"\tthird argument providing the address of the device-tree blob\n"
	"\tis required. To boot a kernel with a device-tree blob but\n"
	"\twithout an initrd image, use a '-' for the initrd argument.\n"
#endif
	"";
#endif

U_BOOT_CMD(
	booti,	CONFIG_SYS_MAXARGS,	1,	do_booti,
	"boot arm64 Linux Image image from memory", booti_help_text
);
