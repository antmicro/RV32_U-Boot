// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Microsemi Corporation.
 * Padmarao Begari, Microsemi Corporation <padmarao.begari@microsemi.com>
 */

#include <asm/mach-types.h>
#include <common.h>
#if defined(CONFIG_MACB) && !defined(CONFIG_DM_ETH)
#include <netdev.h>
#endif
#include <linux/io.h>

DECLARE_GLOBAL_DATA_PTR;

struct hifive_gpio_regs
{
    volatile uint32_t  INPUT_VAL;   /* 0x0000 */
    volatile uint32_t  INPUT_EN;    /* 0x0004 */
    volatile uint32_t  OUTPUT_VAL;  /* 0x0008 */
    volatile uint32_t  OUTPUT_EN;   /* 0x000C */
    volatile uint32_t  PUE;         /* 0x0010 */
    volatile uint32_t  DS;          /* 0x0014 */
    volatile uint32_t  RISE_IE;     /* 0x0018 */
    volatile uint32_t  RISE_IP;     /* 0x001C */
    volatile uint32_t  FALL_IE;     /* 0x0020 */
    volatile uint32_t  FALL_IP;     /* 0x0024 */
    volatile uint32_t  HIGH_IE;     /* 0x0028 */
    volatile uint32_t  HIGH_IP;     /* 0x002C */
    volatile uint32_t  LOW_IE;      /* 0x0030 */
    volatile uint32_t  LOW_IP;      /* 0x0034 */
    volatile uint32_t  reserved0;   /* 0x0038 */
    volatile uint32_t  reserved1;   /* 0x003C */
    volatile uint32_t  OUT_XOR;     /* 0x0040 */
};

struct hifive_gpio_regs *g_aloe_gpio = (struct hifive_gpio_regs *)HIFIVE_BASE_GPIO;


/*
 * Miscellaneous platform dependent initializations
 */

int board_init(void)
{
	gd->bd->bi_arch_number = MACH_TYPE_HIFIVE_U540;
	gd->bd->bi_boot_params = PHYS_SDRAM_0;

	return 0;
}

int dram_init(void)
{
	unsigned long sdram_base = PHYS_SDRAM_0;
	unsigned long expected_size = PHYS_SDRAM_0_SIZE;/* + PHYS_SDRAM_1_SIZE;*/
	unsigned long actual_size;

//	actual_size = PHYS_SDRAM_0_SIZE;/*get_ram_size((void *)sdram_base, expected_size);*/
	actual_size = get_ram_size((void *)sdram_base, expected_size);
	gd->ram_size = actual_size;

	if (expected_size != actual_size) {
		printf("Warning: Only %lu of %lu MiB SDRAM is working\n",
			actual_size >> 20, expected_size >> 20);
	}
	gd->fdt_size = ALIGN(fdt_totalsize(gd->fdt_blob) + 0x1000, 32);
	memcpy((void *)0x41000000, gd->fdt_blob, gd->fdt_size);

	return 0;
}

void reset_phy(void)
{
    volatile uint32_t loop;

/*
 * Init includes toggling the reset line which is connected to GPIO 0 pin 12.
 * This is the only pin I can see on the 16 GPIO which is currently set as an.
 * output. We will hard code the setup here to avoid having to have a GPIO
 * driver as well...
 *
 * The Aloe board is strapped for unmanaged mode and needs two pulses of the
 * reset line to configure the device properly.
 *
 * The RX_CLK, TX_CLK and RXD7 pins are strapped high and the remainder low.
 * This selects GMII mode with auto 10/100/1000 and 125MHz clkout.
 */
    g_aloe_gpio->OUTPUT_EN  |= 0x00001000ul;  /* Configure pin 12 as an output */
    g_aloe_gpio->OUTPUT_VAL &= 0x0000EFFFul;  /* Clear pin 12 to reset PHY */
    for(loop = 0; loop != 1000; loop++)     /* Short delay, I'm not sure how much is needed... */
    {
        ;
    }
    g_aloe_gpio->OUTPUT_VAL  |= 0x00001000ul; /* Take PHY^ out of reset */
    for(loop = 0; loop != 1000; loop++)     /* Short delay, I'm not sure how much is needed... */
    {
        ;
    }
    g_aloe_gpio->OUTPUT_VAL &= 0x0000EFFFul;  /* Second reset pulse */
    for(loop = 0; loop != 1000; loop++)     /* Short delay, I'm not sure how much is needed... */
    {
        ;
    }
    g_aloe_gpio->OUTPUT_VAL  |= 0x00001000ul; /* Out of reset once more */

    /* Need at least 15mS delay before accessing PHY after reset... */
    for(loop = 0; loop != 10000; loop++)     /* Long delay, I'm not sure how much is needed... */
    {
        ;
    }

}
int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_0;
	gd->bd->bi_dram[0].size =  PHYS_SDRAM_0_SIZE;
/*	gd->bd->bi_dram[1].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[1].size =  PHYS_SDRAM_1_SIZE;*/

	return 0;
}
#if defined(CONFIG_MACB) && !defined(CONFIG_DM_ETH)
int board_eth_init(bd_t *bd)
{
	int rc = 0;

	rc = macb_eth_initialize(0, (void *)HIFIVE_BASE_ETHERNET, 0x00);

	return rc;
}
#endif
ulong board_flash_get_legacy(ulong base, int banknum, flash_info_t *info)
{
	return 0;
}

/*void *board_fdt_blob_setup(void)
{
	void **ptr = (void *)CONFIG_SYS_SDRAM_BASE;
	if (fdt_magic(*ptr) == FDT_MAGIC)
			return (void *)*ptr;

	return (void *)CONFIG_SYS_FDT_BASE;
}*/
