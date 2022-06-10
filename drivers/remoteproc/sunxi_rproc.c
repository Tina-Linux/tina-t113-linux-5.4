// SPDX-License-Identifier: GPL-2.0
/*
 * sunxi's Remote Processor Control Driver
 * base on st_remoteproc.c and sunxi_remoteproc.c
 *
 * Copyright (C) 2015 Allwinnertech - All Rights Reserved
 *
 * Author: cody <cody@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/clk/sunxi.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include "remoteproc_internal.h"

struct sunxi_resource_map_table {
	phys_addr_t pa; /* Address of cpu's address */
	phys_addr_t da; /* Address of rproc's address */
	int len;
};

struct sunxi_rproc_data {
	struct sunxi_resource_map_table *map;
	int msize;
};

struct sunxi_rproc {
	struct sunxi_rproc_data *match;
	struct clk *clk;
	struct list_head map;
	struct reset_control *reset;
	int irq;
};

struct sunxi_map_resource {
	struct list_head list;
	struct resource res;
	void __iomem *va;
};

static int sunxi_rproc_start(struct rproc *rproc)
{
	struct sunxi_rproc *ddata = rproc->priv;
	int err;
	void __iomem *base_addr;

	base_addr = ioremap(0x03000000, 0x10);
	iowrite32(0x0, base_addr + 0x8);


	clk_enable(ddata->clk);

	if (ddata->reset) {
		err = reset_control_deassert(ddata->reset);
		if (err) {
			dev_err(&rproc->dev, "reset_control_deassert() failed\n");
			return err;
		}
	}

	return 0;
}

static int sunxi_rproc_stop(struct rproc *rproc)
{
	struct sunxi_rproc *ddata = rproc->priv;
	int err;
	void __iomem *base_addr;

	clk_disable(ddata->clk);

	if (ddata->reset) {
		err = reset_control_assert(ddata->reset);
		if (err) {
			dev_err(&rproc->dev, "reset_control_assert() failed\n");
			return err;
		}
	}

	base_addr = ioremap(0x03000000, 0x10);
	iowrite32(0x1, base_addr + 0x8);

	return 0;
}

static void *sunxi_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct sunxi_rproc *ddata = rproc->priv;
	struct sunxi_map_resource *res;
	struct list_head *head = &ddata->map;
	const struct sunxi_resource_map_table *t;
	int i;

	/* first find the address in table */
	for (i = 0; i < ddata->match->msize; i++) {
		t = &ddata->match->map[i];

		if (da >= t->da && da + len  <= t->da + t->len)
			break;
	}

	if (i == ddata->match->msize) {
		dev_err(&rproc->dev, "failed find match address\n");
		return 0;
	}
	/* second find the mapped resource  */
	list_for_each_entry(res, head, list) {
		if (res->res.start == t->pa)
			return (da - t->da) + res->va;
	}

	dev_err(&rproc->dev, "failed da_to_va\n");

	return 0;
}

static struct elf32_shdr *sunxi_find_oemhead(struct elf32_hdr *ehdr)
{
	struct elf32_shdr *shdr;
	int i;
	const char *name_table;
	const u8 *elf_data = (void *)ehdr;

	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		if (!strcmp(name_table + shdr->sh_name, ".oemhead.text"))
			return shdr;
	}

	return NULL;
}

static int sunxi_rproc_load(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	struct elf32_shdr *oemhead_shdr;
	u32 oemhead_da;
	u32 oemhead_size;
	u8 *oemhead_data = NULL;
	void *oemhead_va;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/*
	 * Find the ".oemhead.text" section, and backup its original data in memory
	 */
	oemhead_shdr = sunxi_find_oemhead(ehdr);
	if (oemhead_shdr) {
		oemhead_da = oemhead_shdr->sh_addr;
		oemhead_size = oemhead_shdr->sh_size;

		oemhead_va = rproc_da_to_va(rproc, oemhead_da, oemhead_size);
		if (!oemhead_va) {
			dev_err(dev, "oemhead: bad phdr da 0x%x size 0x%x\n",
				oemhead_da, oemhead_size);
			return -EINVAL;
		}

		oemhead_data = kmalloc(oemhead_size, GFP_KERNEL);
		if (!oemhead_data) {
			dev_warn(dev, "failed to allocate memory for oemhead\n");
			return -ENOMEM;
		}

		memcpy_fromio(oemhead_data, oemhead_va, oemhead_size);

		dev_dbg(dev, "oemhead found: da: 0x%x, size: 0x%x\n",
			oemhead_da, oemhead_size);
	}

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void *ptr;

		if ((phdr->p_type != PT_LOAD) || (!filesz))
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			memcpy_toio(ptr, elf_data + phdr->p_offset, filesz);

		/*
		 * Zero out remaining memory for this segment.
		 */
		if (memsz > filesz)
			memset_io(ptr + filesz, 0, memsz - filesz);

		/*
		 * If the ".oemhead.text" section is in this segment, its
		 * original data in memory has been overwritten by putting the
		 * whole segment before, now we write it back to memory.
		 */
		if (oemhead_shdr && oemhead_da >= da &&
		    oemhead_da + oemhead_size <= da + memsz)
			memcpy_toio(oemhead_va, oemhead_data, oemhead_size);
	}

	kfree(oemhead_data);

	return ret;
}

static struct rproc_ops sunxi_rproc_ops = {
	.start		= sunxi_rproc_start,
	.stop		= sunxi_rproc_stop,
	.da_to_va   = sunxi_da_to_va,
	.load		= sunxi_rproc_load,
};

/*
 * Fetch state of the processor: 0 is off, 1 is on.
 */
static int sunxi_rproc_state(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_rproc *ddata = rproc->priv;

	return __clk_is_enabled(ddata->clk);
}

static const struct of_device_id sunxi_rproc_match[] = {
	{ .compatible = "sunxi,sunxi-remote-proc"},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_rproc_match);

static int sunxi_rproc_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_rproc *ddata = rproc->priv;
	struct sunxi_resource_map_table *mem_ranges;
	struct device_node *np = dev->of_node;
	int i, err, cnt, array_size;

	cnt = of_property_count_elems_of_size(np, "ranges",
					      sizeof(*mem_ranges));
	if (cnt <= 0) {
		dev_err(dev, "%s: ranges property not defined\n", __func__);
		return -EINVAL;
	}

	ddata->match->map = kcalloc(cnt, sizeof(struct sunxi_resource_map_table), GFP_KERNEL);
	if (!ddata->match->map)
		return -ENOMEM;

	mem_ranges = kcalloc(cnt, sizeof(*mem_ranges), GFP_KERNEL);
	if (!mem_ranges)
		return -ENOMEM;

	array_size = cnt * sizeof(struct sunxi_resource_map_table) / sizeof(u32);

	err = of_property_read_u32_array(np, "ranges",
					 (u32 *)mem_ranges, array_size);
	if (err) {
		dev_err(dev, "error while get ranges property: %x\n", err);
		return -EINVAL;
	}

	ddata->match->msize = cnt;
	for (i = 0; i < cnt; i++) {
		ddata->match->map[i].da = mem_ranges[i].da;
		ddata->match->map[i].pa = mem_ranges[i].pa;
		ddata->match->map[i].len     = mem_ranges[i].len;
		dev_dbg(dev, "memory range[%i]: da %#x, pa %#x, len %#zx:\n",
			i, mem_ranges[i].pa, mem_ranges[i].da,
			mem_ranges[i].len);
	}

	return err;
}

static int sunxi_map_da_res(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_rproc *ddata = rproc->priv;
	struct list_head *head = &ddata->map;
	struct sunxi_map_resource *res;
	int i;

	INIT_LIST_HEAD(head);

	for (i = 0; i < ddata->match->msize; i++) {
		const struct sunxi_resource_map_table *t;
		int f = 0;

		t = &ddata->match->map[i];

		list_for_each_entry(res, head, list) {
			if (res->res.start == t->pa) {
				f = 1;
				break;
			}
		}

		if (f == 1)
			continue;

		res = devm_kzalloc(&pdev->dev,
				   sizeof(struct sunxi_map_resource),
				   GFP_KERNEL);
		if (!res) {
			dev_err(&pdev->dev, "failed get mem\n");
			return -ENOMEM;
		}

		res->res.start = t->pa;
		res->res.end = t->pa + t->len - 1;

		/* for some arch, multi rproc share the some address */
		res->va = devm_ioremap(&pdev->dev, res->res.start, t->len);
		if (!PTR_ERR(res->va)) {
			dev_err(&pdev->dev, "map res err %x - %x\n",
				(int)res->res.start, (int)res->res.end);
			return -ENOMEM;
		}
		dev_dbg(&pdev->dev, "va %x will map res %x - %x\n", res->va,
				(int)res->res.start, (int)res->res.end);
		list_add(&res->list, head);
	}

	return 0;
}

irqreturn_t sunxi_recovery_irq(int irq, void *d)
{
	struct rproc *rproc = d;

	if (!rproc) {
		pr_err("NULL rproc pointer\n");
		return IRQ_HANDLED;
	}

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

static int sunxi_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct sunxi_rproc *ddata;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int enabled;
	int ret;

	match = of_match_device(sunxi_rproc_match, dev);
	if (!match) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, "sunxi-rproc", &sunxi_rproc_ops, NULL, sizeof(*ddata));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	rproc->auto_boot = false;

	ddata = rproc->priv;
	ddata->match = vmalloc(sizeof(struct sunxi_rproc_data));

	platform_set_drvdata(pdev, rproc);

	ddata->irq = platform_get_irq(pdev, 0);
	if (ddata->irq < 0)
		dev_err(dev, "Failed to get recovery irq\n");
	else {
		ret = devm_request_irq(dev, ddata->irq,
				sunxi_recovery_irq, 0,
				dev_name(dev), rproc);
		if (ret)
			dev_err(dev, "Failed to request irq\n");
	}

	ret = sunxi_rproc_parse_dt(pdev);
	if (ret) {
		dev_err(dev, "sunxi rproc parse dt err\n");
		goto free_rproc;
	}

	ret = sunxi_map_da_res(pdev);
	if (ret)
		goto free_rproc;

	ddata->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(ddata->reset)) {
		dev_err(dev, "Failed to get reset\n");
		ret = PTR_ERR(ddata->reset);
		goto free_rproc;
	}

	ddata->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ddata->clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(ddata->clk);
		goto free_rproc;
	}

	ret = clk_prepare_enable(ddata->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		goto free_rproc;
	}

	enabled = sunxi_rproc_state(pdev);
	if (enabled < 0) {
		ret = enabled;
		goto free_rproc;
	} else if (enabled) {
		atomic_inc(&rproc->power);
		rproc->state = RPROC_RUNNING;
	}

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int sunxi_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_rproc *ddata = rproc->priv;

	rproc_del(rproc);

	clk_disable_unprepare(ddata->clk);

	rproc_free(rproc);

	return 0;
}

static struct platform_driver sunxi_rproc_driver = {
	.probe = sunxi_rproc_probe,
	.remove = sunxi_rproc_remove,
	.driver = {
		.name = "sunxi-rproc",
		.of_match_table = sunxi_rproc_match,
	},
};
module_platform_driver(sunxi_rproc_driver);

MODULE_DESCRIPTION("SUNXI Remote Processor Control Driver");
MODULE_AUTHOR("cody <cody@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
