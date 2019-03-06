/*
 * Driver for TPS92518-Q1 Automotive Dual Buck LED Controller
 *
 * Copyright (C) 2019 Fireblade Automation Systems Ltd.    
 * Author: Christopher Tyerman <c.tyerman@firebladeautomationsystems.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

static const struct spi_device_id tps92518_id[] = {
	{ "tps92518", 0 },
	{ /* sentinel */ },
};

static int tps92518_probe(struct spi_device *spi, 
                             const struct spi_device_id *id)
{

}

static int tps92518_remove(struct spi_device *spi)
{

}

MODULE_DEVICE_TABLE(i2c, tps92518_id);
#ifdef CONFIG_ACPI
static const struct acpi_device_id pca9685_acpi_ids[] = {
	{ "INT3492", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(acpi, pca9685_acpi_ids);
#endif

#ifdef CONFIG_OF
static const struct of_device_id tps92518_dt_ids[] = {
	{ .compatible = "ti,tps92518", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tps92518_dt_ids);
#endif


static const struct dev_pm_ops tsp92518_pm = {
	SET_RUNTIME_PM_OPS(tsp92518_runtime_suspend,
			   tsp92518_runtime_resume, NULL)
};

static struct spi_driver tps92518_spi_driver{
  .id_table = tps92518_id,
  .probe = tps92518_probe,
  .remove = tps92518_remove,
  .shutdown = tps92518_shutdown,
  .driver = {
          .name = "tsp92518",
          .acpi_match_table = ACPI_PTR(tps92518_acpi_ids),
          .of_match_table = of_match_ptr(tps92518_dt_ids),
          .pm = &tsp92518_pm,
            },
};

module_spi_driver(tps92518_spi_driver);

MODULE_DESCRIPTION("TPS92518-Q1 Driver");
MODULE_AUTHOR("Christopher Tyerman");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("spi:TPS92518");
