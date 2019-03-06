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


static struct spi_driver tsp92518_spi_driver{
  .id_table = tsp92518_id,
  .probe = ,
  .remove = ,
  .shutdown = ,
  .driver = {
          .name = "tsp92518",
          .acpi_match_table = ACPI_PTR(tsp92518_acpi_ids),
          .of_match_table = of_match_ptr(tsp92518_dt_ids),
          .pm = &tsp92518_pm,
            },
};

module_spi_driver(tsp92518_spi_driver);

MODULE_DESCRIPTION("TPS92518-Q1 Driver");
MODULE_AUTHOR("Christopher Tyerman");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("spi:TPS92518");
