/*
 * export.c - exporting kernel functions that aren't exported by default.
 * The general idea is bring in whatever headers define the functions then use an EXPORT_SYMBOL to expose it to modules.
 */

#include <linux/module.h>
#include <linux/fs.h>

EXPORT_SYMBOL(do_sys_open);
