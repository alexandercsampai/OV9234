#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};


MODULE_INFO(depends, "v4l2-fwnode,videodev,v4l2-async,mc");

MODULE_ALIAS("acpi*:OVTI9234:*");
MODULE_ALIAS("i2c:ov9234");
MODULE_ALIAS("of:N*T*Comnivision,ov9234");
MODULE_ALIAS("of:N*T*Comnivision,ov9234C*");
