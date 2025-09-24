#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
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

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x2c635209, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x85bd1608, "__request_region" },
	{ 0x242c6648, "netdev_info" },
	{ 0x30a93ed, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x86219045, "register_candev" },
	{ 0x2caa7bd4, "pci_free_irq_vectors" },
	{ 0x5f664094, "pci_write_config_word" },
	{ 0x349cba85, "strchr" },
	{ 0x10ea38cd, "single_open" },
	{ 0x77358855, "iomem_resource" },
	{ 0x754d539c, "strlen" },
	{ 0x48330eee, "alloc_can_err_skb" },
	{ 0x42d723fb, "dma_set_mask" },
	{ 0x53363e9, "single_release" },
	{ 0x7a2b4a6b, "usb_reset_endpoint" },
	{ 0x69449ab2, "pci_disable_device" },
	{ 0x130fdd63, "i2c_transfer" },
	{ 0xd89483c5, "netif_carrier_on" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0xc876a99f, "seq_printf" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x510508ea, "usb_kill_urb" },
	{ 0x5c5f6242, "remove_proc_entry" },
	{ 0xdff7669f, "device_destroy" },
	{ 0x8120e312, "__register_chrdev" },
	{ 0x66cca4f9, "__x86_indirect_thunk_rcx" },
	{ 0x74745397, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xa0bcb084, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x46afdcba, "dma_free_attrs" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x3bda749f, "device_create_with_groups" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x223983ec, "seq_read" },
	{ 0x71038ac7, "pv_ops" },
	{ 0x1e0252cf, "dma_set_coherent_mask" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xdde60a79, "can_bus_off" },
	{ 0x614b574d, "netif_rx" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcd3ecaf3, "dma_get_required_mask" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x38ccd1b1, "param_ops_charp" },
	{ 0xaff2dd5d, "pci_set_master" },
	{ 0xf7242b1e, "pci_alloc_irq_vectors_affinity" },
	{ 0x99231916, "_dev_warn" },
	{ 0xfb578fc5, "memset" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0x20e708d2, "close_candev" },
	{ 0x1e1e140e, "ns_to_timespec64" },
	{ 0x6a2bfc93, "netif_tx_wake_queue" },
	{ 0xba0915a3, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x4c9f47a5, "current_task" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0xe180ce57, "usb_deregister" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xfef216eb, "_raw_spin_trylock" },
	{ 0x35b5bbe1, "sysfs_remove_file_from_group" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xcfe1e2f9, "class_unregister" },
	{ 0xde80cd09, "ioremap" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0x3398f7af, "usb_set_interface" },
	{ 0x9c1b39b3, "free_netdev" },
	{ 0x9166fada, "strncpy" },
	{ 0x3d7e65af, "usb_control_msg" },
	{ 0x23f3d4d7, "pci_read_config_word" },
	{ 0x670ecece, "__x86_indirect_thunk_rbx" },
	{ 0xce11de54, "dma_alloc_attrs" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x10bc6bea, "kfree_skb_reason" },
	{ 0x1e6d26a8, "strstr" },
	{ 0xc2d2fffc, "alloc_candev_mqs" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x18c00784, "init_net" },
	{ 0x37d5e8ff, "__class_register" },
	{ 0xf0311d7, "_dev_err" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0x76537aa2, "can_change_mtu" },
	{ 0x1d090da3, "i2c_del_adapter" },
	{ 0xec2dcbc3, "_dev_info" },
	{ 0xb20c3797, "usb_submit_urb" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0x8f548ec0, "unregister_candev" },
	{ 0x8a73ef4d, "alloc_can_skb" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0xf416d181, "usb_reset_device" },
	{ 0xa7b6ca7d, "usb_bulk_msg" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb7d8f02e, "usb_clear_halt" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xc4cffb2c, "netdev_err" },
	{ 0x1035c7c2, "__release_region" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x1c937e3f, "pci_unregister_driver" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xdd47e1fb, "__dev_get_by_name" },
	{ 0x90943690, "open_candev" },
	{ 0xaf88e69b, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xbab9a817, "param_ops_byte" },
	{ 0x640663dd, "pci_irq_vector" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x11b64b24, "seq_lseek" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0xe3f4c62c, "pci_request_regions" },
	{ 0x453fa2e0, "param_array_ops" },
	{ 0xedc03953, "iounmap" },
	{ 0x2554c13b, "__pci_register_driver" },
	{ 0x942410fb, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x49d1e765, "alloc_canfd_skb" },
	{ 0xa6220a12, "sysfs_add_file_to_group" },
	{ 0x6a2d8c2d, "i2c_bit_add_bus" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xb0e602eb, "memmove" },
	{ 0x21531b59, "pci_iomap" },
	{ 0xa487e741, "consume_skb" },
	{ 0x4a1d7a2f, "param_ops_ushort" },
	{ 0x367b5b62, "proc_create" },
	{ 0x21be1b14, "usb_get_current_frame_number" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0xa74ccb9b, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xe8a33f46, "param_ops_ulong" },
	{ 0x7a08b1ac, "param_ops_uint" },
	{ 0x2f886acb, "usb_free_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x437b7175, "usb_alloc_urb" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "can-dev,i2c-algo-bit");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "E384788D80BD3973893A78B");
