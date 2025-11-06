FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI:append = " \
    file://0001-boot-add-Kconfig-to-select-boot-partition-by-label.patch \
	file://0002-config-added-config-for-dualboot.patch \
"

# default UBOOT_CONFIG setting
UBOOT_CONFIG ?= "golden main"

UBOOT_CONFIG[golden] ?= "xilinx_versal_dualboot_golden_defconfig"
UBOOT_CONFIG[main]   ?= "xilinx_versal_dualboot_main_defconfig"
