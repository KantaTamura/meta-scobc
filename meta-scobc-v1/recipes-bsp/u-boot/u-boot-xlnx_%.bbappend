require u-boot-elf-install-fix.inc

UBOOT_XLNX_INCLUDE ?= ""
UBOOT_XLNX_INCLUDE:dualboot = "u-boot-xlnx-dualboot.inc"
include ${UBOOT_XLNX_INCLUDE}
