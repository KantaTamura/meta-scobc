FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " \
    file://0001-boot-add-Kconfig-to-select-boot-partition-by-label.patch \
"

UBOOT_EXTRA_CFGS[golden] = " \
    dualboot-golden.cfg \
"
UBOOT_EXTRA_CFGS[main] = " \
    dualboot-main.cfg \
"
