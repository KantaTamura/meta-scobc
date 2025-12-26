FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " \
    file://0001-boot-add-Kconfig-to-select-boot-partition-by-label.patch \
    file://0001-cmd-add-non-secure-mb-command-to-read-write-PMC_MULT.patch \
"

UBOOT_EXTRA_CFGS:append = " \
    multiboot.cfg \
"

UBOOT_EXTRA_CFGS[golden] = " \
    dualboot-golden.cfg \
"
UBOOT_EXTRA_CFGS[main] = " \
    dualboot-main.cfg \
"
