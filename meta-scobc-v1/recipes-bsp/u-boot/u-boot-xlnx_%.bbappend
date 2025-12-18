require u-boot-elf-install-fix.inc
require u-boot-extra-cfgs.inc

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI:append = " \
    file://0001-net-phy-dp83867-Write-only-SW_RESTART-when-restartin.patch \
"
