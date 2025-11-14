FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += " \
    file://0001-mtd-spi-nor-sfdp-introduce-smpt_read_dummy-fixup-hoo.patch \
    file://0002-mtd-spi-nor-sfdp-introduce-smpt_map_id-fixup-hook.patch \
    file://0003-mtd-spi-nor-spansion-SMPT-fixups-for-S25FS-S.patch \
    file://0004-mtd-spi-nor-spansion-Fix-warning.patch \
    "
