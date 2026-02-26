# The original uboot_install_elf_config() used ${UBOOT_BINARY} (.bin)
# as the symlink target, causing the .elf symlink to overwrite .bin.
# This version uses ${UBOOT_ELF_BINARY} instead to avoid the conflict.
# We'll keep this workaround until it's fixed upstream.
uboot_install_elf_config () {
    config=$1
    type=$2

    install -m 644 ${B}/${config}/${UBOOT_ELF} ${D}/boot/u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX}
    ln -sf u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX} ${D}/boot/${UBOOT_ELF_BINARY}-${type}
    ln -sf u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX} ${D}/boot/${UBOOT_ELF_BINARY}
}
