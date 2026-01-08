SUMMARY = "Zephyr ELF for Versal RPU"
LICENSE = "CLOSED"

inherit allarch
INSANE_SKIP:${PN} += "arch"

PROVIDES += "zephyr-rpu"
SYSROOT_DIRS:append = " /boot"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI = "file://zephyr.elf"
S = "${WORKDIR}"

do_install() {
    install -d ${D}/boot
    install -m 0644 ${WORKDIR}/zephyr.elf ${D}/boot/zephyr-rpu.elf
}

FILES:${PN} += "/boot/zephyr-rpu.elf"

