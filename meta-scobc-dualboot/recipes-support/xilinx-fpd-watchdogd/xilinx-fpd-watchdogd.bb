SUMMARY = "Watchdog kicker daemon for xilinx-wwdt (/dev/watchdog0)"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://Makefile \
    file://xilinx-fpd-watchdogd.c \
    file://xilinx-fpd-watchdogd.service \
    "

S = "${WORKDIR}"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:versal = ".*"
COMPATIBLE_MACHINE:versal-net = ".*"
COMPATIBLE_MACHINE:versal-2ve-2vm = ".*"

inherit systemd

EXTRA_OEMAKE = " \
    DESTDIR=${D} BINDIR=${bindir} \
    CPPFLAGS+='-DTIMEOUT=${XILINX_FPD_WWDT_TIMEOUT} -DCLOSED_WINDOW_PERCENT=${XILINX_FPD_WWDT_CLOSED_WINDOW_PERCENT}' \
    "

do_install() {
    oe_runmake install

    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/xilinx-fpd-watchdogd.service ${D}${systemd_unitdir}/system/
}

FILES:${PN} += " \
    ${bindir}/xilinx-fpd-watchdogd \
    ${systemd_unitdir}/system/xilinx-fpd-watchdogd.service \
    "

SYSTEMD_SERVICE:${PN} = "xilinx-fpd-watchdogd.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

RDEPENDS:${PN} += "udev"
