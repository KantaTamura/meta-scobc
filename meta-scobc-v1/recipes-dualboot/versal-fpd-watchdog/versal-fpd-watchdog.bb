SUMMARY = "Watchdog kicker daemon for /dev/versal-fpd-watchdog"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://Makefile \
    file://versal-fpd-watchdog-daemon.c \
    file://versal-fpd-watchdog.service \
    "

S = "${WORKDIR}"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:versal = ".*"
COMPATIBLE_MACHINE:versal-net = ".*"
COMPATIBLE_MACHINE:versal-2ve-2vm = ".*"

inherit systemd

VERSAL_FPD_SWDT_TIMEOUT ?= "10"
VERSAL_FPD_SWDT_KICK_DELAY ?= "1"

EXTRA_OEMAKE = " \
    DESTDIR=${D} BINDIR=${bindir} \
    CPPFLAGS+='-DTIMEOUT=${VERSAL_FPD_SWDT_TIMEOUT} -DKICK_DELAY=${VERSAL_FPD_SWDT_KICK_DELAY}' \
    "

do_install() {
    oe_runmake install

    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/versal-fpd-watchdog.service ${D}${systemd_unitdir}/system/
}

FILES:${PN} += " \
    ${bindir}/versal-fpd-watchdog-daemon \
    ${systemd_unitdir}/system/versal-fpd-watchdog.service \
    "

SYSTEMD_SERVICE:${PN} = "versal-fpd-watchdog.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

RDEPENDS:${PN} += "udev"
