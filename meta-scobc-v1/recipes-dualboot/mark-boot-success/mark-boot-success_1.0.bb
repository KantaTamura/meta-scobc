SUMMARY = "Mark Linux boot success by restoring PMC_MULTI_BOOT register"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://mark-boot-success \
           file://mark-boot-success.service \
           file://mark-boot-success.timer \
           "

S = "${WORKDIR}"

inherit systemd

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/mark-boot-success ${D}${sbindir}/mark-boot-success

    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/mark-boot-success.service ${D}${systemd_unitdir}/system/
    install -m 0644 ${WORKDIR}/mark-boot-success.timer ${D}${systemd_unitdir}/system/
}

FILES:${PN} += "${sbindir}/mark-boot-success \
                ${systemd_unitdir}/system/mark-boot-success.service \
                ${systemd_unitdir}/system/mark-boot-success.timer \
                "

SYSTEMD_SERVICE:${PN} = "mark-boot-success.service mark-boot-success.timer"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

RDEPENDS:${PN} += "busybox"
