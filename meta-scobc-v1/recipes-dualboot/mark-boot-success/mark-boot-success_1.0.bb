SUMMARY = "Mark Linux boot success by restoring PMC_MULTI_BOOT register"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://mark-boot-success \
           file://mark-boot-success.service \
           file://mark-boot-success.timer.in \
           "

S = "${WORKDIR}"

inherit systemd

MARK_BOOT_SUCCESS_ONBOOT_DELAY ?= "30min"

do_configure() {
    install -d ${B}
    cp ${WORKDIR}/mark-boot-success.timer.in ${B}/mark-boot-success.timer
    sed -i \
        -e "s|@ONBOOT_DELAY@|${MARK_BOOT_SUCCESS_ONBOOT_DELAY}|g" \
        ${B}/mark-boot-success.timer
}

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/mark-boot-success ${D}${sbindir}/mark-boot-success

    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/mark-boot-success.service ${D}${systemd_unitdir}/system/
    install -m 0644 ${B}/mark-boot-success.timer ${D}${systemd_unitdir}/system/
}

FILES:${PN} += "${sbindir}/mark-boot-success \
                ${systemd_unitdir}/system/mark-boot-success.service \
                ${systemd_unitdir}/system/mark-boot-success.timer \
                "

SYSTEMD_SERVICE:${PN} = "mark-boot-success.service mark-boot-success.timer"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

RDEPENDS:${PN} += "busybox"
