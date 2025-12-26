SUMMARY = "reboot command with multiboot register control"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI = " \
    file://boot-to-golden \
    file://boot-to-main \
"

RDEPENDS:${PN} += " \
    multiboot-tool \
    mark-boot-success \
"

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/boot-to-golden ${D}${sbindir}/
    install -m 0755 ${WORKDIR}/boot-to-main   ${D}${sbindir}/
}

FILES:${PN} += " \
    ${sbindir}/boot-to-golden \
    ${sbindir}/boot-to-main \
"
