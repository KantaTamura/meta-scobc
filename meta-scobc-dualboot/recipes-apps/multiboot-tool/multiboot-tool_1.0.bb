SUMMARY = "PMC_MULTI_BOOT control utility (Versal) using devmem"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

RDEPENDS:${PN} += "busybox"

S = "${WORKDIR}"

SRC_URI = " \
    file://multiboot-tool \
"

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/multiboot-tool ${D}${sbindir}/multiboot-tool
}

FILES:${PN} += " \
    ${sbindir}/multiboot-tool \
"
