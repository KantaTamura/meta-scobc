SUMMARY = "PMC_MULTI_BOOT control utility (Versal) using devmem"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " \
    file://post-install-bootbin \
"

do_install() {
    install -d ${D}/usr/lib/rauc
    install -m 0755 ${WORKDIR}/post-install-bootbin ${D}/usr/lib/rauc/post-install-bootbin
}

FILES:${PN}:append = " \
    /usr/lib/rauc/post-install-bootbin \
"
