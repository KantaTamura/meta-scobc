FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " \
    file://fstab.rauc.append \
"

do_install:append() {
    install -d ${D}/srv/artifacts
    cat ${WORKDIR}/fstab.rauc.append >> ${D}${sysconfdir}/fstab
}
