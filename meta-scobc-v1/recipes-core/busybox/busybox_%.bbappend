FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://taskset.cfg \
    "

do_configure:append() {
    cat ${WORKDIR}/taskset.cfg >> ${B}/.config
}

