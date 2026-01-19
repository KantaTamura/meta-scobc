FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

do_install:prepend() {
	sed -i "s/@@MACHINE@@/${MACHINE}/g" ${WORKDIR}/system.conf
}

RDEPENDS:${PN}:append = " \
    rauc-post-install-bootbin \
"
