# Add two subpackages for dualboot images:
# - ${PN}-main installs nothing (placeholder for the main image)
# - ${PN}-golden installs boot.bin and boot0001.bin into /boot
#
# On machines where the 'dualboot' override is active, only the
# xilinx-bootbin-golden package installs boot.bin.
# Building other images such as xilinx-bootbin-main does not place it.
# This setup is intended to be used with the sc-image-dualboot-dev image.
PACKAGES:append = " ${PN}-main ${PN}-golden"

FILES:${PN}:remove = "/boot/BOOT.bin"
FILES:${PN}-golden:append = " \
	/boot/boot.bin \
	/boot/boot0001.bin \
"
FILES:${PN}-main = ""

ALLOW_EMPTY:${PN} = "1"
ALLOW_EMPTY:${PN}-main = "1"

# Based on the Versal machine configuration (machine-xilinx-versal.inc),
# this section extends the standard single-boot setup to a dual-boot
# configuration (main / golden). The existing u-boot-xlnx partition
# definition is reused, and only the ELF file path is switched between
# the two variants.
BIF_PARTITION_ATTR[main] = "${BIF_FSBL_ATTR} ${BIF_DEVICETREE_ATTR} ${BIF_ATF_ATTR} u-boot-main"
BIF_PARTITION_ATTR[golden] =  "${BIF_FSBL_ATTR} ${BIF_DEVICETREE_ATTR} ${BIF_ATF_ATTR} u-boot-golden"

BIF_PARTITION_ATTR[u-boot-main]  = "${@d.getVarFlag('BIF_PARTITION_ATTR','u-boot-xlnx')}"
BIF_PARTITION_ID[u-boot-main]    = "${@d.getVarFlag('BIF_PARTITION_ID','u-boot-xlnx')}"
BIF_PARTITION_IMAGE[u-boot-main] = "${RECIPE_SYSROOT}/boot/u-boot.elf-main"

BIF_PARTITION_ATTR[u-boot-golden]   = "${@d.getVarFlag('BIF_PARTITION_ATTR','u-boot-xlnx')}"
BIF_PARTITION_ID[u-boot-golden]     = "${@d.getVarFlag('BIF_PARTITION_ID','u-boot-xlnx')}"
BIF_PARTITION_IMAGE[u-boot-golden]  = "${RECIPE_SYSROOT}/boot/u-boot.elf-golden"

# This do_configure:append generates two *variant-specific* BIF files
# (bootgen-main.bif / bootgen-golden.bif), in addition to the
# standard bootgen.bif. For each variant, it uses the partition order
# declared in BIF_PARTITION_ATTR[<variant>] (e.g. DTB → ATF → U-Boot)
# together with existing flag definitions (BIF_PARTITION_ATTR/IMAGE/ID[...])
# to build Versal-style image { ... } blocks.
python do_configure:append() {
    arch = d.getVar("SOC_FAMILY")
    B = d.getVar('B')

    # NOTE: Currently only supports versal
    if arch not in ["versal", "versal-net"]:
        bb.fatal(f"Unsupported architecture '{arch}' for dual-boot BIF generation."
                 f"This append is intended for Versal family only.")

    def emit_variant_bif(variant: str):
        bifpath = os.path.join(B, f"bootgen-{variant}.bif")
        with open(bifpath, 'w') as biffd:
            biffd.write("the_ROM_image:\n")
            biffd.write("{\n")

            opt = d.getVar("BIF_OPTIONAL_DATA")
            if opt:
                biffd.write("\toptionaldata { %s }\n" % opt)

            bifattr = (d.getVar("BIF_COMMON_ATTR") or "").split()
            if bifattr:
                attrflags = d.getVarFlags("BIF_COMMON_ATTR") or {}
                if arch in ['zynq', 'zynqmp']:
                    create_zynq_bif(bifattr, attrflags, '', '', 1, biffd, d)
                elif arch in ['versal', 'versal-net']:
                    create_versal_bif(bifattr, attrflags, '', '', 1, biffd, d)
                else:
                    create_bif(bifattr, attrflags, '', '', 1, biffd, d)

            parts_str = d.getVarFlag("BIF_PARTITION_ATTR", variant) or ""
            parts = parts_str.split() if parts_str else []
            if not parts:
                bb.fatal(f"BIF_PARTITION_ATTR[{variant}] is empty")

            attrflags = d.getVarFlags("BIF_PARTITION_ATTR") or {}
            attrimage = d.getVarFlags("BIF_PARTITION_IMAGE") or {}
            ids       = d.getVarFlags("BIF_PARTITION_ID") or {}

            if arch in ['zynq', 'zynqmp']:
                create_zynq_bif(parts, attrflags, attrimage, ids, 0, biffd, d)
            elif arch in ['versal', 'versal-net']:
                create_versal_bif(parts, attrflags, attrimage, ids, 0, biffd, d)
            else:
                create_bif(parts, attrflags, attrimage, ids, 0, biffd, d)

            biffd.write("}\n")
        return bifpath

    for v in ("main", "golden"):
        emit_variant_bif(v)
}

do_compile:append() {
    bootgen -image ${B}/bootgen-main.bif -arch ${BOOTGEN_ARCH} ${BOOTGEN_EXTRA_ARGS} -w -o ${B}/BOOT-main.bin
    bootgen -image ${B}/bootgen-golden.bif -arch ${BOOTGEN_ARCH} ${BOOTGEN_EXTRA_ARGS} -w -o ${B}/BOOT-golden.bin
}

# Remove the default BOOT.bin and rename images to follow
# Versal’s boot fallback naming rule.
# 'boot.bin' is used as the primary (main) image,
# while 'boot0001.bin' serves as the fallback (golden) image.
# The boot sequence is evaluated in order:
#     boot.bin → boot0001.bin.
do_install:append() {
	rm -f ${D}/boot/BOOT.bin
	install -m 0644 ${B}/BOOT-main.bin ${D}/boot/boot.bin
	install -m 0644 ${B}/BOOT-golden.bin  ${D}/boot/boot0001.bin
}

do_deploy:append() {
    install -m 0644 ${B}/BOOT-main.bin ${DEPLOYDIR}/${BOOTBIN_BASE_NAME}-main.bin
    install -m 0644 ${B}/BOOT-golden.bin ${DEPLOYDIR}/${BOOTBIN_BASE_NAME}-golden.bin
    ln -sf ${BOOTBIN_BASE_NAME}-main.bin  ${DEPLOYDIR}/BOOT-${MACHINE}-main.bin
    ln -sf ${BOOTBIN_BASE_NAME}-golden.bin  ${DEPLOYDIR}/BOOT-${MACHINE}-golden.bin
    ln -sf ${BOOTBIN_BASE_NAME}-main.bin  ${DEPLOYDIR}/boot-main.bin
    ln -sf ${BOOTBIN_BASE_NAME}-golden.bin  ${DEPLOYDIR}/boot-golden.bin
}
