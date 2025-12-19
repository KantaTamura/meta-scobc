FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " \
    file://boot.cmd.dualboot \
"

# use PARTUUID-based root parameter instead of /dev/mmcblkXpY
KERNEL_ROOT_SD:remove = "root=/dev/\${bootdev}${PARTNUM}"
KERNEL_ROOT_SD:prepend = "root=PARTUUID=\${rootuuid} "

do_compile:append() {
    emit_variant_cmd() {
        variant="$1"
        rootfs_label="$2"
        bootscript_name="$3"

        # Most @@VAR@@ placeholders are expanded from BitBake variables.
        # Only @@ROOT_PARTITION_LABEL@@ uses the shell variable $rootfs_label,
        # which differs per variant (e.g. "rootm", "rootg").
        sed -e 's/@@KERNEL_IMAGETYPE@@/${KERNEL_IMAGETYPE}/' \
            -e 's/@@KERNEL_LOAD_ADDRESS@@/${KERNEL_LOAD_ADDRESS}/' \
            -e 's/@@DEVICE_TREE_NAME@@/${DEVICE_TREE_NAME}/' \
            -e 's/@@DEVICETREE_ADDRESS@@/${DEVICETREE_ADDRESS}/' \
            -e 's/@@DEVICETREE_ADDRESS_SD@@/${DEVICETREE_ADDRESS_SD}/' \
            -e 's/@@DEVICETREE_OVERLAY_ADDRESS@@/${DEVICETREE_OVERLAY_ADDRESS}/' \
            -e 's/@@RAMDISK_IMAGE@@/${RAMDISK_IMAGE}/' \
            -e 's/@@RAMDISK_IMAGE_ADDRESS@@/${RAMDISK_IMAGE_ADDRESS}/' \
            -e 's/@@KERNEL_BOOTCMD@@/${KERNEL_BOOTCMD}/' \
            -e 's/@@SDBOOTDEV@@/${SDBOOTDEV}/' \
            -e 's/@@BITSTREAM@@/${@boot_files_bitstream(d)[0]}/g' \
            -e 's/@@BITSTREAM_LOAD_ADDRESS@@/${BITSTREAM_LOAD_ADDRESS}/g' \
            -e 's/@@BITSTREAM_IMAGE@@/${@boot_files_bitstream(d)[0]}/g' \
            -e 's/@@BITSTREAM_LOAD_TYPE@@/${@get_bitstream_load_type(d)}/g' \
            -e 's/@@QSPI_KERNEL_OFFSET@@/${QSPI_KERNEL_OFFSET}/' \
            -e 's/@@NAND_KERNEL_OFFSET@@/${NAND_KERNEL_OFFSET}/' \
            -e 's/@@QSPI_KERNEL_SIZE@@/${QSPI_KERNEL_SIZE}/' \
            -e 's/@@NAND_KERNEL_SIZE@@/${NAND_KERNEL_SIZE}/' \
            -e 's/@@QSPI_RAMDISK_OFFSET@@/${QSPI_RAMDISK_OFFSET}/' \
            -e 's/@@NAND_RAMDISK_OFFSET@@/${NAND_RAMDISK_OFFSET}/' \
            -e 's/@@QSPI_RAMDISK_SIZE@@/${QSPI_RAMDISK_SIZE}/' \
            -e 's/@@NAND_RAMDISK_SIZE@@/${NAND_RAMDISK_SIZE}/' \
            -e 's/@@KERNEL_IMAGE@@/${KERNEL_IMAGE}/' \
            -e 's/@@QSPI_KERNEL_IMAGE@@/${QSPI_KERNEL_IMAGE}/' \
            -e 's/@@NAND_KERNEL_IMAGE@@/${NAND_KERNEL_IMAGE}/' \
            -e 's/@@FIT_IMAGE_LOAD_ADDRESS@@/${FIT_IMAGE_LOAD_ADDRESS}/' \
            -e 's/@@QSPI_FIT_IMAGE_OFFSET@@/${QSPI_FIT_IMAGE_OFFSET}/' \
            -e 's/@@QSPI_FIT_IMAGE_SIZE@@/${QSPI_FIT_IMAGE_SIZE}/' \
            -e 's/@@NAND_FIT_IMAGE_OFFSET@@/${NAND_FIT_IMAGE_OFFSET}/' \
            -e 's/@@NAND_FIT_IMAGE_SIZE@@/${NAND_FIT_IMAGE_SIZE}/' \
            -e 's/@@FIT_IMAGE@@/${FIT_IMAGE}/' \
            -e 's/@@PRE_BOOTENV@@/${PRE_BOOTENV}/' \
            -e 's/@@UENV_MMC_LOAD_ADDRESS@@/${UENV_MMC_LOAD_ADDRESS}/' \
            -e 's/@@UENV_TEXTFILE@@/${UENV_TEXTFILE}/' \
            -e 's/@@RAMDISK_IMAGE1@@/${RAMDISK_IMAGE1}/' \
            -e 's/@@PARTNUM@@/${PARTNUM}/' \
            -e 's:@@KERNEL_ROOT_SD@@:${KERNEL_ROOT_SD}:' \
            -e 's:@@KERNEL_ROOT_RAMDISK@@:${KERNEL_ROOT_RAMDISK}:' \
            -e 's:@@KERNEL_COMMAND_APPEND@@:${KERNEL_COMMAND_APPEND}:' \
            -e "s/@@ROOT_PARTITION_LABEL@@/${rootfs_label}/" \
            -e "s/@@BOOT_SCRIPT_NAME@@/${bootscript_name}/" \
            ${SCRIPT_SED_ADDENDUM} \
            "${WORKDIR}/boot.cmd.dualboot" > "${WORKDIR}/boot.cmd-${variant}"
    }

    # create boot script for main image
    emit_variant_cmd "main" "rootm" "boot.scr-main"
    mkimage -A arm -T script -C none -n "Dualboot boot script (main)" -d "${WORKDIR}/boot.cmd-main" boot.scr-main

    # create boot script for golden image
    emit_variant_cmd "golden" "rootg" "boot.scr-golden"
    mkimage -A arm -T script -C none -n "Dualboot boot script (golden)"  -d "${WORKDIR}/boot.cmd-golden"  boot.scr-golden
}

do_install:append() {
    # remove default boot.scr to avoid conflict
    rm -f ${D}/boot/boot.scr
    rm -f ${D}/boot/${UBOOTSCR_BASE_NAME}.scr

    install -m 0644 boot.scr-main ${D}/boot/${UBOOTSCR_BASE_NAME}.scr-main
    install -m 0644 boot.scr-main ${D}/boot/

    install -m 0644 boot.scr-golden ${D}/boot/${UBOOTSCR_BASE_NAME}.scr-golden
    install -m 0644 boot.scr-golden ${D}/boot/
}

do_deploy:append() {
    install -m 0644 boot.scr-main ${DEPLOYDIR}/${UBOOTSCR_BASE_NAME}.scr-main
    install -m 0644 boot.scr-main ${DEPLOYDIR}/

    install -m 0644 boot.scr-golden ${DEPLOYDIR}/${UBOOTSCR_BASE_NAME}.scr-golden
    install -m 0644 boot.scr-golden ${DEPLOYDIR}/
}

# Add three related packages for dualboot configuration:
# - ${PN}           : base package containing common PXE boot files
# - ${PN}-main      : provides boot.scr-main for the main image
# - ${PN}-golden    : provides boot.scr-golden  for the golden image
#
# Both variant packages depend on the base ${PN} package to ensure
# shared PXE configuration files are installed. This allows each
# rootfs (main/golden) to include the correct boot script while
# reusing the same PXE setup.
PACKAGES:append = " ${PN}-main ${PN}-golden"

# ensure base u-boot-xlnx-scr is also installed
RDEPENDS:${PN}-main:append   = " u-boot-xlnx-scr"
RDEPENDS:${PN}-golden:append = " u-boot-xlnx-scr"

FILES:${PN} = " \
	/boot/pxeboot/*/default \
    /boot/${UBOOTPXE_CONFIG}/default \
"
FILES:${PN}-main = " \
    /boot/*.scr-main \
"
FILES:${PN}-golden = " \
    /boot/*.scr-golden \
"

# postinstall: explicitly copy variant boot script to boot.scr
# (update-alternatives was avoided because symbolic links inside the rootfs
#  caused resolution issues during image creation)
pkg_postinst:${PN}-main() {
    if [ -n "$D" ]; then
        dest="$D/boot"
    else
        dest="/boot"
    fi
    if [ -d "$dest" ]; then
        cp -f "$dest/boot.scr-main" "$dest/boot.scr"
    fi
}

pkg_postinst:${PN}-golden() {
    if [ -n "$D" ]; then
        dest="$D/boot"
    else
        dest="/boot"
    fi
    if [ -d "$dest" ]; then
        cp -f "$dest/boot.scr-golden" "$dest/boot.scr"
    fi
}
