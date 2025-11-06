SUMMARY = "DualBoot WIC image (main/golden)"
DESCRIPTION = "Builds a single WIC image that contains two root filesystems \
(main and golden), each embedding its own /boot. The 'main' partition is \
used for day-to-day development and frequent updates, while the 'golden' \
partition is a minimal, stable fallback for recovery."
LICENSE = "MIT"

inherit image
inherit image_types_wic

# Only WIC-based output formats are supported
IMAGE_FSTYPES = "wic wic.xz wic.bmap"
WKS_FILES = "sc-dualboot.wks.in"

do_prepare_rootfs_dirs[depends] += " \
    sc-image-main-dev:do_image_complete \
    sc-image-golden-dev:do_image_complete \
"

DUALBOOT_ROOTFS = "${WORKDIR}/dualboot_rootfs"
ROOTFS_DIR_MAIN = "${DUALBOOT_ROOTFS}/main"
ROOTFS_DIR_GOLDEN  = "${DUALBOOT_ROOTFS}/golden"
WICVARS:append = " DEPLOY_DIR_IMAGE MACHINE ROOTFS_DIR_MAIN ROOTFS_DIR_GOLDEN"

# recovering the two root filesystems (main and golden) referenced by .wks.in from a tar.gz archive
addtask prepare_rootfs_dirs before do_rootfs_wicenv
do_prepare_rootfs_dirs() {
    set -e
    rm -rf ${DUALBOOT_ROOTFS}
    mkdir -p ${ROOTFS_DIR_MAIN} ${ROOTFS_DIR_GOLDEN}

    tar -C ${ROOTFS_DIR_MAIN}   -xzf ${DEPLOY_DIR_IMAGE}/sc-image-main-dev-${MACHINE}.tar.gz
    tar -C ${ROOTFS_DIR_GOLDEN} -xzf ${DEPLOY_DIR_IMAGE}/sc-image-golden-dev-${MACHINE}.tar.gz

    # Create empty IMAGE_ROOTFS to satisfy do_image_wic dependencies
    mkdir -p ${IMAGE_ROOTFS}
}

# No additional packages or features are needed in this image recipe
PACKAGES = ""
IMAGE_INSTALL = ""
IMAGE_FEATURES = ""
do_rootfs[noexec] = "1"
do_image[noexec] = "1"
do_install[noexec] = "1"
do_populate_sysroot[noexec] = "1"
