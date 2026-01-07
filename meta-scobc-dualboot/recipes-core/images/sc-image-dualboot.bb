SUMMARY = "DualBoot WIC image (main/golden)"
DESCRIPTION = " \
Builds a single WIC image that contains two root filesystems \
(main and golden), each embedding its own /boot. The 'main' partition is \
used for day-to-day development and frequent updates, while the 'golden' \
partition is a minimal, stable fallback for recovery."
LICENSE = "MIT"

inherit image
inherit image_types_wic

# Only WIC-based output formats are supported
IMAGE_FSTYPES = "wic wic.xz wic.bmap"
WKS_FILES = "sc-dualboot.wks.in"

MAIN_IMAGE_NAME   = "sc-image-dev"
GOLDEN_IMAGE_NAME = "sc-image-minimal"

DUALBOOT_ROOTFS = "${WORKDIR}/dualboot_rootfs"
MAIN_ROOTFS_DIR = "${DUALBOOT_ROOTFS}/main"
GOLDEN_ROOTFS_DIR = "${DUALBOOT_ROOTFS}/golden"

WICVARS:append = " MAIN_ROOTFS_DIR GOLDEN_ROOTFS_DIR"

do_prepare_rootfs_dirs[depends] += " \
    ${MAIN_IMAGE_NAME}:do_image_complete \
    ${GOLDEN_IMAGE_NAME}:do_image_complete \
"

fakeroot do_prepare_rootfs_dirs() {
    set -e
    rm -rf ${DUALBOOT_ROOTFS}
    mkdir -p ${MAIN_ROOTFS_DIR} ${GOLDEN_ROOTFS_DIR}

    tar --numeric-owner --same-owner -C ${MAIN_ROOTFS_DIR}   -xzf ${DEPLOY_DIR_IMAGE}/${MAIN_IMAGE_NAME}-${MACHINE}.tar.gz
    tar --numeric-owner --same-owner -C ${GOLDEN_ROOTFS_DIR} -xzf ${DEPLOY_DIR_IMAGE}/${GOLDEN_IMAGE_NAME}-${MACHINE}.tar.gz

    # Create empty IMAGE_ROOTFS to satisfy do_image_wic dependencies
    mkdir -p ${IMAGE_ROOTFS}
}
addtask prepare_rootfs_dirs before do_rootfs_wicenv

# No additional packages or features are needed in this image recipe
PACKAGES = ""
IMAGE_INSTALL = ""
IMAGE_FEATURES = ""
do_rootfs[noexec] = "1"
do_image[noexec] = "1"
do_install[noexec] = "1"
do_populate_sysroot[noexec] = "1"
