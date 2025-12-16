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

MAIN_ROOTFS_EXT4_PATH   = "${DEPLOY_DIR_IMAGE}/${MAIN_IMAGE_NAME}-${MACHINE}.ext4"
MAIN_BOOT_VFAT_PATH     = "${DEPLOY_DIR_IMAGE}/${MAIN_IMAGE_NAME}-${MACHINE}.vfat.boot"
GOLDEN_ROOTFS_EXT4_PATH = "${DEPLOY_DIR_IMAGE}/${GOLDEN_IMAGE_NAME}-${MACHINE}.ext4"
GOLDEN_BOOT_VFAT_PATH   = "${DEPLOY_DIR_IMAGE}/${GOLDEN_IMAGE_NAME}-${MACHINE}.vfat.boot"
WICVARS:append = " MAIN_ROOTFS_EXT4_PATH MAIN_BOOT_VFAT_PATH GOLDEN_ROOTFS_EXT4_PATH GOLDEN_BOOT_VFAT_PATH"

do_prepare_rootfs_dirs[depends] += " \
    ${MAIN_IMAGE_NAME}:do_image_complete \
    ${GOLDEN_IMAGE_NAME}:do_image_complete \
"

# Create empty IMAGE_ROOTFS to satisfy do_image_wic dependencies
do_prepare_rootfs_dirs() {
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
