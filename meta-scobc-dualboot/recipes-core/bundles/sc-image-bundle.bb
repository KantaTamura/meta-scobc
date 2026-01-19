SUMMARY = "RAUC bundle (main update): rootfs ext4 + boot vfat"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

inherit bundle

IMAGE_NAME_SUFFIX ?= ""

RAUC_BUNDLE_COMPATIBLE = "${MACHINE}"
RAUC_BUNDLE_DESCRIPTION = "RAUC bundle for sc-image-dev with rootfs and boot partitions"

RAUC_BUNDLE_FORMAT = "verity"

RAUC_BUNDLE_SLOTS = "rootfs boot bootbin/boot.bin"

RAUC_SLOT_rootfs = "sc-image-dev"
RAUC_SLOT_rootfs[fstype] = "ext4"

RAUC_SLOT_boot = "sc-image-dev"
RAUC_SLOT_boot[fstype] = "boot.vfat"

RAUC_SLOT_bootbin/boot.bin = "xilinx-bootbin"
RAUC_SLOT_bootbin/boot.bin[type] = "file"
RAUC_SLOT_bootbin/boot.bin[file] = "boot-main.bin"
