BOOTFS_SIZE ?= "512000"
IMAGE_TYPES:append = " boot.vfat"

oe_mkbootvfatfs () {
    mkfs.vfat $@ -C ${IMGDEPLOYDIR}/${IMAGE_NAME}.boot.vfat ${BOOTFS_SIZE}
    mcopy -i "${IMGDEPLOYDIR}/${IMAGE_NAME}.boot.vfat" -vsmpQ ${IMAGE_ROOTFS}/boot/* ::/
}

IMAGE_CMD:boot.vfat = "oe_mkbootvfatfs ${EXTRA_IMAGECMD}"

DEPENDS:append = " dosfstools-native mtools-native"
