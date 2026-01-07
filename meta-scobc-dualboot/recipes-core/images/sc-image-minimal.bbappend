inherit image_types_bootvfat

IMAGE_FSTYPES:append = " tar.gz ext4 boot.vfat"
IMAGE_INSTALL:append = " xilinx-bootbin-golden u-boot-xlnx-scr-golden"
IMAGE_INSTALL:append = " multiboot-tool mark-boot-success boot-slot-switch"
IMAGE_INSTALL:append = " rauc e2fsprogs e2fsprogs-resize2fs parted"
