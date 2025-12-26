inherit image_types_bootvfat

IMAGE_FSTYPES:append = " ext4 boot.vfat"
IMAGE_INSTALL:append = " xilinx-bootbin-golden u-boot-xlnx-scr-golden"
IMAGE_INSTALL:append = " multiboot-tool mark-boot-success boot-slot-switch"
