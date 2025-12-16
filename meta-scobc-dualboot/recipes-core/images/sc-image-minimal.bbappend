inherit image_types_bootvfat

IMAGE_FSTYPES:append = " ext4 boot.vfat"
IMAGE_INSTALL:append = " xilinx-bootbin-golden"
