DESCRIPTION = "Development image for SC boards with frequently updated components."
LICENSE = "MIT"

require sc-image-common.inc

IMAGE_NAME_SUFFIX ?= ""

# adding minimal format and packages required for building `sc-image-dualboot.bb`
# - ext4: Will be raw-copied as the rootfs
# - tar.gz: Will be extracted once to retrieve the boot files
IMAGE_FSTYPES:append:dualboot = " ext4 tar.gz"
IMAGE_INSTALL:append:dualboot = " xilinx-bootbin-main u-boot-xlnx-scr-main"
