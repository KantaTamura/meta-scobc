DESCRIPTION = "Stable fallback image for SC boards, containing only essential components."
LICENSE = "MIT"

require sc-image-common.inc

IMAGE_NAME_SUFFIX ?= ""

# This specifies only the minimum required formats for building `sc-image-dualboot.bb`:
# - ext4: Will be raw-copied as the rootfs
# - tar.gz: Will be extracted once to retrieve the boot files
IMAGE_FSTYPES:dualboot = "ext4 tar.gz"
IMAGE_INSTALL:append:dualboot = " xilinx-bootbin-golden u-boot-xlnx-scr-golden"
