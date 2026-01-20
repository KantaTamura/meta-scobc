DESCRIPTION = "OSL image definition for SC-OBC Module V1"
LICENSE = "MIT"

require sc-image-dev.inc

IMAGE_FSTYPES:append = " wic wic.bmap wic.xz"

IMAGE_NAME_SUFFIX ?= ""

IMAGE_INSTALL:append = " \
    kernel-module-sc-hls-driver \
    v4l-utils \
"
