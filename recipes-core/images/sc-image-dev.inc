DESCRIPTION = "An image containing the development set of packages for SC-OBC Module V1"
LICENSE = "MIT"

require sc-image-common.inc

IMAGE_INSTALL:append = " \
    mmc-utils \
    libgpiod \
    libgpiod-tools \
    python3-gpiod \
    "
