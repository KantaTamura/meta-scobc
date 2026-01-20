FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

EXTRA_DT_INCLUDE_FILES:append = " \
    bootargs.dtsi \
    usb.dtsi \
    ethernet.dtsi \
    norflash.dtsi \
"

EXTRA_DT_INCLUDE_FILES:append = " \
    hls.dtsi \
"

EXTRA_DT_INCLUDE_FILES:remove:versal-scobc-v1-sdt-full-microblaze-pmc = " \
    hls.dtsi \
"
EXTRA_DT_INCLUDE_FILES:remove:versal-scobc-v1-sdt-full-microblaze-psm = " \
    hls.dtsi \
"
