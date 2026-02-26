FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

EXTRA_DT_INCLUDE_FILES:append = " \
    bootargs.dtsi \
    usb.dtsi \
    ethernet.dtsi \
    norflash.dtsi \
    reset.dtsi \
"
