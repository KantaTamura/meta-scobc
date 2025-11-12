FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append:dualboot = " \
	file://99-fpd-watchdog-alias.rules \
	"
