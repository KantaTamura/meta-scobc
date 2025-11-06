# Enable redundant boot configuration (dualboot)
# Build two U-Boot variants: "golden" and "main"
UBOOT_MACHINE = ""
UBOOT_CONFIG = "golden main"

UBOOT_CONFIG[golden]  = "xilinx_versal_dualboot_golden_defconfig"
UBOOT_CONFIG[main] = "xilinx_versal_dualboot_main_defconfig"

# Append 'dualboot' to OVERRIDES so other recipes (e.g. xilinx-bootbin)
# can add dualboot-specific behavior.
OVERRIDES:append = ":dualboot"
