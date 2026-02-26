# - The traditional way to add settings to U-Boot's .config is to add cfg files to SRC_URI.
#   However, this approach does not work well with multiple U-Boot builds using UBOOT_CONFIG,
#   where each configuration needs different additional settings.
# - Introduce a new variable, UBOOT_EXTRA_CFGS.
#   Common cfg files are specified in UBOOT_EXTRA_CFGS, while per-config (per-type) cfg files
#   are specified in UBOOT_EXTRA_CFGS[type].
# - Ideally, the build directory should be separated per type, but doing so in a bbappend
#   is difficult, so this is a pragmatic workaround.

# Automatically add cfg files specified in UBOOT_EXTRA_CFGS to SRC_URI
python __anonymous() {
    cfgs = (d.getVar("UBOOT_EXTRA_CFGS") or "").split()
    flags = d.getVarFlags("UBOOT_EXTRA_CFGS") or {}
    for v in flags.values():
        cfgs += (v or "").split()

    cfgs = [c for c in cfgs if c]
    if not cfgs:
        return

    src = d.getVar("SRC_URI") or ""
    for c in dict.fromkeys(cfgs):
        token = "file://%s" % c
        if token not in src:
            d.appendVar("SRC_URI", " " + token)
}

# Store variant-specific UBOOT_EXTRA_CFGS in UBOOT_EXTRA_CFGS_MAP so that they can be
# accessed from uboot_configure_config.
# The format is: "key=value1 value2;..."
python __anonymous() {
    flags = d.getVarFlags("UBOOT_EXTRA_CFGS") or {}

    items = []
    for k in sorted(flags.keys()):
        v = (flags.get(k) or "").strip()
        if v:
            items.append("%s=%s" % (k, v))

    d.setVar("UBOOT_EXTRA_CFGS_MAP", ";".join(items))
}

# Extract cfg paths for the target type from UBOOT_EXTRA_CFGS_MAP and pass them to
# merge_config.sh.
# Also save the generated .config per type as .config-${type}.
uboot_configure_config () {
    config=$1
    type=$2

    oe_runmake -C ${S} O=${B}/${config} ${config}

    variant_cfgs="$(printf '%s\n' "${UBOOT_EXTRA_CFGS_MAP}" | tr ';' '\n' \
        | awk -F= -v k="$type" '$1==k {sub($1"=",""); print; exit}')"

    extra_cfgs=""
    for c in ${UBOOT_EXTRA_CFGS} ${variant_cfgs}; do
        [ -n "$c" ] && extra_cfgs="$extra_cfgs ${WORKDIR}/$c"
    done

    if [ -n "$extra_cfgs" ]; then
        bbnote "u-boot(${type}): extra_cfgs=${extra_cfgs}"
        merge_config.sh -m -O ${B}/${config} ${B}/${config}/.config $extra_cfgs
        oe_runmake -C ${S} O=${B}/${config} oldconfig
    fi

    cp -f ${B}/${config}/.config ${B}/${config}/.config-${type}
}

# Treat .config-${type} as the active .config and regenerate related files using oldconfig
uboot_compile_config () {
    i=$1
    config=$2
    type=$3

    if [ ! -e ${B}/${config}/.config-${type} ]; then
        bbfatal "Missing ${B}/${config}/.config-${type}"
    fi

    cp -f ${B}/${config}/.config-${type} ${B}/${config}/.config
    oe_runmake -C ${S} O=${B}/${config} oldconfig

    oe_runmake -C ${S} O=${B}/${config} ${UBOOT_MAKE_TARGET}

    unset k
    for binary in ${UBOOT_BINARIES}; do
        k=$(expr $k + 1);
        if [ $k -eq $i ]; then
            uboot_compile_config_copy_binary $config $type $binary
        fi
    done
    unset k

	# Append a type to copy the file to prevent overwriting the generated ELF file
	install -m 644 ${B}/${config}/${UBOOT_ELF} ${B}/${config}/u-boot-${type}.${UBOOT_ELF_SUFFIX}

    # Generate the uboot-initial-env
    if [ -n "${UBOOT_INITIAL_ENV}" ]; then
        oe_runmake -C ${S} O=${B}/${config} u-boot-initial-env
        cp ${B}/${config}/u-boot-initial-env ${B}/${config}/u-boot-initial-env-${type}
    fi
}

# In addition to the content of `u-boot-elf-install-fix.inc`, it also modifies the source path for installation.
uboot_install_elf_config () {
    config=$1
    type=$2

    install -m 644 ${B}/${config}/u-boot-${type}.${UBOOT_ELF_SUFFIX} ${D}/boot/u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX}
    ln -sf u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX} ${D}/boot/${UBOOT_ELF_BINARY}-${type}
    ln -sf u-boot-${type}-${PV}-${PR}.${UBOOT_ELF_SUFFIX} ${D}/boot/${UBOOT_ELF_BINARY}
}
