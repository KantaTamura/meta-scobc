# gen-sdt.tcl
#
# Copyright (C) 2026 Space Cubics Inc.
#
# Generate System Device Tree (SDT) from a given XSA file.
#
# Usage:
#   sdtgen gen-sdt.tcl <path-to-xsa> <output-directory>
#
# Example:
#   sdtgen scripts/gen-sdt.tcl meta-scobc-v1/recipes-bsp/hdf/files/sc_obc_v1_versal.xsa meta-scobc-v1/recipes-bsp/sdt/files/versal-scobc-v1-sdt
#
# Arguments:
#   <path-to-xsa>        Path to the exported XSA file
#   <output-directory>   Directory where SDT will be generated
#

if { $argc != 2 } {
    puts "Usage: sdtgen gen-sdt.tcl <path-to-xsa> <output-directory>"
    exit 1
}

set xsa	   [lindex $argv 0]
set outdir [lindex $argv 1]

if {[file exists $outdir]} {
	puts "Output directory '$outdir' already exists. Removing it..."
	file delete -force $outdir
}

# set_dt_param -repo /home/kanta/workspace/sc/meta-scobc/data.local/system-device-tree-xlnx
set_dt_param -debug enable
set_dt_param -board_dts versal-scobc-v1-ve2302i
set_dt_param -dir $outdir
set_dt_param -xsa $xsa

generate_sdt
