#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Kernel configuration front end for the NSING N32H7xx tree.
#
#   kconfig.sh seed             make sure $BUILD_DIR/.config exists and is current
#   kconfig.sh reset            drop the working .config and re-seed it
#   kconfig.sh menuconfig       seed if needed, then run that kbuild target.
#   kconfig.sh <target>         Supported: menuconfig, nconfig, xconfig, config,
#                               oldconfig, olddefconfig, savedefconfig.
#
# Environment (the top-level make sets all of them):
#
#   ARCH=arm
#   CROSS_COMPILE=...           only used to keep kbuild happy; no compiler runs
#   DEFCONFIG=nsing_n32h7xx_defconfig
#                               a name under arch/$ARCH/configs/, or a path
#   BUILD_DIR=<kbuild O= dir>   where .config lives (out/kernel/build)
#   STAGE_DIR=<staged artifacts>  (accepted, unused here)
#
# Why this exists
# ---------------
# The released tree used to re-run "<defconfig>" on every build, which silently
# threw away anything a customer changed in menuconfig.  The rules here are:
#
#   * $BUILD_DIR/.config is the working configuration and it is *kept*;
#   * the shipped defconfig is only the *seed*, applied when there is no
#     .config yet, or on `reset`;
#   * two stamp files make it possible to tell "the release changed its
#     defconfig" from "the customer customised the config":
#         .config-seed     copy of the defconfig that seeded the current .config
#         .config-seeded   the .config exactly as that seed wrote it
#   * when the shipped defconfig changes and the .config is still pristine, it
#     is re-seeded, so a plain `make` keeps following the release;
#   * a customised .config is never overwritten -- the change is reported and
#     the customer decides (`reset` goes back to the shipped configuration).

set -eu

srctree=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)

ARCH=${ARCH:-arm}
CROSS_COMPILE=${CROSS_COMPILE:-arm-none-eabi-}
DEFCONFIG=${DEFCONFIG:-nsing_n32h7xx_defconfig}
BUILD_DIR=${BUILD_DIR:-$srctree}

case $DEFCONFIG in
*/*) seedfile=$DEFCONFIG ;;
*)   seedfile=$srctree/arch/$ARCH/configs/$DEFCONFIG ;;
esac

if [ ! -f "$seedfile" ]; then
	echo "error: no such defconfig: $seedfile" >&2
	echo "       DEFCONFIG is either a name under arch/$ARCH/configs/ or a path." >&2
	exit 1
fi

if [ "$BUILD_DIR" != "$srctree" ]; then
	mkdir -p "$BUILD_DIR"
	make_args="ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE O=$BUILD_DIR"
else
	make_args="ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE"
fi

cfg=$BUILD_DIR/.config
seed_stamp=$BUILD_DIR/.config-seed
pristine_stamp=$BUILD_DIR/.config-seeded

# shellcheck disable=SC2086 # make_args is a deliberately split word list
kbuild() { make -C "$srctree" $make_args "$@"; }

seed_now() {
	echo "==> kconfig: seeding $cfg from $seedfile"
	cp -f "$seedfile" "$cfg"
	kbuild -s olddefconfig
	cp -f "$seedfile" "$seed_stamp"
	cp -f "$cfg" "$pristine_stamp"
}

# Absorb new Kconfig symbols after a kernel update.  If the .config was still
# pristine before the refresh, it stays pristine afterwards, so a later
# defconfig change is still recognised as such.
refresh() {
	pristine=0
	if cmp -s "$pristine_stamp" "$cfg"; then pristine=1; fi
	kbuild -s olddefconfig
	if [ "$pristine" = 1 ]; then cp -f "$cfg" "$pristine_stamp"; fi
}

ensure() {
	if [ ! -f "$cfg" ]; then
		seed_now
	elif [ ! -f "$seed_stamp" ] || [ ! -f "$pristine_stamp" ]; then
		# Build directory from before this helper existed.  Keep whatever
		# configuration is there and start tracking it from now on.
		cp -f "$seedfile" "$seed_stamp"
		cp -f "$cfg" "$pristine_stamp"
		refresh
	elif cmp -s "$seed_stamp" "$seedfile"; then
		refresh
	elif cmp -s "$pristine_stamp" "$cfg"; then
		echo "==> kconfig: $(basename "$seedfile") changed and .config was untouched; re-seeding"
		seed_now
	else
		cat >&2 <<EOF
warning: $seedfile
         changed since $cfg was written, and that .config has your own edits,
         so it has been kept as is.  What you can do:

           make kernel-oldconfig        review the new symbols kbuild added
           make kernel-defconfig        go back to the released configuration
           make kernel-savedefconfig    save your configuration as a defconfig

EOF
		cp -f "$seedfile" "$seed_stamp"
		refresh
	fi
}

usage() {
	sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
}

cmd=${1:-}
if [ -z "$cmd" ]; then
	usage
	exit 2
fi
shift

case $cmd in
seed)
	ensure
	;;
reset)
	echo "==> kconfig: dropping the working configuration"
	seed_now
	;;
oldconfig | olddefconfig)
	# These *are* the refresh, so do not run olddefconfig first: the point of
	# oldconfig is to ask the user about the symbols the kernel just added.
	[ -f "$cfg" ] || seed_now
	kbuild "$cmd" "$@"
	;;
menuconfig | nconfig | xconfig | config)
	ensure
	kbuild "$cmd" "$@"
	echo "==> kconfig: working configuration is $cfg (kept by 'make kernel')"
	;;
savedefconfig)
	ensure
	kbuild savedefconfig
	echo "==> kconfig: minimal defconfig written to $BUILD_DIR/defconfig"
	echo "    to build with it, put it in the kernel tree and name it:"
	echo "        cp $BUILD_DIR/defconfig $srctree/arch/$ARCH/configs/myboard_defconfig"
	echo "        make kernel DEFCONFIG=myboard_defconfig"
	echo "    or point DEFCONFIG straight at the file:"
	echo "        make kernel DEFCONFIG=$BUILD_DIR/defconfig"
	;;
*)
	echo "error: unknown command '$cmd'" >&2
	echo >&2
	usage >&2
	exit 2
	;;
esac
