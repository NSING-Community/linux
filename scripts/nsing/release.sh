#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Build and stage the NSING N32H7xx ARMv7-M Linux release.

set -eu

srctree=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)

# Two environment knobs decide where things land, both settable by the caller
# (the enclosing project's top-level make points them at <root>/out/kernel so
# nothing is written next to the sources):
#
#   BUILD_DIR     kbuild out-of-tree object tree ($O).  Defaults to an in-tree
#                 build, which is only meant for a standalone kernel checkout.
#   STAGE_DIR     directory that receives the staged boot artifacts: zImage,
#                 <board>.dtb, System.map, config, SHA256SUMS.  The loadable
#                 modules are installed under <STAGE_DIR>/modules.
#   MODULES_DIR   where modules_install puts the .ko tree.  Defaults to
#                 <STAGE_DIR>/modules; the top-level project passes that and
#                 merges it into the rootfs SquashFS image.
#
#   DEFCONFIG     seed configuration: a name under arch/$ARCH/configs/ or a
#                 path.  It only *seeds* $BUILD_DIR/.config; a configuration
#                 the customer edited (make kernel-menuconfig) is kept across
#                 builds.  See scripts/nsing/kconfig.sh for the rules.
#
# The artifacts land flat in STAGE_DIR on purpose.  There is no per-release
# subdirectory and no "latest" symlink, so nothing downstream has to resolve a
# kernel version or a git id to find the image it should flash.
#
# Note that an out-of-tree build (BUILD_DIR != srctree) requires a clean source
# tree: kbuild refuses to run with an in-tree .config / include/config /
# arch/$ARCH/include/generated lying around.  "make mrproper" clears those.
ARCH=${ARCH:-arm}
CROSS_COMPILE=${CROSS_COMPILE:-arm-none-eabi-}
BOARD=${BOARD:-n32h787-hmi}
DEFCONFIG=${DEFCONFIG:-nsing_n32h7xx_defconfig}
BUILD_DIR=${BUILD_DIR:-$srctree}
STAGE_DIR=${STAGE_DIR:-$srctree/release}
MODULES_DIR=${MODULES_DIR:-$STAGE_DIR/modules}

make_args="ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE"
if [ "$BUILD_DIR" != "$srctree" ]; then
	make_args="$make_args O=$BUILD_DIR"
fi

# The configuration is seeded from $DEFCONFIG the first time and then kept, so
# changes made through menuconfig survive the next build; kconfig.sh also
# reports when the shipped defconfig moved on.  This used to be a plain
# "$DEFCONFIG" target, which reverted every customer edit on each build.
ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE DEFCONFIG=$DEFCONFIG BUILD_DIR=$BUILD_DIR \
	"$srctree/scripts/nsing/kconfig.sh" seed

make -C "$srctree" $make_args zImage dtbs -j"${JOBS:-$(nproc)}"

# Install the loadable modules into a tree that mirrors what the board expects
# ($MODULES_DIR/lib/modules/<release>/), including modules.dep, so the rootfs
# packaging can merge it in and the board's modprobe can resolve dependencies.
# The shipped defconfig builds everything in, so today this stages an empty
# tree; it is what an out-of-tree module or a product configuration that ships
# a .ko relies on.  The directory is rebuilt from scratch because
# modules_install only adds, so a symbol that turned into built-in code would
# otherwise leave a stale .ko behind.
case "$MODULES_DIR" in
*/modules) ;;
*) echo "release.sh: refusing to clean unexpected MODULES_DIR '$MODULES_DIR'" >&2
   exit 1 ;;
esac
rm -rf "$MODULES_DIR"
make -C "$srctree" $make_args modules -j"${JOBS:-$(nproc)}"
make -C "$srctree" $make_args INSTALL_MOD_PATH="$MODULES_DIR" \
	INSTALL_MOD_STRIP=1 modules_install
# modules_install points <release>/build and <release>/source at the build
# tree; those belong to a developer's machine, not to a shipped rootfs.
rm -f "$MODULES_DIR"/lib/modules/*/build "$MODULES_DIR"/lib/modules/*/source

outdir=$STAGE_DIR
mkdir -p "$outdir"

# Clear only the files we stage.  In the combined project STAGE_DIR is
# out/kernel, which also holds the out-of-tree build tree in out/kernel/build,
# so this must never be a blanket removal.
for staged in zImage "$BOARD.dtb" System.map config SHA256SUMS; do
	rm -f "$outdir/$staged"
done

if [ "$BUILD_DIR" = "$srctree" ]; then
	bindir="$srctree/arch/arm/boot"
	dtbdir="$bindir/dts/nsing"
else
	bindir="$BUILD_DIR/arch/arm/boot"
	dtbdir="$bindir/dts/nsing"
fi

image="$bindir/zImage"
dtb="$dtbdir/$BOARD.dtb"

test -s "$image"
test -s "$dtb"

cp "$image" "$outdir/zImage"
cp "$dtb" "$outdir/$BOARD.dtb"
if [ "$BUILD_DIR" != "$srctree" ]; then
	cp "$BUILD_DIR/System.map" "$outdir/System.map"
	cp "$BUILD_DIR/.config" "$outdir/config"
else
	cp "$srctree/System.map" "$outdir/System.map"
	cp "$srctree/.config" "$outdir/config"
fi

(cd "$outdir" && sha256sum zImage "$BOARD.dtb" \
	System.map config > SHA256SUMS)

# The version is only reported, never used to build a path.
printf 'kernel:  %s\n' "$(make -s -C "$srctree" $make_args kernelrelease)"
printf 'stage:   %s\n' "$outdir"
printf 'zImage:  %s (%s bytes)\n' "$outdir/zImage" "$(wc -c <"$outdir/zImage")"
printf 'dtb:     %s (%s bytes)\n' "$outdir/$BOARD.dtb" "$(wc -c <"$outdir/$BOARD.dtb")"
printf 'modules: %s (%s .ko)\n' "$MODULES_DIR" \
	"$(find "$MODULES_DIR" -name '*.ko' | wc -l)"
