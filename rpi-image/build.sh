#!/bin/bash
set -euo pipefail

VERSION="2022.11"
DIR="buildroot-$VERSION"
ARCHIVE="$DIR.tar.xz"
URL="https://buildroot.org/downloads/$ARCHIVE"
SHA256SUM=e3a077182c3eb5e71c86906696fac3aabb7b4d93815c9a69762231eac2be530d

echo -e "Building Chicken Door Image\n"

echo "==> Downloading buildroot"
if [ ! -f $ARCHIVE ]; then
    wget $URL
fi

echo "==> Verifying checksum"
if [ $SHA256SUM != "$(sha256sum $ARCHIVE | cut -d' ' -f 1)" ]; then
    echo "ERROR: Invalid checksum"
    exit 1
fi

echo "==> Unpacking buildroot"
tar xf $ARCHIVE

echo "==> Build!"
cd $DIR
make BR2_EXTERNAL=.. chickendoor_defconfig
make -j$(($(nproc) - 2))

echo "==> Done! Find the image at $DIR/output/images/sdcard.img"
