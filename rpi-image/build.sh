#!/bin/bash
set -euo pipefail

VERSION="2022.02"
DIR="buildroot-$VERSION"
ARCHIVE="$DIR.tar.xz"
URL="https://buildroot.org/downloads/$ARCHIVE"
SHA256SUM=8161c43dc6a11c0bc86588c09a8e1dc935b28ca046447ad02bf7074064456701

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
