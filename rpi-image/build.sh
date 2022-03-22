#!/bin/bash
set -euo pipefail

VERSION="2021.11.2"
DIR="buildroot-$VERSION"
ARCHIVE="$DIR.tar.xz"
URL="https://buildroot.org/downloads/$ARCHIVE"
SHA256SUM=3060005779e5f64bca317f0749ee306e8b02e693b7ccd502b0fd083847e493d2

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
make -j$(($(nproc) - 2)) nginx-rtmp-module  # See https://lists.buildroot.org/pipermail/buildroot/2022-March/638117.html and replies
make -j$(($(nproc) - 2))

echo "==> Done! Find the image at $DIR/output/images/sdcard.img"
