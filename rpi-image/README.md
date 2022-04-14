# Chicken Cam

> «Hot chicks in your area!»

Two webcams on a Raspberry Pi livestreamed straight to the browser using WebRTC
(backed by Janus as media server).

The Raspberry Pi image is built using buildroot, using the `BR2_EXTERNAL` feature.

The Janus config can be found at `board/overlay/etc/janus/`. The website used
to display the streamed webcams is at `board/overlay/srv/www/`.

To build:

    ./build.sh

To flash:

    sudo dd if=buildroot-2022.02/output/images/sdcard.img of=/dev/disk/<your-disk> bs=8M
