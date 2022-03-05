Bootstrapping:

    cd buildroot-2021.11.2/
    make BR2_EXTERNAL=.. raspberrypi4_defconfig
    make BR2_EXTERNAL=.. menuconfig
    make BR2_EXTERNAL=.. savedefconfig BR2_DEFCONFIG=../configs/chickendoor_defconfig
