########################
##### How to Build #####
########################

(1) Get Toolchain from Codesourcery site. (http://www.codesourcery.com)

    ex) Download : https://sourcery.mentor.com/sgpp/lite/arm/portal/subscription?@template=lite
    
    recommand - Feature : ARM
                Target OS : "EABI"
                Release : "Sourcery G++ Lite 2010q1-188"
                package : "IA32 GNU/Linux TAR"

(2) Compile as follow commands.

    $ cd kernel    
    $ make clean
    $ make mrproper
    $ make ARCH=arm CROSS_COMPILE=../toolchains/arm-2010q1/bin/arm-none-eabi- msm8660_celox_usa_att_defconfig
    $ make ARCH=arm CROSS_COMPILE=../toolchains/arm-2010q1/bin/arm-none-eabi-


(3) Get the zImage on follow path.

    kernel/arch/arm/boot/zImage
