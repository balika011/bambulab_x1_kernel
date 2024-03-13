export CROSS_COMPILE=$PWD/../gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
make ARCH=arm rv1126_defconfig rv1126-X1.config
make ARCH=arm boot.img -j56
