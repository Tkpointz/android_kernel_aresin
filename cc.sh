export ANDROID_MAJOR_VERSION=r
make O=out ARCH=arm64 ares_user_defconfig
#make O=out ARCH=arm64 menuconfig
PATH="/media/linux/linux/compilers/clang-r383902/bin:/media/linux/linux/compilers/toolchain/bin:/media/linux/linux/compilers/toolchain-arm/bin:${PATH}" \
make -j$(nproc --all) O=out \
		      ARCH=arm64 \
                      CC=clang \
                      CROSS_COMPILE=aarch64-linux-android-  \
                      CROSS_COMPILE_ARM32=arm-linux-androideabi- \
		      CLANG_TRIPLE=aarch64-linux-gnu- \
		      2>&1 | tee error.log
