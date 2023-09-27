#!/bin/bash

function compile() 
{

source ~/.bashrc && source ~/.profile
export LC_ALL=C && export USE_CCACHE=1
ccache -M 100G
export ARCH=arm64
export KBUILD_BUILD_HOST=MARKxDEVS
export KBUILD_BUILD_USER="AbzRaider"
git clone --depth=1  https://gitlab.com/LeCmnGend/proton-clang.git -b clang-13  clang



 if ! [ -d "out" ]; then
echo "Kernel OUT Directory Not Found . Making Again"
mkdir out
fi

make O=out ARCH=arm64 ares_user_defconfig

PATH="${PWD}/clang/bin:${PATH}:${PWD}/clang/bin:${PATH}:${PWD}/clang/bin:${PATH}" \
make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC="clang" \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE="${PWD}/clang/bin/aarch64-linux-gnu-" \
                      CROSS_COMPILE_ARM32="${PWD}/clang/bin/arm-linux-gnueabi-" \
		      LD=ld.lld \
                      STRIP=llvm-strip \
                      AS=llvm-as \
		      AR=llvm-ar \
		      NM=llvm-nm \
		      OBJCOPY=llvm-objcopy \
   		      OBJDUMP=llvm-objdump \
                      CONFIG_NO_ERROR_ON_MISMATCH=y 2>&1 | tee error.log 
}

function zupload()
{
rm -rf AnyKernel	
git clone --depth=1 https://github.com/AbzRaider/AnyKernel33 -b ares AnyKernel
cp out/arch/arm64/boot/Image.gz-dtb AnyKernel
cd AnyKernel
zip -r9 Test-OSS-KERNEL-ARES-S.zip *
curl --upload-file "Test-OSS-KERNEL-ARES-S.zip" https://free.keep.sh
}
compile
zupload
