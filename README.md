### bl-hi3798c

Usage:
1. Get aarch64 toolchain from `https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu.tar.xz`
2. Get armhf toolchain from `https://releases.linaro.org/components/toolchain/binaries/latest-7/arm-linux-gnueabihf/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf.tar.xz`
3. Extract toolchain, setup PATH
4. for imou-sn1, Run `BOARD=IMOU-SN1 ./build.sh` to build normal l-loader, run `BOARD=IMOU-SN1 ./build.sh RECOVERY` to build recovery image
5. for DSH4904, Run `BOARD=DSH4904 ./build.sh` to build normal l-loader, run `BOARD=DSH4904 ./build.sh RECOVERY` to build recovery image
