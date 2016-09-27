#!/bin/bash
set -e
#########################################################################
#
#          Simple build scripts to build krenel(with rootfs)
#		AW-SoM Technologies 2014 aw-som.com
#
#########################################################################


#Setup common variables
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export AS=${CROSS_COMPILE}as
export LD=${CROSS_COMPILE}ld
export CC=${CROSS_COMPILE}gcc
export AR=${CROSS_COMPILE}ar
export NM=${CROSS_COMPILE}nm
export STRIP=${CROSS_COMPILE}strip
export OBJCOPY=${CROSS_COMPILE}objcopy
export OBJDUMP=${CROSS_COMPILE}objdump

PLATFORM=$1
MODULE_OUT="output/${PLATFORM}"
USE_DEFCONF=false
KDIR=`pwd`
LICHEE_MOD_DIR=${KDIR}/output/lib/modules/${KERNEL_VERSION}

show_help()
{
	printf "
Build script for aw-som platform

Valid Options:

	help         - show this help
	kernel       - build kernel
	modules      - build kernel module in modules dir
	clean        - clean kernel and modules

"
}

build_kernel()
{
	if [ -e .config ]; then
		rm .config
	fi
        if [ ! -e .config.${PLATFORM} ]; then
		if [ ! -e arch/arm/configs/${PLATFORM}_defconfig ]; then
			echo -e "\n\t\tInvalid platform ${PLATFORM}!\n"
			exit 1
		fi
		echo -e "\n\t\tUsing ${PLATFORM}_defconfig config... ...!\n"
                cp arch/arm/configs/${PLATFORM}_defconfig .config.${PLATFORM}
		ln -s .config.${PLATFORM} .config
	else
		echo -e "\n\t\tUsing previous ${PLATFROM} config file .config.${PLATFORM}"
		ln -s .config.${PLATFORM} .config
        fi
	make ARCH=${ARCH} INSTALL_MOD_PATH=${MODULE_OUT} CROSS_COMPILE=${CROSS_COMPILE} -j8 uImage modules
	make ARCH=${ARCH} INSTALL_MOD_PATH=${MODULE_OUT} CROSS_COMPILE=${CROSS_COMPILE} -j8 modules_install
	cp arch/arm/boot/uImage ${MODULE_OUT}
	#${OBJCOPY} -R .note.gnu.build-id -S -O binary vmlinux output/bImage
	#cp -vf arch/arm/boot/[zu]Image output/
	#cp .config output/
	#cp rootfs/sun4i_rootfs.cpio.gz output/

	#mkbootimg --kernel output/bImage \
	#		--ramdisk output/sun4i_rootfs.cpio.gz \
	#		--board 'sun4i' \
	#		--base 0x40000000 \
	#		-o output/boot.img
}

build_modules()
{

        return;
	echo "Building modules"

	if [ ! -f include/generated/utsrelease.h ]; then
		printf "Please build kernel first!\n"
		exit 1
	fi

	make -C modules/example LICHEE_MOD_DIR=${MODULE_OUT} LICHEE_KDIR=${KDIR} \
		install

	(
	export LANG=en_US.UTF-8
	unset LANGUAGE
	make -C modules/mali LICHEE_MOD_DIR=${MODULE_OUT} LICHEE_KDIR=${LICHEE_KDIR} \
		install
	)
}

clean_kernel()
{
	make clean
	rm -rf output/*
}

clean_modules()
{
	echo "Cleaning modules"
	make -C modules/example LICHEE_MOD_DIR=${MODULE_OUT} LICHEE_KDIR=${KDIR} clean

	(
	export LANG=en_US.UTF-8
	unset LANGUAGE
	make -C modules/mali LICHEE_MOD_DIR=${MODULE_OUT} LICHEE_KDIR=${KDIR} clean
	)
}

#####################################################################
#
#                      Main Runtine
#
#####################################################################

case "$2" in
kernel)
	build_kernel
	;;
modules)
	build_modules
	;;
clean)
	clean_kernel
	clean_modules
	;;
all)
	build_kernel
	build_modules
	;;
*)
	show_help
	;;
esac

