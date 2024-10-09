#source ./compile_live.sh  VERSION ROOTFS.CPIO.GZ DESTINATION_ROOT_FOLDER
#example
#source ./compile_live.sh 001 /AAA/BBB/egf-image-update-imx8mp-egf-3sm1009.rootfs-1.0.cpio.gz  /DDD/CCC

echo $#

display_usage() {
	echo "source ./compile_live.sh  VERSION ROOTFS.CPIO.GZ DESTINATION_FOLDER"
}

if [  $# -ne 3 ]
then
	display_usage
	return
fi


VERSION=$1
cp $2 rootfs.cpio.gz
source /opt/fsl-imx-xwayland/6.6-scarthgap-imx8mm/environment-setup-armv8a-poky-linux
make ARCH=arm64 imx_v8_egf_update_defconfig

sed -i -e 's#CONFIG_LOCALVERSION=""#CONFIG_LOCALVERSION="-'$VERSION'"#' .config
make ARCH=arm64 -j8

echo "-----------------------------------------------------------------------"
echo "--------------    RELEASED VERSION        -----------------------------"
strings arch/arm64/boot/Image | grep 6.6.23 | grep updates | awk -F'/' '{print $5}'
RELDIR=$3/$VERSION-live
mkdir $RELDIR
strings arch/arm64/boot/Image | grep 6.6.23 | grep updates | awk -F'/' '{print $5}' > $RELDIR/kernel_commit.txt
gzip -dc rootfs.cpio.gz | cpio -idv  -D$RELDIR etc/version.gf; mv $RELDIR/etc/version.gf $RELDIR/rootfs.version.txt; rm -rf etc/
cp -v arch/arm64/boot/Image $RELDIR
cp -v arch/arm64/boot/dts/freescale/*egf*.dtb $RELDIR

