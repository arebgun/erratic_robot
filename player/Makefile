VERSION = player_svn-r7125
TARBALL = build/$(VERSION).tar.bz2
TARBALL_URL = http://download.ros.org/downloads/$(VERSION).tar.bz2
SOURCE_DIR = build/player-svn
TARBALL_PATCH = cmake-2.4-ros.patch
UNPACK_CMD = tar xf

SVN_DIR = player-svn
SVN_URL = https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/player/trunk
SVN_REVISION = -r 7125
#include $(shell rospack find mk)/svn_checkout.mk

all: installed

include $(shell rospack find mk)/download_unpack_build.mk


CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=`rospack find player`/player \
	     -D INCLUDE_RTK=OFF \
	     -D BUILD_PLAYERCC=OFF \
	     -D BUILD_UTILS=OFF \
	     -D BUILD_EXAMPLES=OFF \
	     -D PLAYER_BUILD_TESTS=OFF \
	     -D BUILD_PYTHON_BINDINGS=OFF \
             -D ENABLE_DRIVER_ACCEL_CALIB=OFF \
             -D ENABLE_DRIVER_ACR120U=OFF \
             -D ENABLE_DRIVER_ACTS=OFF \
             -D ENABLE_DRIVER_AIOTOSONAR=OFF \
             -D ENABLE_DRIVER_ALSA=OFF \
             -D ENABLE_DRIVER_AMCL=ON \
             -D ENABLE_DRIVER_AMTECM5=OFF \
             -D ENABLE_DRIVER_AMTECPOWERCUBE=OFF \
             -D ENABLE_DRIVER_AODV=OFF \
             -D ENABLE_DRIVER_ARTOOLKITPLUS=OFF \
             -D ENABLE_DRIVER_BUMPER2LASER=OFF \
             -D ENABLE_DRIVER_BUMPERSAFE=OFF \
             -D ENABLE_DRIVER_CAMERA1394=OFF \
             -D ENABLE_DRIVER_CAMERACOMPRESS=OFF \
             -D ENABLE_DRIVER_CAMERAUNCOMPRESS=OFF \
             -D ENABLE_DRIVER_CAMERAUVC=OFF \
             -D ENABLE_DRIVER_CAMERAV4L=OFF \
             -D ENABLE_DRIVER_CANONVCC4=OFF \
             -D ENABLE_DRIVER_CHATTERBOX=OFF \
             -D ENABLE_DRIVER_CLODBUSTER=OFF \
             -D ENABLE_DRIVER_CMUCAM2=OFF \
             -D ENABLE_DRIVER_CMVISION=OFF \
             -D ENABLE_DRIVER_CREATE=OFF \
             -D ENABLE_DRIVER_CVCAM=OFF \
             -D ENABLE_DRIVER_DUMMY=OFF \
             -D ENABLE_DRIVER_EEDHCONTROLLER=OFF \
             -D ENABLE_DRIVER_ER1=OFF \
             -D ENABLE_DRIVER_ERRATIC=ON \
             -D ENABLE_DRIVER_FAKELOCALIZE=OFF \
             -D ENABLE_DRIVER_FESTIVAL=OFF \
             -D ENABLE_DRIVER_FLEXIPORT=OFF \
             -D ENABLE_DRIVER_FLOCKOFBIRDS=OFF \
             -D ENABLE_DRIVER_GARCIA=OFF \
             -D ENABLE_DRIVER_GARMINNMEA=OFF \
             -D ENABLE_DRIVER_GBXSICKACFR=OFF \
             -D ENABLE_DRIVER_HOKUYO_AIST=OFF \
             -D ENABLE_DRIVER_IMAGESEQ=OFF \
             -D ENABLE_DRIVER_INSIDEM300=OFF \
             -D ENABLE_DRIVER_ISENSE=OFF \
             -D ENABLE_DRIVER_IWSPY=OFF \
             -D ENABLE_DRIVER_KARTOWRITER=OFF \
             -D ENABLE_DRIVER_KHEPERA=OFF \
             -D ENABLE_DRIVER_LASERBAR=OFF \
             -D ENABLE_DRIVER_LASERBARCODE=OFF \
             -D ENABLE_DRIVER_LASERCSPACE=OFF \
             -D ENABLE_DRIVER_LASERCUTTER=OFF \
             -D ENABLE_DRIVER_LASERPOSEINTERPO=OFF \
             -D ENABLE_DRIVER_LASERPTZCLOUD=OFF \
             -D ENABLE_DRIVER_LASERRESCAN=OFF \
             -D ENABLE_DRIVER_LASERSAFE=OFF \
             -D ENABLE_DRIVER_LASERTORANGER=OFF \
             -D ENABLE_DRIVER_LASERVISUALBARCO=OFF \
             -D ENABLE_DRIVER_LASERVISUALBW=OFF \
             -D ENABLE_DRIVER_LIFOMCOM=OFF \
             -D ENABLE_DRIVER_LINUXJOYSTICK=OFF \
             -D ENABLE_DRIVER_LINUXWIFI=OFF \
             -D ENABLE_DRIVER_LOCALBB=OFF \
             -D ENABLE_DRIVER_MAPCSPACE=OFF \
             -D ENABLE_DRIVER_MAPFILE=OFF \
             -D ENABLE_DRIVER_MAPSCALE=OFF \
             -D ENABLE_DRIVER_MBICP=OFF \
             -D ENABLE_DRIVER_MICA2=OFF \
             -D ENABLE_DRIVER_MICROSTRAIN=OFF \
             -D ENABLE_DRIVER_MOTIONMIND=OFF \
             -D ENABLE_DRIVER_MRICP=OFF \
             -D ENABLE_DRIVER_ND=OFF \
             -D ENABLE_DRIVER_NIMU=OFF \
             -D ENABLE_DRIVER_NOMAD=OFF \
             -D ENABLE_DRIVER_OBOT=OFF \
             -D ENABLE_DRIVER_P2OS=ON \
             -D ENABLE_DRIVER_PASSTHROUGH=OFF \
             -D ENABLE_DRIVER_PBSLASER=OFF \
             -D ENABLE_DRIVER_PHIDGETACC=OFF \
             -D ENABLE_DRIVER_PHIDGETIFK=OFF \
             -D ENABLE_DRIVER_PHIDGETRFID=OFF \
             -D ENABLE_DRIVER_POSTGIS=OFF \
             -D ENABLE_DRIVER_PTU46=OFF \
             -D ENABLE_DRIVER_RCORE_XBRIDGE=OFF \
             -D ENABLE_DRIVER_READLOG=OFF \
             -D ENABLE_DRIVER_REB=OFF \
             -D ENABLE_DRIVER_RELAY=OFF \
             -D ENABLE_DRIVER_RFLEX=OFF \
             -D ENABLE_DRIVER_ROBOTEQ=OFF \
             -D ENABLE_DRIVER_ROBOTINO=OFF \
             -D ENABLE_DRIVER_ROOMBA=OFF \
             -D ENABLE_DRIVER_RS4LEUZE=OFF \
             -D ENABLE_DRIVER_SEGWAYRMP=OFF \
             -D ENABLE_DRIVER_SERIALSTREAM=OFF \
             -D ENABLE_DRIVER_SERVICE_ADV_MDNS=OFF \
             -D ENABLE_DRIVER_SHAPETRACKER=OFF \
             -D ENABLE_DRIVER_SICKLMS200=OFF \
             -D ENABLE_DRIVER_SICKLMS400=OFF \
             -D ENABLE_DRIVER_SICKNAV200=OFF \
             -D ENABLE_DRIVER_SICKRFI341=OFF \
             -D ENABLE_DRIVER_SICKS3000=OFF \
             -D ENABLE_DRIVER_SIMPLESHAPE=OFF \
             -D ENABLE_DRIVER_SKYETEKM1=OFF \
             -D ENABLE_DRIVER_SONARTORANGER=OFF \
             -D ENABLE_DRIVER_SONYEVID30=OFF \
             -D ENABLE_DRIVER_SPHERE=OFF \
             -D ENABLE_DRIVER_SPHINX2=OFF \
             -D ENABLE_DRIVER_SR3000=OFF \
             -D ENABLE_DRIVER_STATGRAB=OFF \
             -D ENABLE_DRIVER_STOC=OFF \
             -D ENABLE_DRIVER_TCPSTREAM=OFF \
             -D ENABLE_DRIVER_UNICAPIMAGE=OFF \
             -D ENABLE_DRIVER_UPCBARCODE=OFF \
             -D ENABLE_DRIVER_URGLASER=OFF \
             -D ENABLE_DRIVER_VEC2MAP=OFF \
             -D ENABLE_DRIVER_VFH=OFF \
             -D ENABLE_DRIVER_VMAPFILE=OFF \
             -D ENABLE_DRIVER_WAVEFRONT=ON \
             -D ENABLE_DRIVER_WBR914=OFF \
             -D ENABLE_DRIVER_WRITELOG=OFF \
             -D ENABLE_DRIVER_XSENSMT=OFF \
             -D ENABLE_DRIVER_YARPIMAGE=OFF

installed: wiped $(SOURCE_DIR)/unpacked
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	rm -rf $(SOURCE_DIR)/build
	rm -rf player installed wiped

wipe: clean
	rm -rf build

