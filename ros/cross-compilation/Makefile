TOOLCHAIN=gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf
DEPENDING_PACKAGES=cooperative_driving
ADDITIONAL_ROS_PACKAGES=rostest camera_calibration compressed_image_transport

ifneq ($(wildcard $(shell which rospack)),)
  DEPENDING_PACKAGEXMLS=$(foreach dep,$(DEPENDING_PACKAGES),$(shell rospack find $(dep))/package.xml)
endif

all: ros-ccs-labs.deb

$(TOOLCHAIN).tar.xz:
ifeq ($(wildcard $(TOOLCHAIN).tar.xz),)
	wget https://releases.linaro.org/components/toolchain/binaries/4.9-2017.01/arm-linux-gnueabihf/$(TOOLCHAIN).tar.xz --quiet
endif

$(TOOLCHAIN): $(TOOLCHAIN).tar.xz
	tar -xJmf $<

ifdef DEPENDING_PACKAGEXMLS
ros-dependencies: $(DEPENDING_PACKAGEXMLS)
	( rospack depends1 $(DEPENDING_PACKAGES) && echo $(ADDITIONAL_ROS_PACKAGES) ) > $@
else ifeq ($(wildcard ros-dependencies),)
ros-dependencies:
	$(error ros-dependencies does not exist. Please provide this file or install rospack)
endif

ros-ccs-labs.rosinstall: ros-dependencies
	xargs -a $< rosinstall_generator --rosdistro kinetic --deps --wet-only --tar > $@

src: ros-ccs-labs.rosinstall src.patch
	wstool init -j8 $@ $< > /dev/null
	patch -p1 < $@.patch

multistrap.conf: multistrap.conf.in apt-dependencies
	sed 's/@APT_DEPENDENCIES@/$(shell cat apt-dependencies | tr '\n' ' ')/g' $< > $@

rootfs: multistrap.conf rootfs.patch pip-dependencies
	@echo "Building rootfs"
	$(RM) -r $@
	multistrap -a armhf -d $@ -f multistrap.conf > /dev/null
	pip install --system --target=rootfs/usr/lib/python2.7/dist-packages $(shell cat pip-dependencies | tr '\n' ' ') > /dev/null
	patch -p1 < $@.patch > /dev/null
	cd $@ && find . -lname '/*' -exec sh -c 'ln -sf `dirname {} | sed "s!/[^/]*!/..!g"``readlink {}` {}' \; # make links relative > /dev/null
	touch $@

install_isolated: $(TOOLCHAIN) src rootfs raspberry-pi-toolchain.cmake
	MAKELEVEL= MAKEFLAGS= RASPI_ROOTFS=$(shell pwd)/rootfs RASPI_TOOLCHAIN=$(shell pwd)/$(TOOLCHAIN) DESTDIR=$(shell pwd)/$@ ./src/catkin/bin/catkin_make_isolated \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(shell pwd)/raspberry-pi-toolchain.cmake \
		-DCATKIN_SKIP_TESTING=On \
		--install-space=/opt/ros/kinetic \
		--install > /dev/null
	find install_isolated -name '*.cmake' -exec sed -i 's!$(shell pwd)/rootfs!!g' {} \;
	find install_isolated -name '*.cmake' -exec sed -i 's!$(shell pwd)/install_isolated!!g' {} \;

install_isolated/DEBIAN/control: deb-control.in | install_isolated
	[ -d $(shell dirname $@) ] || mkdir $(shell dirname $@)
	sed 's/@APT_DEPENDENCIES@/$(shell cat apt-dependencies | perl -pe "chomp if eof" | tr '\n' ',')/g' $< > $@

install_isolated/DEBIAN/prerm: install_isolated deb-prerm
	[ -d $(shell dirname $@) ] || mkdir $(shell dirname $@)
	cp deb-prerm $(shell dirname $@)

ros-ccs-labs.deb: install_isolated install_isolated/DEBIAN/control install_isolated/DEBIAN/prerm
	fakeroot sh -c "chown -R root: install_isolated/opt && dpkg -b install_isolated $@"

complete-rootfs: rootfs ros-ccs-labs.deb
	dpkg -x ros-ccs-labs.deb rootfs

clean: clean-toolchain clean-rootfs clean-src clean-compiled

clean-toolchain:
	$(RM) -r $(TOOLCHAIN) $(TOOLCHAIN).tar.xz

clean-rootfs:
	$(RM) -r multistrap.conf rootfs

clean-src:
	$(RM) -r ros-ccs-labs.rosinstall src

clean-compiled:
	$(RM) -r build_isolated devel_isolated install_isolated

clean-package:
	$(RM) ros-ccs-labs.deb

dist-clean: clean clean-package

.PHONY:	all complete-rootfs clean dist-clean
