.PHONY: debug all doc install relase clean

ifeq ($(shell uname -m), armv7l)
#ifndef ROS_PARALLEL_JOBS
#	$(error ROS_PARALLEL_JOBS is not set, set it to '-j2 -l2' first)
#	exit
PARALLEL_FLAG=ROS_PARALLEL_JOBS='-j2 -l2'
#endif # end jobs
endif # end arm

debug:
ifndef ROS_DISTRO
	$(error ROS_DISTRO is not set, source ROS workspace first)
else
	MAKEFLAGS= MAKELEVEL= $(PARALLEL_FLAG) catkin_make -DCMAKE_BUILD_TYPE=Debug
endif # end distro

all: debug doc

devel/share/cooperative_driving/doc:
	mkdir -p $@

doc: devel/share/cooperative_driving/doc
	doxygen src/cooperative_driving/Doxyfile

install:
ifndef ROS_DISTRO
	$(error ROS_DISTRO is not set, source ROS workspace first)
else
	MAKEFLAGS= MAKELEVEL= $(PARALLEL_FLAG) catkin_make install -DCMAKE_BUILD_TYPE=Release
endif # end distro

release:
ifndef ROS_DISTRO
	$(error ROS_DISTRO is not set, source ROS workspace first)
else
	MAKEFLAGS= MAKELEVEL= $(PARALLEL_FLAG) catkin_make -DCMAKE_BUILD_TYPE=Release
endif # end distro

clean:
	$(RM) -r build
	$(RM) -r devel
	$(RM) -r install
