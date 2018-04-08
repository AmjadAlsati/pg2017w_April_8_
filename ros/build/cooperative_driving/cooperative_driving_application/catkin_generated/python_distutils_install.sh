#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_application"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/amjad/Desktop/pg2017w/ros/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/amjad/Desktop/pg2017w/ros/install/lib/python2.7/dist-packages:/home/amjad/Desktop/pg2017w/ros/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/amjad/Desktop/pg2017w/ros/build" \
    "/usr/bin/python" \
    "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_application/setup.py" \
    build --build-base "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/amjad/Desktop/pg2017w/ros/install" --install-scripts="/home/amjad/Desktop/pg2017w/ros/install/bin"
