#!/bin/sh -x

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

cd "/home/pioneer/group41/src/rosserial-hydro-devel/rosserial_xbee"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/pioneer/group41/install/lib/python2.7/dist-packages:/home/pioneer/group41/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pioneer/group41/build" \
    "/usr/bin/python" \
    "/home/pioneer/group41/src/rosserial-hydro-devel/rosserial_xbee/setup.py" \
    build --build-base "/home/pioneer/group41/build/rosserial-hydro-devel/rosserial_xbee" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/pioneer/group41/install" --install-scripts="/home/pioneer/group41/install/bin"
