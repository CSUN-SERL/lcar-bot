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

cd "/home/thomas/lcar-bot/src/vision_opencv/src"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/thomas/lcar-bot/install/lib/python2.7/dist-packages:/home/thomas/lcar-bot/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/thomas/lcar-bot/build" \
    "/usr/bin/python" \
    "/home/thomas/lcar-bot/src/vision_opencv/src/setup.py" \
    build --build-base "/home/thomas/lcar-bot/build/vision_opencv/src" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/thomas/lcar-bot/install" --install-scripts="/home/thomas/lcar-bot/install/bin"
