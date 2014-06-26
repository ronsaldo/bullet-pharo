#!/bin/sh

TOP=`pwd`

# Build Bullet.
cd $TOP
sh ./buildBullet.sh

# Make the bindings
cd $TOP/bindings
sh ./make-bindings.sh

# Copy to the distribution folder.
cd $TOP
sh ./copyToDist.sh

