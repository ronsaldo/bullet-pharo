#!/bin/sh

sh ./buildBullet.sh
sh ./bindings/make-bindings.sh
sh ./copyToDist.sh

