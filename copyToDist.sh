#!/bin/sh
TOP=`dirname $0`
DEST="$TOP/dist"

mkdir -p dist
cp -v $TOP/install/lib/*.so.* $DEST
cp -v $TOP/bindings/*.so $DEST
cp -v $TOP/bindings/*.st $DEST

