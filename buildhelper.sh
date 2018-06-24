#!/bin/bash

if [ -z ${HOU_VER+x} ]; then
	HOU_VER=hfs16.5
fi

tmppwd=$PWD

if [ -z ${HOU_FOLDER+x} ]; then
	cd /opt/$HOU_VER/
else
	cd ${HOU_FOLDER}
fi
echo $PWD
source houdini_setup
cd $tmppwd

if [ -z ${NVFLEX_DIR+x} ]; then
	export NVFLEX_DIR=$HOME/flex-110/flex
fi

export PATH=${HB}:${PATH}

make install
rm nvFlexDop/*.o
rm *.so
