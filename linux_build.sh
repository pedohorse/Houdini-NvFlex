#!/bin/bash

#change to specific build if needed
HOU_VER=hfs16.5

tmppwd=$PWD

#change path to your custom location
cd /opt/$HOU_VER/
echo $PWD
source houdini_setup
cd $tmppwd

export NVFLEX_DIR=$HOME/flex-110/flex
echo $NVFLEX_DIR

export PATH=${HB}:${PATH}
#export CXX=/path/to/custom/g++
make install
rm nvFlexDop/*.o
