#!/bin/bash

#change to specific build if needed
export HOU_VER=hfs16.5

#change to your custom location of flex library of version 1.1.0
export NVFLEX_DIR=$HOME/flex-110/flex

#uncomment and set for custom houdini location (HOU_VER will be ignored)
#export HOU_FOLDER=/my/custom/folder/hfs16.5

#uncomment and set for custom compiler
#export CXX=/use/bin/g++

./buildhelper.sh
