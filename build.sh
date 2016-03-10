#!/bin/sh

BUILD_PTH='./TestPipeline/build/'

git reset -- hard && git pull;

cd $BUILD_PTH && cmake .. && make
