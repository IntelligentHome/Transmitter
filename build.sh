#!/bin/bash

function build_Transmitter {
    mkdir Transmiter
    cd Transmiter
    cmake ../../src
    make
    cd ..
}


if [ "$1" = "clean" ]; then
    echo "Clean"
    rm -f -r bin
fi

if [ "$1" = "build" ]; then
    echo "Build"
    mkdir bin
    cd bin
    build_Transmitter
fi
