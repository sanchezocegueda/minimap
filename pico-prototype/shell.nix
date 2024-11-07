{ pkgs ? import <nixpkgs> {} } :
let
  lib = pkgs.lib;
  picoSetup = ''
    export PICO_SDK_PATH="$PWD/pico-sdk"
    export PICOTOOL_DIR="$PWD/picotool/build/picotool"
    export PICO_BOARD="pico_w"
    export PATH="$PATH:$PWD/bin"

    git submodule update --init --recursive
    mkdir -p bin

    if [ ! -f bin/picotool ]; then
      echo "[shell] building picotool"
      mkdir -p picotool/build
      pushd picotool/build > /dev/null
      cmake -DCMAKE_INSTALL_PREFIX=. -DPICOTOOL_FLAT_INSTALL=1 ..
      make
      ln -sf "$PWD/picotool" ../../bin
      popd > /dev/null
    else
      echo "[shell] picotool already built, skipping build"
    fi

      echo "[shell] setting up pico-sdk"
      cd pico-sdk
      git submodule update --init
      cd ..
    '';
in
pkgs.mkShell {

  packages = with pkgs; [
    cmake
    gcc-arm-embedded
    git
    gdb
    libusb1
    openocd
    screen
  ];

  shellHook = picoSetup +''
    echo "[shell] pico-prototype shell ready"
    '';
}
