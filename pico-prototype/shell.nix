{ pkgs ? import <nixpkgs> {} } :
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
  buildInputs = [
  ];

  shellHook = ''
    export PICO_SDK_PATH="$PWD/pico-sdk"
    export PICO_BOARD=pico_w
    mkdir -p bin
    export PATH="$PATH:$PWD/bin"
    echo "[shell] building picotool"
    git submodule update --init
    pushd picotool > /dev/null 2>&1
    mkdir -p build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=. -DPICOTOOL_FLAT_INSTALL=1 ..
    make
    ln -sf "$PWD/picotool" ../../bin
    popd > /dev/null 2>&1
    echo "[shell] setting up pico-sdk"
    cd pico-sdk
    git submodule update --init
    cd ..
    echo "[shell] pico-prototype shell ready"
    '';
}
