cmake_minimum_required(VERSION 3.13...3.27)

# Let pico-sdk know where our picotool binary is
set(picotool_DIR "$ENV{PICOTOOL_DIR}")

# initialize pico-sdk from submodule
# note: this must happen before project()
include("$ENV{PICO_SDK_PATH}/pico_sdk_init.cmake")

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(hello_usb
    hello_usb.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(blink pico_stdlib)


set(EXECUTABLE_OUTPUT_PATH, ../bin)

# NOTE FOR DEVELOPING --------------------------
# create map/bin/hex/uf2 file in addition to ELF.
# pico_add_extra_outputs(hello_world)
# uf2 files can be easily dragged into folders into the pico on windows.
# uncomment pico_add_extra_outputs to create uf2 files
