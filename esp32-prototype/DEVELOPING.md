# Setting up Espressif-IDF
Install [prerequisites](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-1-install-prerequisites)

Assuming you are in `minimap/esp32-prototype`
```bash
# ensure esp-idf is downloaded
git submodule update --init

cd esp-idf
./install.sh
. ./export.sh # run everytime we setup a shell, remember to include the absolute path to export.sh!!!
cd ..
```

## Activating Toolchain (Everytime you start your IDE, or new shell)
In a vscode terminal, you can run `. /path/to/export.sh` to setup your current shell environment for VSCode.
- Similar to how we needed to `nix develop` to use Lingua Franca

## Using the toolchain from CLI
### Creating a new project
```bash
idf.py create-project PROJECT_NAME
cd PROJECT_NAME
```
- **You should always run this in `esp32-prototype` to create new projects.**
- **If you want to test something, create a new project for it.**

### Initial project setup
```bash
# Specify the board
idf.py set-target esp32

# Edit any specific configuration of IDF you may want
idf.py menuconfig

# Building the esp components
idf.py all
```
### Development
```bash
# Build project
idf.py build

# Flashing
idf.py -p DEVICE_PATH flash

# Monitoring
idf.py -p DEVICE_PATH monitor
```

## VSCode Extension Setup
**Install the esp-idf vscode extension and choose custom idf installation, specify the esp-idf submodule in our repo.**

Do not blindly follow this, but the full instructions are here if wanted.
- [Installing Extension](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md)

Read through the [basic use](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/basic_use.md) to understand how the extension is used/configured.

### References
[ESP-IDF VSCode settings](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/SETTINGS.md)

# Understanding IDF
[Running the main task](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/startup.html#running-the-main-task)
- "Unlike normal FreeRTOS tasks (or embedded C main functions), the app_main task is allowed to return. If this happens, The task is cleaned up and the system will continue running with other RTOS tasks scheduled normally. Therefore, it is possible to implement app_main as either a function that creates other application tasks and then returns, or as a main application task itself."

[Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
- [Component Requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
[Tools](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html)

# IDF Components Available
- [RadioLib](https://components.espressif.com/components/jgromes/radiolib/versions/7.1.0)
- [IMU](https://components.espressif.com/components/truita/mpu9250/versions/1.0.1)
- [Display](https://components.espressif.com/components/espressif/esp_lcd_gc9a01/versions/2.0.0)

# Security
