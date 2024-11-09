# Setting up Espressif-IDF
Install [prerequisites](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-1-install-prerequisites)

Then run
```bash
cd esp-idf
./install.sh
. ./export.sh # run everytime we setup a shell
cd ..
```

### Activating Toolchain (Everytime you start your IDE, or new shell)
In a vscode terminal, you can run `. /path/to/export.sh` from `esp32-prototype` to setup your current shell environment for VSCode.
- Similar to how we needed to `nix develop` to use Lingua Franca

```bash
# Creating a new Project
idf.py create-project PROJECT_NAME
cd PROJECT_NAME

# Setup workflow
idf.py set-target esp32

# Let's you manually edit the IDF sdkconfig in a TUI.
idf.py menuconfig

# Building
idf.py all

# Flashing
idf.py -p DEVICE_PATH flash

# Monitoring
idf.py -p DEVICE_PATH monitor
```

# VSCode Setup
[Installing Extension](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md)
- Install the esp-idf vscode extension and choose custom idf installation, specify the esp-idf submodule in our repo.

Read through this thoroughly.
[Basic Use](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/basic_use.md)


## Setup udev rules on Linux for debugging (You guys shouldn't have to worry about this)
```bash
sudo cp -n "/home/$USER/.espressif/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32/share/openocd/contrib/60-openocd.rules" /etc/udev/rules.d
```

[ESP-IDF VSCode settings](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/SETTINGS.md)

---
# Understanding IDF
[Running the main task](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/startup.html#running-the-main-task)
- "Unlike normal FreeRTOS tasks (or embedded C main functions), the app_main task is allowed to return. If this happens, The task is cleaned up and the system will continue running with other RTOS tasks scheduled normally. Therefore, it is possible to implement app_main as either a function that creates other application tasks and then returns, or as a main application task itself."

[Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
- [Component Requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
[Tools](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html)

---
# IDF Components Available
[RadioLib](https://components.espressif.com/components/jgromes/radiolib/versions/7.1.0)
[IMU](https://components.espressif.com/components/truita/mpu9250/versions/1.0.1)
[Display](https://components.espressif.com/components/espressif/esp_lcd_gc9a01/versions/2.0.0)

---
# Security

