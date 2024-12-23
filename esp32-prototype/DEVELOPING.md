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
```
- **To put the esp into bootloader mode, ensure GPIO 0 is pulled low by pressing the 0 button when flashing, also GPIO 2 must be pulled low, floating, or disconnected**, see [Boot Mode Selection](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html#boot-mode-selection)

```bash
# Monitoring
idf.py -p DEVICE_PATH monitor
```

### Using a component/library from [Esspressif's Component Registry](https://components.espressif.com/) or repos that support ESP-IDF (look for `idf_component.yml`)
- **This is only a short description**, please skim [IDF Component Manager](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-component-manager.html#idf-component-manager) and [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) 

If the component has a significant amount of downloads, use the component registry to include it with
```bash
idf.py add-dependency COMPONENT_NAME
```
Otherwise, you should manually include it by cloning the repository into `components` folder. Take note of the `components` subdirectory name, this will be the same name referenced in `CMakeLists.txt`.

Once added, you must register the component within the `CMakeLists.txt` of `main`

```bash
idf_component_register(SRCS "foo.c" "bar.c"
                       REQUIRES name)
```
- **Make sure `name` matches what's in `managed_components` or the `components` directory**

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

### References
[Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
- [Project CMakeLists File](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#example-project)
- [Minimal Project CMakeLists.txt file](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#minimal-example-cmakelists)

May be useful later: [Tools](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html)