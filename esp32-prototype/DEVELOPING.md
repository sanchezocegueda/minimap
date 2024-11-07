# Setting up Espressif-IDF
Install [prerequisites](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-1-install-prerequisites)


Then run
```bash
cd esp-idf
./esp-idf/install.sh
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

