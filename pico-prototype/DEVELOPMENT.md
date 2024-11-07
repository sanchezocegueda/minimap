# Getting Started
Ensure you have `nix` on your system and your user is configured to access the nix repositories.

Run `nix develop` in `pico-prototype` to build picotool and setup the pico-sdk for your machine to get started with a pico-w.

# Building binaries and loading elfs

```bash
cmake -S . -B build
make -C build
```

To load elfs, hold BOOTSEL and plug in the pico to your pc.
`sudo picotool load -fx build/bin.elf `

If you're on Windows, you can optionally build uf2 files to drag and drop into the pico-w by adding
`pico_add_extra_outputs(hello_world)` to `CMakeLists.txt`.
