# KISS-ICP C++ Library

## How to build

```bash
cmake -Bbuild
cmake --build build -j$(nproc --all)
```

## How to install:

```bash
# install system-wide
cmake --install build
# or install to a custom location (e.g. "/usr/local")
cmake --install build --prefix "<your_custom_install_prefix>"
```

## Dependencies

The cmake build system should handle all dependencies for you. In case you have some particular
requirements for the library dependencies, by default the build system will attempt to use the
ones you have installed on your local system.
