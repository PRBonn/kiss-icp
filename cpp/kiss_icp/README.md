# KISS-ICP C++ Library

## How to build

```sh
cmake -Bbuild
cmake --build build -j$(nproc --all)
```

## Dependencies

The cmake build system should handle all dependencies for you. In case you have some particular
requirements for the library dependencies, by default the build system will attempt to use the
ones you have installed on your local system.
