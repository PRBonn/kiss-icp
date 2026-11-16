# KISS-ICP C++ Library

## How to build

```sh
cmake -Bbuild
cmake --build build -j$(nproc --all)
```

To run the C++ regression tests from the repository root:

```sh
cmake -S cpp/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```

## Dependencies

The cmake build system should handle all dependencies for you. In case you have some particular
requirements for the library dependencies, by default the build system will attempt to use the
ones you have installed on your local system.
