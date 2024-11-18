# Example on how to include the KissICP library in other C++ projects with CMake

See `CMakeLists.txt` for details on how to find the library and link the targets, and `src/example.cpp` on how to include the corresponding files.

## Building your project
```bash
cmake -Bbuild
cmake --build build -j$(nproc --all)
```

