rem Builds TestVHACD out of tree, so CMake's project files do not land beside the sources.
cmake -S . -B ../build/app -DCMAKE_GENERATOR_PLATFORM=x64
cmake --build ../build/app --config Release
