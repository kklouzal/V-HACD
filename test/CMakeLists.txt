cmake_minimum_required(VERSION 3.16)
project(VHACDContractTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()
add_executable(VHACDContractTests ContractTests.cpp)
target_include_directories(VHACDContractTests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")

# The library is a header, so the tests are where its warnings surface. Conformance mode matters as much
# as the warnings: a name used above its definition compiles only under MSVC's permissive default.
if(MSVC)
    target_compile_options(VHACDContractTests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(VHACDContractTests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_test(NAME VHACDContractTests COMMAND VHACDContractTests)
