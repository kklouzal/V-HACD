cmake_minimum_required(VERSION 3.16)
project(VHACDContractTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()
add_executable(VHACDContractTests ContractTests.cpp)
target_include_directories(VHACDContractTests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
add_test(NAME VHACDContractTests COMMAND VHACDContractTests)
