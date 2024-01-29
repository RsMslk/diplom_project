cmake_minimum_required(VERSION 3.27)

project(main)

set(CMAKE_CXX_STANDARD 17)

# Поиск пакета fmt
find_package(fmt CONFIG REQUIRED)

# Поиск пакета Open3D
find_package(Open3D REQUIRED)

# Добавление исходных файлов
add_executable(main
        main.cpp
        physics.h
        physics.cpp
        Visualizer.h
        Visualizer.cpp
)

# Линковка с библиотеками
target_link_libraries(main PRIVATE fmt::fmt Open3D::Open3D)
