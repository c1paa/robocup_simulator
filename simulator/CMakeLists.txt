cmake_minimum_required(VERSION 3.20)
project(robocup_simulator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# GLM (OpenGL Mathematics) — header-only
# ============================================================
include(FetchContent)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

# ============================================================
# nlohmann/json — header-only
# ============================================================
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# ============================================================
# SDL2
# ============================================================
find_package(SDL2 REQUIRED)

# ============================================================
# OpenGL
# ============================================================
find_package(OpenGL REQUIRED)

# ============================================================
# Bullet Physics
# ============================================================
find_package(Bullet REQUIRED)

# ============================================================
# gRPC + Protobuf
# ============================================================
find_package(gRPC CONFIG REQUIRED)
find_package(Protobuf REQUIRED)

# gRPC code generation
set(PROTO_DIR "${CMAKE_SOURCE_DIR}/proto")
set(PROTO_FILE "${PROTO_DIR}/simulator.proto")

get_target_property(GRPC_CPP_PLUGIN gRPC::grpc_cpp_plugin LOCATION)
get_target_property(PROTOBUF_PROTOC_EXEC protobuf::protoc LOCATION)

set(GENERATED_PROTO_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_PROTO_DIR})

# Generate protobuf C++ sources
add_custom_command(
    OUTPUT
        ${GENERATED_PROTO_DIR}/simulator.pb.cc
        ${GENERATED_PROTO_DIR}/simulator.pb.h
        ${GENERATED_PROTO_DIR}/simulator.grpc.pb.cc
        ${GENERATED_PROTO_DIR}/simulator.grpc.pb.h
    COMMAND ${PROTOBUF_PROTOC_EXEC}
        --proto_path=${PROTO_DIR}
        --cpp_out=${GENERATED_PROTO_DIR}
        --grpc_out=${GENERATED_PROTO_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
)

# ============================================================
# Sources
# ============================================================
set(SOURCES
    src/main.cpp
    src/app.cpp
    src/renderer.cpp
    src/physics.cpp
    src/field.cpp
    src/robot.cpp
    src/camera.cpp
    src/config.cpp
    src/grpc_server.cpp
)

set(GENERATED_SOURCES
    ${GENERATED_PROTO_DIR}/simulator.pb.cc
    ${GENERATED_PROTO_DIR}/simulator.grpc.pb.cc
)

# ============================================================
# Executable
# ============================================================
add_executable(simulator ${SOURCES} ${GENERATED_SOURCES})

target_include_directories(simulator PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${GENERATED_PROTO_DIR}
)

target_link_libraries(simulator PRIVATE
    ${SDL2_LIBRARIES}
    OpenGL::GL
    ${BULLET_LIBRARIES}
    glm::glm
    nlohmann_json::nlohmann_json
    gRPC::grpc++
    protobuf::libprotobuf
)

# macOS needs frameworks
if(APPLE)
    target_link_libraries(simulator PRIVATE
        "-framework OpenGL"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
    )
endif()
