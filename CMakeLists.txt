#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.12)

# project settings
project(GRIPPER C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(USE_ROM_SIZE STM32F40_41xxx)

# define macro
add_definitions(
-DSTM32F40_41xxx
-DUSE_STDPERIPH_DRIVER
-DARM_MATH_CM4
)

set_property(SOURCE STMLIB/startup_stm32f407xx.s PROPERTY LANGUAGE C)

# 设置编译选项
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Og -g" CACHE INTERNAL "c compiler flags")
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard" CACHE STRING "asmebler flags")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-xassembler-with-cpp>)

# 添加头文件路径
include_directories(
    hardware
    hardware/mpu6050
    system/delay
    system/sys
    system/usart
    fwlib/inc
    src/
    core
    cmsis/
    rtos/include
    rtos/portable/Keil
    rtos/portable/MemMang
)

# 添加 C 源文件
file(GLOB_RECURSE SOURCES
    hardware/*.c
    hardware/mpu6050/*.c
    system/delay/*.c
    system/sys/*.c
    system/usart/*.c
    fwlib/src/*.c
    src/*.c
    main.c
    rtos/*.c
    rtos/portable/Keil/*.s
    rtos/portable/MemMang/*.c
)

# 创建库目标
add_library(GRIPPER_LIB STATIC ${SOURCES})

if(USE_ROM_SIZE STREQUAL STM32F10X_HD)
    list(APPEND SOURCES "cmsis/startup_stm32f10x_hd.s")
elseif(USE_ROM_SIZE STREQUAL STM32F10X_MD)
    list(APPEND SOURCES "cmsis/startup_stm32f10x_md.s")
elseif(USE_ROM_SIZE STREQUAL STM32F40_41xxx)
    list(APPEND SOURCES "cmsis/startup_stm32f407xx.s")
endif()
message(STATUS >>> ${USE_ROM_SIZE})

list(APPEND SOURCES "")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/cmsis/STM32F407VETx_FLASH.ld)

add_link_options(-specs=nano.specs -specs=nosys.specs)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

# 链接所需库
target_link_libraries(GRIPPER_LIB
    m
    c
    nosys
)

# 设置启动文件和链接脚本
set_target_properties(GRIPPER_LIB PROPERTIES
    LINK_FLAGS "-Tstm32_flash.ld -Wl,-Map=GRIPPER.map"
)

# 设置输出文件名
set_target_properties(GRIPPER_LIB PROPERTIES OUTPUT_NAME "GRIPPER")

# 添加 hex 和 bin 目标
add_custom_target(GRIPPER.hex ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> GRIPPER.hex
)

add_custom_target(GRIPPER.bin ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> GRIPPER.bin
)
