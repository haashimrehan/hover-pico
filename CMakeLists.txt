cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(hover-pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)

add_executable(hover-pico
  hover-pico.c
  pico_uart_transport.c
)

pico_generate_pio_header(hover-pico ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
pico_generate_pio_header(hover-pico ${CMAKE_CURRENT_LIST_DIR}/uart_tx.pio)
pico_generate_pio_header(hover-pico ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)

target_link_libraries(hover-pico
    pico_stdlib
    microros
    pico_multicore
    hardware_pio
)

target_include_directories(hover-pico PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Enable printf output over USB and disable for UART
pico_enable_stdio_usb(hover-pico 1)
pico_enable_stdio_uart(hover-pico 0)

# enable line-ending settings for microros
#add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(hover-pico)
