cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32G474xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../USB_Device/App
    ../../USB_Device/Target
    ../../Drivers/STM32G4xx_HAL_Driver/Inc
    ../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/adc.c
    ../../Core/Src/dma.c
    ../../Core/Src/fdcan.c
    ../../Core/Src/spi.c
    ../../Core/Src/tim.c
    ../../Core/Src/usart.c
    ../../Core/Src/stm32g4xx_it.c
    ../../Core/Src/stm32g4xx_hal_msp.c
    ../../USB_Device/Target/usbd_conf.c
    ../../USB_Device/App/usb_device.c
    ../../USB_Device/App/usbd_desc.c
    ../../USB_Device/App/usbd_cdc_if.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    ../../Core/Src/system_stm32g4xx.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32g474xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


