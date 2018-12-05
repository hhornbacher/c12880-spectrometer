add_executable(firmware 
    ../startup_stm32l432xx.s
    main.c 
    stm32l4xx_hal_msp.c 
    system_stm32l4xx.c 
    stm32l4xx_it.c 
    usbd_desc.c 
    usb_device.c 
    usbd_cdc_if.c 
    usbd_conf.c
)
target_link_libraries(firmware c m nosys App HAL UsbCore UsbClass)
# set_target_properties(firmware PROPERTIES LINKER_LANGUAGE C)

firmware_size(firmware)
generate_object(firmware .bin binary)