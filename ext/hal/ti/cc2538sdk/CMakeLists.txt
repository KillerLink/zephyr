if(CONFIG_TI_CC2538_USE_SDK_DRIVERLIB)
	zephyr_include_directories(
		.
		source
		source/inc
		source/driverlib
	)
	zephyr_library()
	zephyr_library_compile_definitions(${COMPILER})
#	zephyr_library_compile_definitions(USE_CC2538_ROM_DRV_API)
	zephyr_library_sources(
#		source/driverlib/adc.c
		source/driverlib/ioc.c
		source/driverlib/sys_ctrl.c
#		source/driverlib/flash.c
#		source/driverlib/aes.c
		source/driverlib/gpio.c
#		source/driverlib/pka.c
		source/driverlib/systick.c
		source/driverlib/uart.c
#		source/driverlib/udma.c
#		source/driverlib/sha256.c
		source/driverlib/sleepmode.c
		source/driverlib/interrupt.c
#		source/driverlib/ssi.c
		source/driverlib/watchdog.c
		source/driverlib/debug.c
		source/driverlib/cpu.c
#		source/driverlib/ccm.c
#		source/driverlib/i2c.c
	)

#    set_source_files_properties(source/ti/drivers/net/wifi/source/driver.c
#      PROPERTIES COMPILE_DEFINITIONS "__LINUX_ERRNO_EXTENSIONS__;${COMPILER}" )
    set_source_files_properties(source/driverlib/uart.c
      PROPERTIES COMPILE_FLAGS -O0)

endif()
