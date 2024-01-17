#Lessons Learned

## USB Cable Debugging

Connectivity > USB_OTG_FS
	Mode: Device_only
Middleware > USB_DEVICE
	Class for FS IP: Communication Device Class (Virtual Port Com)

## SD card
To use the on-board SD slot, make sure you do the following:
Connectivity > SDIO : 4 bits wide bus
	DMA Settings: Add a DMA stream
	NVIC Settings: Enable interrupts for DMA stream and SDIO global interrupt
System Core > GPIO > SDIO : make sure all pins are pull-up
Middleware > FATFS
	Check SD Card
	Under Advanced Settings,
		Use dma template: enabled




