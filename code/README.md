# Code

## General information
Here there is the code for the [receiver](receiver/) and the [transmitter](transmitter/) modules that uses [DWM1001C](https://www.qorvo.com/products/p/DWM1001C) module which is a
Ultra Wideband (UWB) and Bluetooth hardware based on DecaWave's DW1000 IC and Nordic Semiconductor [nrF52832](https://www.nordicsemi.com/Products/nRF52832) SoC.

The project was created with the IDE: [Segger Embedded Studio (SES)](https://www.segger.com/products/development-tools/embedded-studio/).\
To install it:
- Go to the download page [Embedded Studio for RISC-V](https://www.segger.com/downloads/embedded-studio/#ESforRISCV) and download the version V4.12
- When using SES IDE, you will need to install the following package that can be installed from SES itself, through the package manager in the tools menu:
  -  CMSIS 5 CMSIS-CORE Support Package (version 5.02)
  -  CMSIS-CORE Support Package (version 4.05)
  -  Nordic Semiconductor nRF CPU Support Package (version 1.06)

More information on getting started with DWM1001-dev can be found in the [Firmware User Guide](../doc/DWM1001FirmwareUserGuide.pdf)

The project is built as follow : 
```
code/
├── deca_driver                   // DW1000 API software package 2.04
├── nRF5_SDK_14.2.0               // Nordic Semiconductor SDK 14.2 for nrF52832
├── receiver                      // SES project for the receiver modulus
│   ├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
│   ├── ses                       
|   │   ├── receiver.emProject    // Segger Embedded Studio project
|   │   ├── main.c                // Initialization and main program
│   ├── UART                      // Uart 
├── transmitter                   // SES project for the trasmitter modulus
│   ├── config                    // Contains sdk_config.h file for nrF SDK 14.2 customization
│   ├── ses                       
|   │   ├── transmitter.emProject // Segger Embedded Studio project
|   │   ├── main.c                // Initialization and main program
│   ├── UART                      // Uart 
└── README.md
```

