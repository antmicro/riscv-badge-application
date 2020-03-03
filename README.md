# RISC-V Electronic Badge application

## Cloning the repository

The application is based on the Zephyr RTOS. The code of the Zephyr RTOS is referenced as a submodule.
In order to clone the repository and its sudmodules run the following command:

```sh
$ git clone --recursive https://github.com/antmicro/riscv-badge-application.git
```

## Building the application

### Prerequisites

In order to build the application and link it against the Zephyr RTOS, the Zephyr SDK has to be installed in the system.
To install the Zephyr SDK, do the following:

```sh
$ wget https://github.com/zephyrproject-rtos/meta-zephyr-sdk/releases/download/0.9.2/zephyr-sdk-0.9.2-setup.run
$ chmod +x zephyr-sdk-0.9.2-setup.run
$ ./zephyr-sdk-0.9.2-setup.run
```

Zephyr itself requires some additional tools to be installed in the system. Follow the [Zephyr documentation](http://docs.zephyrproject.org/getting_started/installation_linux.html) for requirements.

### Building

To build the application you need to setup a build environment.
To do this, source the `zephyr-env.sh` file from the `zephyr` directory.
```sh
$ source riscv-badge-application/zephyr/zephyr-env.sh
```
Additionally, three environment variables have to be set before building the app:

```sh
$ export ZEPHYR_GCC_VARIANT="zephyr"
$ export ZEPHYR_SDK_INSTALL_DIR="/opt/zephyr-sdk" #assuming default sdk install directory
$ export BOARD="hifive1"
```

The next step is to create a make environment:
```sh
$ cd riscv-badge-application/badge_fe310/
$ mkdir -p build && cd build
$ cmake ../

```

To build the application, simply run `make`:

```sh
$ make -j`nproc`
```

The resulting binary is located in the `build/zephyr` directory under `zephyr.elf`.

## Binary deployment

The device can be programmed using the JTAG interface via an FTDI based adapter. The FTDI adapter is a part of the module, so no additional hardware is required.

### Installing the freedom-e-sdk

In order to upload the application to the Badge device, SiFive's Freedom-E-SDK is required.
Download and installation instructions can be found in the [SDK's GitHub repository](https://github.com/sifive/freedom-e-sdk).

### Uploading

With the SDK installed, you can connect to the board using OpenOCD with the configuration file from the `utils` directory.
To establish an OpenOCD connection, switch to the `utils` directory and run:

```sh
# assuming that the openocd executable's location is in PATH
$ sudo openocd -f openocd_quad.cfg
```

Leave it running, and in a different terminal, use GDB to upload the binary to the board.
Use the RISC-V GDB from SiFive's Freedom-E-SDK toolchain.

Before loading, the device's flash protection has to be disabled.
In order to load the binary to the device, run the following commands in the GDB terminal:
```sh
$ gdb
(gdb) set remotetimeout 240
(gdb) target extended-remote localhost:3333
(gdb) monitor reset halt
(gdb) monitor flash protect 0 64 last off
(gdb) load {path to repository}/build/zephyr/zephyr.elf
(gdb) monitor resume
```

## Connecting to the board

The board uses a quad USB chip, so it will show up in the system as four tty devices.
Zephyr standard output will launch on the third interface.
Use your favorite com terminal (e.g. `picocom`) to connect with the board.
The default baud rate is 115200.
