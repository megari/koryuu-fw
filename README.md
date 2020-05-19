Koryuu video transcoder firmware
================================

This is the firmware for the Koryuu video transcoder.

It is licensed under whatever license the underlying AVR-libc, yaal and yaamake require. TODO

The firmware is written in C++14 with using code and/or tools from the following projects:
 * [yaal](https://github.com/raphendyr/yaal), an efficient C++ framework for programming on AVR microcontrollers, written by Jaakko Kantojärvi, with contributions from myself, with permission,
 * [yaamake](https://github.com/raphendyr/yaamake), a custom build system for AVR projects, written by Jaakko Kantojärvi, with minor modifications with permission,
 * avr-gcc and avr-binutils toolchains
 * [avr-libc](http://savannah.nongnu.org/projects/avr-libc), a FOSS libc for AVR programming.
 * [avrdude](http://savannah.nongnu.org/projects/avrdude), a program for using AVR programmers.

The firmware targets the atmega328p microcontroller, which the Koryuu uses.

Requirements
------------

In addition to the software components listed above, an AVR programmer with a 6-pin ICSP programming cable is required for flashing a new firmware on the Koryuu.

I use the [Olimex AVR-ISP-MK2](https://www.olimex.com/Products/AVR/Programmers/AVR-ISP-MK2/open-source-hardware) myself, but there are less expensive options available, such as Sparkfun's [Pocket AVR Programmer](https://www.sparkfun.com/products/9825) or [STK500 Compatible USB Programmer](https://www.sparkfun.com/products/8702), or various programmers that can be found by Googling or searching eBay or AliExpress for `usbisp` or `usbasp`.

However, please be mindful that the cheaper programmers, especially those directly from China, should always be carefully checked for obvious electronic defects (such as short circuits) before connecting to expensive equipment, such as your computer or the Koryuu.

Building
--------

Building the firmware is fairly simple in a POSIX-compatible environment (eg. some Linux or BSD variant), assuming sufficiently recent versions of avr-libc, avr-gcc and avr-binutils are already installed.

First, we need to clone the firmware Git repository and check out the required submodules (`vendor/yaal` and `vendor/yaamake`):
```sh
git clone https://github.com/megari/koryuu-fw.git
cd koryuu-fw
git submodule update --init --recursive --checkout
```

The next step is to build `yaamake` and check that it works:
```sh
cd vendor/yaamake
make NO_TEENSY=1
./yaamake --version
```
(For more advanced configuration of `yaamake`, please see `vendor/yaamake/README.md`.)

As the last preparatory step, we need configure the `Makefile` of the firmware:
```sh
cd ../.. # Should end up in the base directory of the koryuu-fw repository
editor Makefile # You may substitute editor with one of your choice, such as nano, vi, vim, ...
```

The most relevant lines to check and modify are the programmer settings:
```makefile
PROGRAMMER_PORT = usb
#PROGRAMMER_BAUD = 115200
PROGRAMMER = avrispmkII
```
Please consult the vendor of your AVR programmer for the correct settings, or try to find the matching programmer in the list of programmers supported by `avrdude`:
```sh
avrdude -p atmega328p -c foobar # "foobar" can be substituted with any string that does not match a known programmer.
```

Now we should be all set up for building the firmware:
```sh
make build_hex
```

Flashing the built firmware can be accomplished by the command
```sh
make program
```

Debug builds
------------

There are also debug build targets:
```sh
make build_debug
make build_debug2
make build_no_autoreset
make build_no_panic
```

All versions of the firmware can be built like so:
```sh
./generate_fw_imgs.sh
```
