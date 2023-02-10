# ESP32forthStation
Stand alone Forth computer with LillyGo TTGO VGA32 single board computer and ESP32forth

---

Based on the work of several open source projects (see below) it was possible to make the _ESP32forthStation_. It is a single board Forth computer with keyboard (PS/2), video monitor (VGA) and networking capabilities (WiFi). It can be used as a complete development environment for your own experiments.

![ESP32fortStation setup on a table including keyboard and monitor](img/ESP32forthStation.jpg "")

## Hardware

The ESP32forthStation hardware uses the

- <a href="http://www.lilygo.cn/prod_view.aspx?TypeId=50063&Id=1083" target="_blank">TTGO VGA32 board</a> by <a href="http://www.lilygo.cn" target="_blank">LillyGo</a>, an ESP32 board with PS/2 Mouse, PS/2 keyboard, VGA video output, audio connectorm an micro SD card slot, some I/O pins as well as a 3.3V serial line. ESP32forthStation uses version V1.4 of that board.

- <a href="https://www.thingiverse.com/thing:4675382" target="_blank">TTGO VGA32 Case</a> from thingiverse by <a href="https://www.thingiverse.com/firepower9966/designs" target="_blank">Neil Bowden (firepower9966)</a>  .

![ESP32fortStation setup on a table including keyboard and monitor](img/LillyGo_TTGO_VGA32-board-and-case.jpg "")

## Software

ESP32forthStation is based on

- <a href="http://www.fabglib.org/" target="_blank">FABGL library</a> by 
   <a href="https://github.com/fdivitto" target="_blank">Fabrizio Di Vittorio</a>, an Arduino based library for the ESP32 that contributes keyboard/monitor access and ANSI terminal capablilities to ESP32forthStation. FABGL has a lot of additional features and uses the TTGO VGA32 board (among others) for many other applications worth investigating. See the <a href="http://www.fabglib.org/" target="_blank">FABGL home page</a> for details.

- <a href="https://esp32forth.appspot.com/ESP32forth.html">ESP32forth</a> by 
   <a href="https://github.com/flagxor" target="_blank">Brad Nelson</a>, a modern Arduino based inreractive Forth implementation with block and stream file support, networking with embedded Web-Server, access to hardware and ESP32 libraries/device drivers. See the <a href="https://esp32forth.appspot.com/ESP32forth.html">ESP32forth home page</a> for details.

The firmware of ESP32forthStation is a snapshot of ESP32forth augmented with appropriate calls to the FABGL library for I/O. This repostory provides released versions that are updated from time to time. Development takes place in the <a href="https://github.com/uho/ueforth/tree/VGA32">VGA32 branch of my ueforth fork</a> of <a href="https://github.com/flagxor/ueforth">Brad Nelson original ueforth repository</a>.

### Flashing the firmware

In order flash the ESP32forthStation firmware to the TTG32 Board use the Arduino IDE, select the board in the Arduino Board manager and flash the file [ESP32forth.ino](src/ESP32forth.ino). If in doubt please check the installation instruction on the <a href="https://esp32forth.appspot.com/ESP32forth.html">ESP32forth home page</a>.

## Licensing

The code in this repository is licensed under the terms of the GNU GENERAL PUBLIC LICENSE unless stated otherwise. The different contributing projects have different licensing condition. Please check with the appropriate repositories.

## _We're dwarfs standing on the shoulders of giants._

Contributions go to those that made this project possible (Brad Nelson, Fabrizio Di Vittorio, the engineers at LillyGo, Neil Bowden)

### Contact

If you have any questions and issues regarding this project please contact me.

Ulrich Hoffmann uho@xlerb.de

---

## Journal

- 2023-02-10 Publication of this repository

- 2022-12-10 <a href="doc/So_this _is_Christmas-ESP32forthStation.pdf">Presentation of ESP32forthStation</a> at the international Zoom meeting of the <a href="https://www.forth2020.org/">Forth2020 group</a>.


