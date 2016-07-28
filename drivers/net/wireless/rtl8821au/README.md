rtl8821au linux (or rtl8812au)
==============================

rtl8821/rtl8812 linux kernel driver Wireless Dual-Band USB Adapter

For the USB3 issue read at the end  
and for RTL8813AU/RTL8814AU devices too.  

<u>If one USB-ID is missing, please mail me.</u>  

**NEWS:**  
added driver (more) driver sources :  
- v4.2.2 (I'm started with **THIS**)  
- v4.3.8  
- v4.3.22-beta  

Known differences in the drivers are :  
v4.2.2 supports STA only with wireless extension  
v4.3.8 supports STA/AP maybe monitor, with cfg80211  
v4.3.22-beta STA/AP/Monitor and new USB3 hack  

currently firmware blobs are taken from v4.3.8

Using rtlwifi modparams  
`swenc:  Set to 1 for software crypto`  
`ips:    Set to 0 to not use link power save`  
`debug:  Set debug level (0-5)`  

currently ips is/may be disabled

In 2T2R Modes they are some issues with the bandwith.  
Output form iperf3  
used with Xioami MiWifi : Wifi Chip MT7612E  
`[  4]   0.00-10.00  sec   113 MBytes  94.5 Mbits/sec    0             sender`  
`[  4]   0.00-10.00  sec   112 MBytes  94.2 Mbits/sec                  receiver`  
used with Edimax BR-6208AC : Wifi Chip RTL8812AE  
`[  4]   0.00-10.00  sec  17.3 MBytes  14.5 Mbits/sec    0             sender`  
`[  4]   0.00-10.00  sec  17.0 MBytes  14.2 Mbits/sec                  receiver`  

Maybe I test this further, if I have time. tough

Building and install driver
---------------------------

for building type  
`make`  

for load the driver  
`sudo insmod rtl8821au.ko`  

You need to install the needed fw with  
`sudo make installfw`  

If you need to crosscompile use  
`ARCH= CROSS_COMPILE= KSRC=`  
while calling `make` i.e.  

`make ARCH="arm" CROSS_COMPILE=armv5tel-softfloat-linux-gnueabi- KSRC=/home/linux-master modules`  

TESTED DEVICES:
---------------
* D-Link DWA 171  
* Digitus Wirelss AC  
* TP-Link T4U AC 1200  
* Linksys WUSB 6300  
* EDUP EP-AC1601  
* TP-Link T1U Nano USB  
and some other unbranded ones.  

with kernel 4.2 and up, lower kernel down to 3.10 will work too.

STATUS:
-------
* Currently driver works with old wireless extension **only**
* Support for 'iw' only sta modes, no AP/Monitor mode !!!

The info about AP mode is my fault and realtek versioning.

ISSUES:
-------
- With low traffic (150kBit/s), the driver will go into low power mode. Currently maybe fixed.

USB3 Mode Issue
---------------
Realtek aka the chipdesigner does some **stupid** idea to switch into USB3 mode via special efuse read/write.  
Documented in `rtl8821au/quirks.c`  
and in the v4.3.22-beta sources `os_dep/linux/os_intfs.c`  
  
They are (mabye) facing some problems if a USB3 capable device is on a USB2 hub. The code does not detect correctly if the device does not switch and will end in an endless loop.  
So they do this via modparam `rtw_switch_usb3`,  
<u>Which is not good.</u>  
Maybe they are other ways to do this ;-)  
And of cources the USB3 hack is currenty **not tested** by me

RTL8813AU/RTL8814AU
-------------------
v4.3.22-beta does not support above devices, there is no hint about the firmware blob and other things. Maybe someone can drop me a note.  
I own a `COMFAST 1750 Mbit WLAN WiFi` to test this it's a RTL8813AU device.

TODO:
-----
- more checkpatch fixes, code rewriting
- more adjustments for rtlwifi
- for a iterim stage switch to lib80211
- regulation fix for 2.4G/5G band (errors currently disabled)
- move into to rtlwifi, going upstream  

Adding cfg80211 interface is out of scope because of missing concurrency with STA/AP/Monitor Modes together. 

Hans Ulli Kroll <ulli.kroll@googlemail.com>
