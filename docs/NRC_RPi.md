# Fast & Simple, NRC 7394, 6.6 Debian Bookworm based RaspOS

## Linux Installation
On your SD install RaspOS 64 bit using RaspImager.

We can install arm64 because from Bookworm, 32 bit images decouple userland and kernel with 32/64 bit versions anyway.
(JOŠ SE NE MIRIM S OVOM GLUPOSTI JEBOTE.)

Install 6.6 version, ideally with 6.6.51 kernel, as this guide will be based on EXACTLY this kernel.
If you choose to do kernel uplift, BE AWARE, you will be fixing bugs that will not be part of this walkthrough!

For ease of installation, use WiFi connected to internet (Mobile AP, or whatever).
To set-up initial WiFi config, use RaspiImager settings, or modify wpa-supplicant.conf on the SD.

Apart from WiFi settings, you will need to enable SSH too!

Ethernet WILL BE required in later steps.!

### RPi
After successfully SSHing to your RPi, DO NOT UPDATE RPI!
We need to freeze kernel updates:

```bash
sudo apt-mark hold raspberrypi-kernel
sudo apt-mark hold linux-image-6.6.51+rpt-rpi-v8
sudo apt-mark hold linux-headers-6.6.51+rpt-rpi-v8
```

AND:

```bash
sudo nano /etc/apt/preferences.d/no-kernel-upgrade
```

```
put:
paste:
Package: linux-image*
Pin: release *
Pin-Priority: -1

Package: linux-headers*
Pin: release *
Pin-Priority: -1

Package: raspberrypi-kernel
Pin: release *
Pin-Priority: -1
```

now do sudo update, upgrade whatever.

### We need to switch away from NetworkManager!

Next step will probabbly fuck up your internet connection:
Best way is to manually reconnect to WiFi by providing necessary wpa-config before this step.

or just connect Rpi to your router, it really is a lot easier.

```bash

sudo apt-get update
sudo apt-get install -y dhcpcd5 iptables hostapd dnsmasq
sudo cp /usr/share/dhcpcd/hooks/10-wpa_supplicant /lib/dhcpcd/dhcpcd-hooks/
sudo reboot
```

Također, iptables:

```bash
sudo apt install iptables
```


### You can now follow everything else from their guide.
While builiding kernel module, you will encounter errors.
Fix them one by one if no .patch is provided a to bude kada bude.


## ROS2 Jazzy (prebuilt za Debian Bookworm)

```bash
wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo apt install ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
sudo pip install --break-system-packages vcstool colcon-common-extensions
```

everything else is ROS2 classic setup.
Also! Don't forget, since we never added sources, EVERY package will need to be built and sourced from the source code!