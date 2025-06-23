# **Setting up the RPi4**

This section will be used to describe the setup of the Raspberry Pi 4 used for development purposes.

## OS image install (24.04 Server LTS)

Before we begin with the installation and configuration of all the necessary software, it is required to install the operating system on our RPi4. To maintain maximum compatibility with the ROS2 Jazzy Jalisco version, we selected the Ubuntu 24.04.x Server LTS operating system. For simplicity, the host computer will also use Ubuntu 24.04 LTS in the Desktop version.

## Raspberry imager

To create the SD card with the selected operating system and initial settings, we will use Raspberry Imager. Install it with:

```bash
sudo apt install rpi-imager
```

After the installation is complete, run Raspberry Imager and select the following settings:

 1. Device -> Raspberry Pi4
 2. Operating System -> Ubuntu Server 24.04.x LTS
 3. Storage -> SD Card
 4. **Next**

In the next window, change the initial settings that make the first access easier by selecting "Edit settings":

**General:**
 1. Set hostname: rasp4.local
 2. Set username and password for default user: *username* and *password*
 3. Configure WLAN: *SSID* and *password* of network you'll use for initial setup
 
 **Services:**
 
 1. Enable SSH: 	Use password authentication
 
Now we can proceed with the installation of the operating system on the SD card.

## Ethernet headless setup
### Raspberry Pi

Since we will use the RPi as a headless device (no keyboard, mouse, or monitor), and the WiFi interface will be required to create and share a wireless network, it is necessary to access the RPi via Ethernet.

To access the RPi device via Ethernet, after connecting via SSH over the previously defined WiFi network, it is necessary to configure the eth0 interface in the **netplan** settings.

Using your preferred editor, open the `/etc/netplan/50-cloud-init.yaml` file and add the following to the current configuration:

```
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses: [192.168.1.10/24]
  wifis:
    wlan0:
      dhcp4: true
      dhcp6: true
      access-points:
        "<wifi SSID>":
          password: "<wifi password>"

```
These settings will assign the IP address 192.168.1.10 to our RPi device.

### Linux host

For the host and Raspberry Pi to connect successfully, it is necessary to assign a static IP address to our Host PC. In the Wired profile settings, select the Ethernet profile, navigate to the IPv4 window, and enter the following:

1.  IPv4 Method: Manual
2.  Addresses: 192.168.1.20
3.  Netmask: 255.255.255.0
4.  Gateway: 0.0.0.0 or leave it empty
5.  Use this connection only for resources on its network

This setting will allow us to maintain the internet connection via the WiFi interface, even though the Raspberry Pi is connected via Ethernet.

Additionally, if the connection is not established automatically, restart the **NetworkManager** service:
```
sudo systemctl restart NetworkManager
```
Now you have an Ethernet connection to the Raspberry Pi, and both your computer and the Raspberry Pi are connected to the external internet via their wireless interfaces.

# Ethernet, fast and simple

RPi:

```
sudo nano /etc/netplan/99-ethernet.yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.50.2/24]
      dhcp4: no
```
And then:
```
sudo netplan apply
```

PC:
```
sudo nano /etc/netplan/99-ethernet.yaml
```

```
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enx4086cbc8819d:
      addresses: [192.168.50.1/24]
      dhcp4: no
```

```
sudo netplan apply
```

Also, check for routes with ```ip route```

# CycloneDDS, normal setup from their manual and then:

PC:
```
export CYCLONEDDS_URI="<CycloneDDS><Discovery><Peers><Peer address=\"192.168.50.2\"/></Peers></Discovery></CycloneDDS>"
```
RPi:
```
export CYCLONEDDS_URI="<CycloneDDS><Discovery><Peers><Peer address=\"192.168.50.1\"/></Peers></Discovery></CycloneDDS>"
```

# UDEV rule for ESP32 Nano:

sudo nano /etc/udev/rules.d/99-my-serial.rules
```
# Rule for ESP32
SUBSYSTEM=="tty", ATTRS{serial}=="3C8427C2F0F0", SYMLINK+="esp_load_cell", MODE="0666"
```