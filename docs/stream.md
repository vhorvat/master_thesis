# WiFi HaLow video demo:
This is a quick guide on how to set-up a video stream
on NRC HaLow units. 

Ensure you're running DHCP on both of RPis.
Also, check for IPs.

Static IP will be given to the HaLow AP, everything else is on DHCP lease.

For this demo and current situation:

| Device | Interface | Expected IP (example)     |
|--------|-----------|---------------------------|
| PC     | `enx4086cbc8819d`    | `10.42.0.1`               |
| Pi     | `eth0`    | `10.42.0.184`             |
| Pi     | `wlan0`   | `192.168.200.38`          |
| Pi2    | `wlan0`   | `192.168.200.1`           |

Confirm IPs by using ```bash ip a``` on every device

## Network part

### PC (10.42.0.1)

Add a static route to the AP RPi network over STA Pi.
STA Pi will be 'gateway' for this interfce.

```bash
sudo ip route add 192.168.200.0/24 via 10.42.0.184
```

### STA Pi (10.42.0.1)
Enable forwarding:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
```

Setup forwarding rules:
```bash
sudo iptables -t nat -A POSTROUTING -o wlan0 -s 10.42.0.0/24 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state ESTABLISHED,RELATED -j ACCEPT
```
### STA Pi (10.42.0.1)
```bash
sudo ip route add 10.42.0.0/24 via 192.168.200.38
```

At the end test with ping.

## ffmpeg/VLC part

Ensure camera is on /dev/video0 interface.

Smallest latency has been achived by using following encoding setitngs.
Some of them like resolution and framerate depend on the camera.

```bash
ffmpeg -f v4l2 -framerate 20 -video_size 800x448 -i /dev/video0 -vcodec h264_v4l2m2m -tune zerolatency -preset ultrafast -g 30 -keyint_min 30 -f mpegts udp://10.42.0.1:1234 
```

On the PC, open the UDP stream with:
```bash
vlc --network-caching=100 udp://@10.42.0.1:1234
```

Seeing some frame decode/buffer errors is normal as we're using small buffer.