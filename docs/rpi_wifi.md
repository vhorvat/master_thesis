

## **Comprehensive guide: Raspberry Pi 4 as Wi-Fi AP (Sharing PC's Internet (or not!))**

**Goal:**

-   PC (Ethernet, Internet via Wi-Fi) -> RPi (Ethernet, Static IP) -> RPi (WLAN, Access Point) -> Other devices
    

**Assumptions:**

-   **PC's Wi-Fi Interface (Internet):**  wlp2s0 
-   **PC's Ethernet Interface (to RPi):**  enx4086cbc8819d (check with ip link show)
-   **RPi's Ethernet Interface:**  eth0
-   **RPi's WLAN Interface:**  wlan0
-   **Desired SSID:**  MyRPiAP 
-   **Desired Password:**  MySecurePassword 
-   **Wi-Fi IP Address Range:** 192.168.4.0/24 (RPi's wlan0 will be 192.168.4.1)
-   **RPi OS:** Ubuntu Server 24.04 LTS (uses netplan)
-   **PC OS:** Ubuntu Desktop 24.04 LTS (uses NetworkManager)
-   **PC Router IP:**  192.168.1.254 (your ISP gateway)
    

**Steps on the Raspberry Pi (Ubuntu Server 24.04 LTS):**

1.  **Install Packages:**
   
    ```
    sudo apt update
    sudo apt install hostapd dnsmasq iptables-persistent netfilter-persistent -y
    ```
        
2.  **Configure hostapd:**
    
    ```
    sudo nano /etc/hostapd/hostapd.conf
    ```
    
    Paste this, changing ssid and wpa_passphrase:
    
    ```
    interface=wlan0
    driver=nl80211
    ssid=MyRPiAP
    hw_mode=g
    channel=6
    macaddr_acl=0
    auth_algs=1
    ignore_broadcast_ssid=0
    wpa=2
    wpa_passphrase=MySecurePassword
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=TKIP
    rsn_pairwise=CCMP
    ```
    
    **Important:**
   
       -   hw_mode: Use hw_mode=a for 5GHz, not really needed for our use

        
3.  **Configure dnsmasq:**
    
    ```
    sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
    sudo nano /etc/dnsmasq.conf
    ```
    
    Paste this:
  
    ```
    interface=wlan0
    bind-interfaces
    server=8.8.8.8
    server=8.8.4.4
    domain-needed
    bogus-priv
    dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,12h
    ```
 
        
4.  **Configure netplan (RPi):**
    
    ```
    sudo nano /etc/netplan/50-cloud-init.yaml  # Or your netplan file
    ```

    
    Replace the entire contents:
    
    ```
    network:
      version: 2
      ethernets:
        eth0:
          dhcp4: false
          addresses: [192.168.1.10/24]
          routes:
            - to: default
              via: 192.168.1.254
          nameservers:
            addresses: [8.8.8.8, 8.8.4.4]
      wifis:
        wlan0:
          dhcp4: false
          dhcp6: false
          access-points:
            "MyRPiAP":  
              password: "MySecurePassword" 
          addresses: [192.168.4.1/24]
    ```
    

    Apply: sudo netplan apply or use sudo netplan try which will give you 120s timeout option
        
5.  **Enable IP forwarding (RPi):**
    
    ```
    sudo nano /etc/sysctl.conf
    ```
    
    Uncomment: net.ipv4.ip_forward=1
    
    Apply: sudo sysctl -p
   
    
7.  **Configure NAT with iptables (RPi):**
    
    ```
    sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
    sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
    sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
    sudo netfilter-persistent save
    sudo netfilter-persistent reload
    ```
   
    
    **Troubleshooting:**
    
    -   sudo iptables -L -v -n and sudo iptables -t nat -L -v -n: Check rules.
        
    -   If FORWARD policy is DROP, add: sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
        
8.  **Start and enable services (RPi):**
    
    ```
    sudo systemctl unmask hostapd
    sudo systemctl enable hostapd
    sudo systemctl start hostapd
    sudo systemctl enable dnsmasq
    sudo systemctl start dnsmasq
    ```
    
9.  **Reboot the Raspberry Pi:**
    
    ```
    sudo reboot
    ```
    

**Steps on the PC (Ubuntu Desktop 24.04 LTS):**

1.  **Run the Configuration Script:** This script does everything on the PC side: enables IP forwarding, sets up NAT, adds the static route, and persists the iptables rules using the most robust method. Copy and paste this entire script into your PC's terminal and run it.

Mind you, this script will work will static interfaces used on Viktors ASUS laptop, one might want to change it so that it can get the interface names dynamically and be totally independent!

```
#!/bin/bash  

# Enable IP forwarding (temporarily) 
sudo sysctl -w net.ipv4.ip_forward=1 

# Make IP forwarding permanent 
sudo sed -i 's/#net.ipv4.ip_forward=1/net.ipv4.ip_forward=1/' /etc/sysctl.conf 
sudo sysctl -p 

# Delete any existing NAT rules in POSTROUTING 
sudo iptables -t nat -F POSTROUTING 

# Add the correct NAT rule 
sudo iptables -t nat -A POSTROUTING -s 192.168.4.0/24 -o wlp2s0 -j MASQUERADE 

# Add forwarding rules 
sudo iptables -A FORWARD -i enx4086cbc8819d -o wlp2s0 -j ACCEPT sudo iptables -A FORWARD -i wlp2s0 -o enx4086cbc8819d -m state --state RELATED,ESTABLISHED -j ACCEPT 

# Add the static route (temporarily) 
sudo ip route add 192.168.4.0/24 via 192.168.1.10 

# --- Make the static route permanent (NetworkManager) ---
# Get the connection name for the wired interface
CONNECTION_NAME=$(nmcli -f NAME,DEVICE connection show | awk '$2 == "enx4086cbc8819d" {print $1}')

if [ -z "$CONNECTION_NAME" ]; then
  echo "ERROR: Could not find NetworkManager connection for enx4086cbc8819d"
  exit 1
fi

sudo nmcli connection modify "$CONNECTION_NAME" +ipv4.routes "192.168.4.0/24 192.168.1.10"

# --- Save and Restore iptables Rules (systemd) ---  
# Save the current rules 
sudo iptables-save > /etc/iptables/rules.v4 sudo ip6tables-save > /etc/iptables/rules.v6 

# Create the systemd service file
sudo tee /etc/systemd/system/iptables-restore.service << EOF
[Unit]
Description=Restore iptables rules
Before=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/sbin/iptables-restore < /etc/iptables/rules.v4
ExecStart=/sbin/ip6tables-restore < /etc/iptables/rules.v6
ExecReload=/sbin/iptables-restore < /etc/iptables/rules.v4
ExecReload=/sbin/ip6tables-restore < /etc/iptables/rules.v6
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Enable and start the service
sudo systemctl enable iptables-restore.service
sudo systemctl start iptables-restore.service
```

   