# Onboard-Systems

# Getting the Pi back onto wifi after reflash

1. Connect the Pi to a phone hotspot or by ethernet

2. Install `ca-certificates` and `network-manager-gnome`

    sudo apt-get install ca-certificates network-manager-gnome

`ca-certificates` will contain the SSL certificates necessary for connecting to Western's wifi

`network-manager-gnome` is a much better wifi connection management utility. So far, it's the only one that's been able to connect to wifi in all places.

3. edit /etc/NetworkManager/NetworkManager.conf

change `managed=false` to `managed=true`

4. Disable and stop the default connection manager

    sudo systemctl stop networking
    sudo systemctl disable networking

5. Enable and start NetworkManager

    sudo systemctl start NetworkManager
    sudo systemctl enable NetworkManager

Right click top bar, add Network Manager to the bar

Follow the instructions [on Western's page](http://uwo.ca/its/hdi/wireless/computers/linux-ubuntu.html) to connect to Western's wifi
