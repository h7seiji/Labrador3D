#configure network settings (run with sudo)
#add last number of ip with
#sudo nano /etc/network/interfaces
sudo echo "# interfaces(5) file used by ifup(8) and ifdown(8)" > '/etc/network/interfaces'
sudo echo "# Include files from /etc/network/interfaces.d" >> '/etc/network/interfaces'
sudo echo "source-directory /etc/network/interfaces.d" >> '/etc/network/interfaces'
sudo echo "auto eth0" >> '/etc/network/interfaces'
sudo echo "iface eth0 inet static" >> '/etc/network/interfaces'
sudo echo "address 10.1.1." >> '/etc/network/interfaces'
sudo echo "netmask 255.255.255.0" >> '/etc/network/interfaces'
sudo echo "#auto eth0" >> '/etc/network/interfaces'
sudo echo "iface eth0 inet dhcp" >> '/etc/network/interfaces'
