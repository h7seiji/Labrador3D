#create startup script
sudo echo "#run labrador3d" > '/home/citi/startup.sh'
sudo echo "PASS=citi" >> '/home/citi/startup.sh'
sudo echo "cd /home/citi/labrador3d/appBoard/" >> '/home/citi/startup.sh'
sudo echo "echo $PASS | sudo bash labrador3d.sh" >> '/home/citi/startup.sh'
#configure startup script
#sudo nano .config/lxsession/LXDE/autostart
sudo echo "@sh /home/citi/startup.sh" >> '/home/citi/.config/lxsession/LXDE/autostart'
#configure network settings
sudo echo "# Interfaces(5) file used by ifup(8) and ifdown(8)" > '/etc/network/interfaces'
sudo echo "# Include files from /etc/network/interfaces.d" >> '/etc/network/interfaces'
sudo echo "source-directory /etc/network/interfaces.d" >> '/etc/network/interfaces'
sudo echo "" >> '/etc/network/interfaces'
sudo echo "auto eth0" >> '/etc/network/interfaces'
sudo echo "iface eth0 inet static" >> '/etc/network/interfaces'
sudo echo "address 10.1.1." >> '/etc/network/interfaces'
sudo echo "netmask 255.255.255.0" >> '/etc/network/interfaces'
sudo echo "" >> '/etc/network/interfaces'
sudo echo "#auto eth0" >> '/etc/network/interfaces'
sudo echo "#iface eth0 inet dhcp" >> '/etc/network/interfaces'
#manually assign different port numbers for each board
sudo nano /home/citi/labrador3d/appBoard/labrador3d.sh
#manually assign a unique valid IP address for the board (just add the last number)
sudo nano /etc/network/interfaces
sudo ifconfig eth0 up
sudo /etc/init.d/networking restart
