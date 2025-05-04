sudo raspi-config
nmcli radio wifi
nmcli radio wifi on
nmcli general status
sudo systemctl disable dhcpcd
sudo systemctl stop dhcpcd
sudo systemctl enable NetworkManager
sudo systemctl start NetworkManager
nmcli general status
nmcli radio wifi
nmcli dev wifi list
sudo nmcli devf wifi connect 3c3a password 11111113
sudo nmcli dev wifi connect 3c3a password 11111113
sudo raspi-config

sudo systemctl start ssh
sudo raspi-config
sudo raspi-config nonint do_i2c 1
sudo raspi-config nonint do_serial_hw 1
sudo raspi-config
sudo raspi-config nonint do_serial_hw 1
sudo nano /boot/config.txt
grep -ir do_serial_hw /boot
grep -ir do_serial_hw /etc
sudo grep -ir do_serial_hw /etc
cat /boot/cmdline.txt
dmesg | grep do_serial_hw
sudo raspi-config nonint do_serial_hw 1
sudo raspi-config
ls
sudo python3 stb2.py
sudo pip3 install psutil
sudo python3 stb2.py
i2cdetect -y 1
sudo python3 stb2.py
i2cdetect -y 1
nano stb3.py
chmod +x stb3.py
sudo python3 stb3.py
sudo systemctl poweroff
