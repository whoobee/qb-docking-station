# Stop the kiosk service
sudo systemctl stop kiosk

# Remove the kiosk service from startup
sudo systemctl disable kiosk

# Remove the kiosk service
sudo rm -f /etc/systemd/system/kiosk.service

# Reload systemctl daemons
sudo systemctl daemon-reload

# Remove the kiosk startup file and directory
sudo rm -rf /opt/kiosk

# Remove X.org, OpenBox, Firefox and autoremove associated programs no longer in use
sudo apt remove --autoremove -y xorg openbox firefox
