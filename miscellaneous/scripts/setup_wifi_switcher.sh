touch /etc/systemd/system/wifi_switcher.service
> /etc/systemd/system/wifi_switcher.service
echo "[Unit]
Description=MRS WiFi Switcher service
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=1
ExecStart=/home/mrs/git/uav_core/miscellaneous/scripts/wifi_switcher.sh
[Install]
WantedBy=multi-user.target" >> /etc/systemd/system/wifi_switcher.service
systemctl daemon-reload
systemctl enable wifi_switcher
systemctl start wifi_switcher
