touch /etc/systemd/system/uav_configurator.service
> /etc/systemd/system/uav_configurator.service
echo "[Unit]
Description=MRS UAV Configurator service
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=1
ExecStart=/home/mrs/git/uav_core/miscellaneous/configurator_scripts/configurator.sh
[Install]
WantedBy=multi-user.target" >> /etc/systemd/system/uav_configurator.service
systemctl daemon-reload
systemctl enable uav_configurator
systemctl start uav_configurator

touch /etc/systemd/system/uav_configurator_bluetooth.service
> /etc/systemd/system/uav_configurator_bluetooth.service
echo "[Unit]
Description=MRS UAV Bluetooth Configurator service
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
ExecStart=/usr/bin/python3 /home/mrs/git/uav_core/miscellaneous/configurator_scripts/bt_conf/bt_app.py
[Install]
WantedBy=multi-user.target" >> /etc/systemd/system/uav_configurator_bluetooth.service
systemctl daemon-reload
systemctl enable uav_configurator_bluetooth
systemctl start uav_configurator_bluetooth
