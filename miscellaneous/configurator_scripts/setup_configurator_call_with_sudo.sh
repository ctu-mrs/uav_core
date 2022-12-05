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
