1. Place the tmux.service in /etc/systemd/system folder with say a name of tmux.service
2. Make that your script executable with:
`chmod u+x`
3. Enable it to run at boot:
`sudo systemctl enable tmux`
4. The tmux script should start with

```bash
#!/bin/bash
### BEGIN INIT INFO
# Provides: miner
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi
```

1. The script should not be attaching to the session (at the end), so COMMENT the line: 
    $TMUX_BIN -2 attach-session -t $SESSION_NAME
