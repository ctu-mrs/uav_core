# UAV tmux scripts

Example of tmux scritps that are used to run the system on a real UAV.
Do NOT use for simulation.
The simulation scripts are located in the *simulation* repository.

## What to customize

* PROJECT_NAME=just_flying
  * this defines where the logs and rosbags appear (~/bag_files/$PROJECT_NAME/)
* input=(...
  * the pre-defined sequence of pairs {window_name, command} that is send into each new tmux window
* init_window="Status"
  * the name of the windos (window_name) that is going to be focused after the start
* if you start the session via systemclt service, comment the following bottom part:

```bash
...

$TMUX_BIN -2 attach-session -t $SESSION_NAME

clear
```
