# kpick_manager.py ~status: training and running the learned picking policy
bool training                # kpick.sh train is running
string train_job             # job id, once the server gave one
string train_stage           # last progress line of kpick_train.sh
string train_error           # why the last training failed; empty if it did not
string[] results             # result directories, newest first
string policy                # running policy launch: "" (none), "preview" or "execute"
string policy_result         # result directory the running policy uses
string policy_status         # /kpick_policy/status as one line, when the node publishes it
string policy_state          # its state: idle, running, awaiting_step, stopped; "" if unknown
bool step_mode               # the node shows the next steps and runs them on ~step
string message
