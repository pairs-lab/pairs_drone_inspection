#!/bin/bash

SESSION_NAME=warehouse

# send ctrl-c to every pane, give the processes a moment, then kill the session
for pane in $(tmux list-panes -s -t "$SESSION_NAME" -F '#{pane_id}' 2>/dev/null); do
  tmux send-keys -t "$pane" C-c
done
sleep 2
tmux kill-session -t "$SESSION_NAME"
