#!/bin/bash
# Pre-flight sanity check for the REAL inspection drone. Run AFTER start.sh has brought the
# stack up and BEFORE arming. Verifies the FCU link, sensor rates, TF tree, and the estimator.
# Non-zero exit = at least one check failed. Read every WARN before you fly.
UAV_NAME=${UAV_NAME:-uav1}
FAIL=0

say()  { printf "  %-46s %s\n" "$1" "$2"; }
ok()   { say "$1" "OK"; }
warn() { say "$1" "!! $2"; FAIL=1; }

hz() { # topic, min_hz
  local r
  r=$(timeout 4 rostopic hz "$1" 2>/dev/null | grep -m1 average | grep -oE '[0-9.]+' | head -1)
  if [ -z "$r" ]; then warn "rate $1" "no messages"; return; fi
  awk -v r="$r" -v m="$2" 'BEGIN{exit !(r+0 >= m)}' \
    && ok "rate $1 (${r} Hz)" || warn "rate $1" "${r} Hz < ${2} Hz"
}
has_tf() { rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q Translation \
    && ok "tf $1 -> $2" || warn "tf $1 -> $2" "missing"; }

echo "== inspection_real preflight ($UAV_NAME) =="

echo "-- environment --"
[ "$RUN_TYPE" = "realworld" ] && ok "RUN_TYPE=realworld" || warn "RUN_TYPE" "is '$RUN_TYPE' (must be realworld)"
[ -e /dev/pixhawk ] && ok "/dev/pixhawk present" || warn "/dev/pixhawk" "no FCU serial symlink"

echo "-- FCU / hw_api --"
CONN=$(timeout 4 rostopic echo -n1 /$UAV_NAME/mavros/state 2>/dev/null | grep -m1 'connected:' )
echo "$CONN" | grep -q True && ok "mavros connected" || warn "mavros" "FCU not connected"
hz /$UAV_NAME/hw_api/imu 80
hz /$UAV_NAME/hw_api/distance_sensor 8

echo "-- sensors --"
hz /$UAV_NAME/livox/points 8
hz /$UAV_NAME/point_lio/odom 20
hz /$UAV_NAME/front_rgbd/color/image_raw 10
hz /$UAV_NAME/tag_detections 5

echo "-- TF tree --"
has_tf $UAV_NAME/fcu $UAV_NAME/livox
has_tf $UAV_NAME/world_origin $UAV_NAME/fcu

echo "-- estimator --"
EST=$(timeout 4 rostopic echo -n1 /$UAV_NAME/estimation_manager/diagnostics 2>/dev/null)
echo "$EST" | grep -q 'point_lio' && ok "estimator = point_lio" || warn "estimator" "point_lio not active"

echo "-- battery --"
V=$(timeout 4 rostopic echo -n1 /$UAV_NAME/mavros/battery 2>/dev/null | grep -m1 'voltage:' | grep -oE '[0-9.]+')
[ -n "$V" ] && ok "battery ${V} V" || warn "battery" "no reading"

echo
[ "$FAIL" = 0 ] && echo "PREFLIGHT PASS — clear to arm (visual + RC check still required)." \
                 || echo "PREFLIGHT FAILED — resolve the !! items before arming."
exit $FAIL
