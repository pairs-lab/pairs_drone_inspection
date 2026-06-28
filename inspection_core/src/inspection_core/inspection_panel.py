#!/usr/bin/env python
"""PAIRS Inspection rqt panel.

A single rqt plugin that drives the whole warehouse inspection drone, replacing
both the old standalone Tkinter ``relative_navigator`` GUI *and* the separate
``pairs_rqt_control`` flight window — everything is now one panel:

* flight control -- arm / offboard / one-click takeoff / land / hover / e-land,
  and a free ``goto`` (collision-free via the octomap planner, or straight-line);
* relative rack/bin navigation -- auto-find a rack via its anchor AprilTag,
  zig-zag through the 18 bins, visual-servo to centre each bin tag;
* precise landing on the charging dock (go-to-dock / land / abort);
* a live camera feed (tag-detection overlay, front colour/IR, or the down
  landing cam), embedded as a QLabel fed via cv_bridge.

Navigation goto's are routed through ``octomap_planner/goto`` (collision-free,
routes AROUND the racks) when the planner is up, falling back to the straight
``control_manager/goto`` otherwise; fine visual-servo nudges always go direct.

Long, blocking flight sequences run on worker threads; the camera frame is
marshalled to the GUI thread with a Qt signal.
"""
import math
import os
import threading
import time

import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QLineEdit, QSizePolicy, QDoubleSpinBox,
)
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QImage, QPixmap

from std_srvs.srv import Trigger, SetBool

# Optional deps -- keep the plugin importable even if the workspace overlay
# isn't sourced (matches pairs_rqt_control's graceful-degradation pattern).
try:
    from pairs_msgs.srv import Vec4, Vec4Request
    from pairs_msgs.msg import ControlManagerDiagnostics, HwApiStatus
    _HAVE_VEC4 = True
except Exception:
    _HAVE_VEC4 = False

try:
    from apriltag_ros.msg import AprilTagDetectionArray
    _HAVE_TAGS = True
except Exception:
    _HAVE_TAGS = False

try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _HAVE_IMG = True
except Exception:
    _HAVE_IMG = False


# Selectable camera feeds: (label, topic suffix under /<uav>/)
CAMERA_FEEDS = [
    ("Tag detections", "tag_detections_image"),
    ("Front colour", "front_rgbd/color/image_raw"),
    ("Front IR", "front_rgbd/infra1/image_raw"),
    ("Down landing cam", "bluefox_optflow/image_raw"),
]

# Rack layout (must match warehouse.world anchor tags + relative_navigator.py)
RACKS_CONFIG = {
    1: {"id": 101, "y_start": 6.3, "dir": 1},
    2: {"id": 102, "y_start": 3.7, "dir": -1},
    3: {"id": 103, "y_start": 1.3, "dir": 1},
    4: {"id": 104, "y_start": -1.3, "dir": -1},
    5: {"id": 105, "y_start": -3.7, "dir": 1},
    6: {"id": 106, "y_start": -6.3, "dir": -1},
}
ZIGZAG_ORDER = [1, 2, 3, 6, 5, 4, 7, 8, 9, 12, 11, 10, 13, 14, 15, 18, 17, 16]
BIN_TAG_IDS = (0, 1, 2)  # bin-centring fiducials per config/tags.yaml (distinct from anchor ids 101-106)
CORRIDOR_X = -5.5        # shared approach corridor
BASE_Z = 1.5


class InspectionPanel(Plugin):

    # emitted from the ROS image thread, consumed on the GUI thread
    _image_ready = Signal(QImage)

    def __init__(self, context):
        super(InspectionPanel, self).__init__(context)
        self.setObjectName('InspectionPanel')

        self.uav_name = os.environ.get('UAV_NAME', 'uav1')

        # ---- navigation state (ported from relative_navigator.py) ----
        self.BAY_WIDTH = 2.70
        self.LEVEL_HEIGHT = 1.20
        self.STANDOFF_DEPTH = 1.50
        self.tag_detected = False
        self.lock_anchor = False
        self.servo_active = False
        self.servo_error_x = None
        self.servo_error_y = None
        self.anchor_x = self.anchor_y = self.anchor_z = 0.0
        self.current_bin = None
        self.last_x, self.last_y, self.last_z = CORRIDOR_X, 0.0, BASE_Z
        self.selected_rack = 3

        # ---- flight status (from pairs_rqt_control) ----
        self._diag = None
        self._hw = None
        self._sub_diag = None
        self._sub_hw = None
        # one-click takeoff retry counters
        self._tk_arm_tries = 0
        self._tk_loop_tries = 0

        self._bridge = CvBridge() if _HAVE_IMG else None
        self._img_sub = None
        self._tag_sub = None
        self._busy = False
        self._shutting_down = False

        self._build_ui(context)
        self._image_ready.connect(self._on_image)
        self._subscribe_tags()
        self._subscribe_status()
        self._select_feed(0)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(500)

    # ----------------------------------------------------------------- UI
    def _build_ui(self, context):
        w = QWidget()
        w.setObjectName('InspectionPanelUi')
        w.setWindowTitle('PAIRS Inspection Control')
        root = QHBoxLayout(w)

        left = QVBoxLayout()

        # UAV namespace
        row = QHBoxLayout()
        row.addWidget(QLabel('UAV:'))
        self._uav_edit = QLineEdit(self.uav_name)
        self._uav_edit.editingFinished.connect(self._on_uav_changed)
        row.addWidget(self._uav_edit)
        left.addLayout(row)

        # live flight status line (armed / offboard / tracker / flying)
        self._flight_status = QLabel(u'—')
        self._flight_status.setStyleSheet('font-family: monospace; padding: 2px;')
        left.addWidget(self._flight_status)

        # flight controls (merged from pairs_rqt_control). These bypass the
        # nav "busy" guard so Hover / E-Land / Land always work mid-mission.
        g_flight = QGroupBox('Flight')
        fg = QGridLayout(g_flight)
        flight_buttons = [
            ('Arm',       lambda: self._call('hw_api/arming', SetBool, True)),
            ('Disarm',    lambda: self._call('hw_api/arming', SetBool, False)),
            ('Offboard',  lambda: self._call('hw_api/offboard', Trigger)),
            ('Takeoff',   self._takeoff_sequence),
            ('Land',      lambda: self._call('uav_manager/land', Trigger)),
            ('Land Home', lambda: self._call('uav_manager/land_home', Trigger)),
            ('Hover',     lambda: self._call('control_manager/hover', Trigger)),
            ('E-Land',    lambda: self._call('control_manager/eland', Trigger)),
        ]
        for i, (label, cb) in enumerate(flight_buttons):
            b = QPushButton(label)
            b.clicked.connect(cb)
            fg.addWidget(b, i // 4, i % 4)
        left.addWidget(g_flight)

        # free go-to (collision-free via planner, or straight-line)
        g_goto = QGroupBox('Go to  (x  y  z  heading)')
        gg = QGridLayout(g_goto)
        self._spin = {}
        for col, (name, default) in enumerate(
                [('x', 0.0), ('y', 0.0), ('z', 1.5), ('heading', 0.0)]):
            sb = QDoubleSpinBox()
            sb.setRange(-1000.0, 1000.0)
            sb.setDecimals(2)
            sb.setSingleStep(0.5)
            sb.setValue(default)
            self._spin[name] = sb
            gg.addWidget(QLabel(name), 0, col)
            gg.addWidget(sb, 1, col)
        b_avoid = QPushButton('Go To (avoid)')
        b_avoid.setStyleSheet('font-weight: bold;')
        b_avoid.clicked.connect(lambda: self._run_bg(self._goto_fields, False))
        b_direct = QPushButton('Go To (direct)')
        b_direct.clicked.connect(lambda: self._run_bg(self._goto_fields, True))
        gg.addWidget(b_avoid, 2, 0, 1, 2)
        gg.addWidget(b_direct, 2, 2, 1, 2)
        left.addWidget(g_goto)

        # relative navigation
        g_nav = QGroupBox('Relative navigation')
        nav = QGridLayout(g_nav)
        nav.addWidget(QLabel('Rack:'), 0, 0)
        self._rack_combo = QComboBox()
        self._rack_combo.addItems(['Rack %d' % i for i in range(1, 7)])
        self._rack_combo.setCurrentIndex(2)  # default Rack 3
        self._rack_combo.currentIndexChanged.connect(self._on_rack_changed)
        nav.addWidget(self._rack_combo, 0, 1)
        nav.addWidget(QLabel('Bin:'), 1, 0)
        self._bin_combo = QComboBox()
        self._bin_combo.addItems(['Bin %d' % i for i in range(1, 19)])
        nav.addWidget(self._bin_combo, 1, 1)
        b1 = QPushButton('1 - Auto find rack')
        b1.clicked.connect(lambda: self._run_bg(self.execute_global_approach))
        b2 = QPushButton('2 - Zig-zag to bin')
        b2.clicked.connect(lambda: self._run_bg(
            self.execute_zigzag_sequence, self._selected_bin(), False))
        b3 = QPushButton('3 - Zig-zag inspect rack')
        b3.setStyleSheet('font-weight: bold;')
        b3.clicked.connect(lambda: self._run_bg(
            self.execute_zigzag_sequence, 18, True))
        nav.addWidget(b1, 2, 0, 1, 2)
        nav.addWidget(b2, 3, 0, 1, 2)
        nav.addWidget(b3, 4, 0, 1, 2)
        self._status = QLabel('Not seeing anchor tag')
        self._status.setStyleSheet('color: red; font-style: italic;')
        nav.addWidget(self._status, 5, 0, 1, 2)
        left.addWidget(g_nav)

        # precise landing on the charging dock
        g_dock = QGroupBox('Precise landing (charging dock)')
        dock = QVBoxLayout(g_dock)
        bd = QPushButton('Go to dock  (-6, 0, 2)')
        bd.clicked.connect(lambda: self._run_bg(self._goto_dock))
        bl = QPushButton('LAND  (precise)')
        bl.setStyleSheet('font-weight: bold;')
        bl.clicked.connect(lambda: self._run_bg(
            self._call_trigger, 'precise_landing/land'))
        ba = QPushButton('ABORT landing')
        ba.clicked.connect(lambda: self._run_bg(
            self._call_trigger, 'precise_landing/abort'))
        dock.addWidget(bd)
        dock.addWidget(bl)
        dock.addWidget(ba)
        left.addWidget(g_dock)
        left.addStretch(1)
        root.addLayout(left)

        # camera feed
        right = QVBoxLayout()
        feed_row = QHBoxLayout()
        feed_row.addWidget(QLabel('Camera:'))
        self._feed_combo = QComboBox()
        self._feed_combo.addItems([n for n, _ in CAMERA_FEEDS])
        self._feed_combo.currentIndexChanged.connect(self._select_feed)
        feed_row.addWidget(self._feed_combo)
        feed_row.addStretch(1)
        right.addLayout(feed_row)
        self._img_label = QLabel('no image')
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setMinimumSize(360, 270)
        self._img_label.setStyleSheet('background: #222; color: #888;')
        self._img_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right.addWidget(self._img_label, 1)
        root.addLayout(right, 1)

        context.add_widget(w)
        self._widget = w

        if not _HAVE_VEC4:
            self._flight_status.setText(
                'pairs_msgs not found — source the PAIRS workspace before launching')

    # -------------------------------------------------------------- UI cbs
    def _selected_bin(self):
        return self._bin_combo.currentIndex() + 1

    def _on_uav_changed(self):
        new = self._uav_edit.text().strip() or 'uav1'
        if new != self.uav_name:
            self.uav_name = new
            self._subscribe_tags()
            self._subscribe_status()
            self._select_feed(self._feed_combo.currentIndex())

    def _on_rack_changed(self, idx):
        self.selected_rack = idx + 1
        self.tag_detected = False
        self.lock_anchor = False
        self.current_bin = None

    def _refresh_status(self):
        # anchor-tag line
        if self.tag_detected:
            self._status.setText('Locked anchor tag - ready')
            self._status.setStyleSheet('color: green; font-style: italic;')
        else:
            self._status.setText('Not seeing anchor tag')
            self._status.setStyleSheet('color: red; font-style: italic;')
        # flight line
        if not _HAVE_VEC4:
            return
        parts = ['uav=%s' % self.uav_name]
        if self._hw is not None:
            parts.append('armed=%s' % ('Y' if self._hw.armed else 'N'))
            parts.append('offboard=%s' % ('Y' if self._hw.offboard else 'N'))
        else:
            parts.append('hw_api: no data')
        if self._diag is not None:
            parts.append('tracker=%s' % self._diag.active_tracker)
            parts.append('flying=%s' % ('Y' if self._diag.flying_normally else 'N'))
        self._flight_status.setText('   '.join(parts))

    # ----------------------------------------------------------- threading
    def _run_bg(self, fn, *args):
        if self._busy:
            rospy.logwarn('inspection_rqt: busy, ignoring command')
            return

        def wrap():
            self._busy = True
            try:
                fn(*args)
            except Exception as e:  # noqa: BLE001
                rospy.logerr('inspection_rqt task error: %s', e)
            finally:
                self._busy = False

        threading.Thread(target=wrap, daemon=True).start()

    # ------------------------------------------------------------- ROS I/O
    def _call(self, srv, srv_type, *args):
        """Generic service call (flight controls); returns success bool."""
        full = '/%s/%s' % (self.uav_name, srv)
        try:
            rospy.wait_for_service(full, timeout=2.0)
            resp = rospy.ServiceProxy(full, srv_type)(*args)
            ok = bool(getattr(resp, 'success', True))
            rospy.loginfo('%s -> %s %s', full, 'OK' if ok else 'FAIL',
                          getattr(resp, 'message', ''))
            return ok
        except Exception as e:  # noqa: BLE001
            rospy.logerr('%s failed: %s', full, e)
            return False

    def _call_trigger(self, srv):
        return self._call(srv, Trigger)

    def _nav_goto_service(self):
        """Prefer the collision-free planner; fall back to straight-line goto."""
        planner = '/%s/octomap_planner/goto' % self.uav_name
        try:
            rospy.wait_for_service(planner, timeout=0.5)
            return 'octomap_planner/goto'
        except Exception:  # noqa: BLE001
            return 'control_manager/goto'

    def send_flight_command(self, x, y, z, heading, direct=False):
        """Fly to (x,y,z,heading) in the world frame.

        ``direct=False`` routes through the octomap planner (collision-free)
        when it is available; ``direct=True`` always uses the straight-line
        control_manager/goto (for short visual-servo nudges the planner rejects).
        """
        if not _HAVE_VEC4:
            rospy.logerr('pairs_msgs/Vec4 unavailable; cannot command goto')
            return False, 0.0
        srv = 'control_manager/goto' if direct else self._nav_goto_service()
        full = '/%s/%s' % (self.uav_name, srv)
        try:
            rospy.wait_for_service(full, timeout=5.0)
            req = Vec4Request()
            req.goal = [float(x), float(y), float(z), float(heading)]
            resp = rospy.ServiceProxy(full, Vec4)(req)
            if resp.success:
                dist = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2
                                 + (z - self.last_z) ** 2)
                self.last_x, self.last_y, self.last_z = float(x), float(y), float(z)
                return True, dist
            rospy.logwarn('%s rejected: %s', srv, getattr(resp, 'message', ''))
            return False, 0.0
        except Exception as e:  # noqa: BLE001
            rospy.logerr('goto failed: %s', e)
            return False, 0.0

    def _goto_fields(self, direct):
        x = self._spin['x'].value()
        y = self._spin['y'].value()
        z = self._spin['z'].value()
        h = self._spin['heading'].value()
        rospy.loginfo('inspection_rqt: goto (%.2f, %.2f, %.2f, %.2f) %s',
                      x, y, z, h, 'direct' if direct else 'avoid')
        self.send_flight_command(x, y, z, h, direct=direct)

    def _goto_dock(self):
        rospy.loginfo('inspection_rqt: flying over the charging dock (-6, 0, 2)')
        self.send_flight_command(-6.0, 0.0, 2.0, 0.0)

    # ------------------------------------------- one-click takeoff sequence
    # arm -> control output -> offboard -> takeoff, chained with non-blocking
    # timers (ported from pairs_rqt_control). PX4 SITL often rejects the first
    # arm command(s) and drops OFFBOARD on the ground unless takeoff follows
    # quickly, so the tail retries on failure.
    def _takeoff_sequence(self):
        self._tk_arm_tries = 0
        self._tk_loop_tries = 0
        rospy.loginfo('takeoff: arming...')
        self._tk_arm()

    def _tk_arm(self):
        if (self._hw is not None and self._hw.armed) or self._call('hw_api/arming', SetBool, True):
            QTimer.singleShot(1500, self._tk_output)
        elif self._tk_arm_tries < 15:
            self._tk_arm_tries += 1
            QTimer.singleShot(1000, self._tk_arm)
        else:
            rospy.logerr('takeoff: could not arm after %d attempts', self._tk_arm_tries)

    def _tk_output(self):
        self._call('control_manager/toggle_output', SetBool, True)
        QTimer.singleShot(600, self._tk_offboard)

    def _tk_offboard(self):
        self._call('hw_api/offboard', Trigger)
        QTimer.singleShot(700, self._tk_takeoff)

    def _tk_takeoff(self):
        if self._call('uav_manager/takeoff', Trigger):
            return
        if self._tk_loop_tries < 4:
            self._tk_loop_tries += 1
            QTimer.singleShot(700, self._tk_arm)

    # ------------------------------------------------------ status callbacks
    def _subscribe_status(self):
        if not _HAVE_VEC4:
            return
        for s in (self._sub_diag, self._sub_hw):
            if s is not None:
                s.unregister()
        self._diag = None
        self._hw = None
        self._sub_diag = rospy.Subscriber(
            '/%s/control_manager/diagnostics' % self.uav_name,
            ControlManagerDiagnostics, self._on_diag, queue_size=1)
        self._sub_hw = rospy.Subscriber(
            '/%s/hw_api/status' % self.uav_name,
            HwApiStatus, self._on_hw, queue_size=1)

    def _on_diag(self, msg):
        self._diag = msg

    def _on_hw(self, msg):
        self._hw = msg

    # --------------------------------------------------------- tag callback
    def _subscribe_tags(self):
        if not _HAVE_TAGS:
            rospy.logwarn('apriltag_ros msgs unavailable; tag anchoring disabled')
            return
        if self._tag_sub is not None:
            self._tag_sub.unregister()
        self._tag_sub = rospy.Subscriber(
            '/%s/tag_detections' % self.uav_name,
            AprilTagDetectionArray, self._tag_cb, queue_size=10)

    def _tag_cb(self, msg):
        if getattr(self, 'lock_anchor', False):
            return
        target_id = RACKS_CONFIG[self.selected_rack]["id"]
        rack_y = RACKS_CONFIG[self.selected_rack]["y_start"]
        for det in msg.detections:
            if target_id in det.id:
                p = det.pose.pose.pose.position
                self.anchor_x = CORRIDOR_X + p.z
                self.anchor_y = rack_y - p.x
                self.anchor_z = BASE_Z - p.y
                self.tag_detected = True
                return
        # bin-centring fiducial (nearest by depth) for the visual servo
        if self.servo_active:
            best_z, bx, by = 999.0, None, None
            for det in msg.detections:
                if any(t in det.id for t in BIN_TAG_IDS):
                    cz = det.pose.pose.pose.position.z
                    if cz < best_z:
                        best_z = cz
                        bx = det.pose.pose.pose.position.x
                        by = det.pose.pose.pose.position.y
            if bx is not None:
                self.servo_error_x = bx
                self.servo_error_y = by

    # --------------------------------------------------- navigation phases
    def execute_global_approach(self):
        """Phase 1: fly to the shared corridor and along the chosen rack."""
        rack_y = RACKS_CONFIG[self.selected_rack]["y_start"]
        target_id = RACKS_CONFIG[self.selected_rack]["id"]
        rospy.loginfo('[Phase 1] approaching rack %d (y=%.1f) anchor %d',
                      self.selected_rack, rack_y, target_id)
        self.tag_detected = False
        self.lock_anchor = False
        self.current_bin = None

        _, dist = self.send_flight_command(CORRIDOR_X, self.last_y, BASE_Z, 0.0)
        time.sleep(max(3.0, dist * 1.0))
        _, dist = self.send_flight_command(CORRIDOR_X, rack_y, BASE_Z, 0.0)
        time.sleep(max(3.0, dist * 1.0))

        start = time.time()
        while time.time() - start < 15.0 and not self._shutting_down:
            if self.tag_detected:
                rospy.loginfo('[Phase 1] anchor locked; ready for Phase 2')
                return
            time.sleep(0.5)
        rospy.logwarn('[Phase 1] anchor tag not seen after 15 s; check the camera')

    def run_visual_servo(self, rack_dir, target_y, heading):
        self.servo_active = True
        self.servo_error_x = None
        self.servo_error_y = None
        time.sleep(1.0)
        for _ in range(5):
            if self._shutting_down:
                break
            if self.servo_error_x is None:
                rospy.logwarn('  [servo] bin tag (ids 0/1/2) not seen; skip centring')
                break
            if abs(self.servo_error_x) < 0.05 and abs(self.servo_error_y) < 0.05:
                rospy.loginfo('  [servo] centred')
                break
            new_x = self.last_x + (self.servo_error_x * rack_dir * 0.7)
            new_z = self.last_z - (self.servo_error_y * 0.7)
            # fine centring nudge -> straight-line goto (too short for the planner)
            self.send_flight_command(new_x, target_y, new_z, heading, direct=True)
            self.servo_error_x = None
            time.sleep(2.5)
        self.servo_active = False

    def execute_zigzag_sequence(self, target_bin, full_scan=False):
        """Phase 2/3: zig-zag through the bins (to target, or the whole rack)."""
        if not self.tag_detected:
            rospy.logwarn('[error] anchor tag not locked; press "Auto find rack" first')
            return
        self.lock_anchor = True

        if full_scan:
            rospy.loginfo('[Phase 2] full rack zig-zag scan (1 -> 18)')
            path = ZIGZAG_ORDER
        else:
            rospy.loginfo('[Phase 2] zig-zag to bin %d', target_bin)
            end_idx = ZIGZAG_ORDER.index(target_bin)
            if self.current_bin is None:
                path = ZIGZAG_ORDER[0:end_idx + 1]
            else:
                start_idx = ZIGZAG_ORDER.index(self.current_bin)
                if start_idx < end_idx:
                    path = ZIGZAG_ORDER[start_idx + 1:end_idx + 1]
                elif start_idx > end_idx:
                    path = ZIGZAG_ORDER[start_idx - 1:end_idx - 1:-1]
                else:
                    rospy.loginfo('already at bin %d', target_bin)
                    return

        rack_dir = RACKS_CONFIG[self.selected_rack]["dir"]
        target_y = self.anchor_y - (self.STANDOFF_DEPTH * rack_dir)
        heading = 1.57 if rack_dir == 1 else -1.57

        if abs(self.last_x - CORRIDOR_X) < 0.2:
            self.send_flight_command(CORRIDOR_X, target_y, BASE_Z, heading)
            time.sleep(4.0)

        for step_bin in path:
            if self._shutting_down:
                return
            rospy.loginfo('=> bin %d', step_bin)
            idx = step_bin - 1
            col = idx % 3
            row = idx // 3
            target_x = self.anchor_x + (1.4 + col * self.BAY_WIDTH)
            target_z = self.anchor_z - 0.4 + row * self.LEVEL_HEIGHT
            _, dist = self.send_flight_command(target_x, target_y, target_z, heading)
            time.sleep(max(3.0, dist * 1.5))
            # always auto-centre on the bin tag before the QR/label capture
            self.run_visual_servo(rack_dir, target_y, heading)
            self.current_bin = step_bin
            rospy.loginfo('[scan] bin %d centred -> QR/label capture point', step_bin)
            time.sleep(1.0)
        rospy.loginfo('[finished] zig-zag complete (bin %s)',
                      18 if full_scan else target_bin)

    # ------------------------------------------------------------- camera
    def _select_feed(self, idx):
        if not _HAVE_IMG:
            self._img_label.setText('cv_bridge unavailable')
            return
        suffix = CAMERA_FEEDS[idx][1]
        topic = '/%s/%s' % (self.uav_name, suffix)
        if self._img_sub is not None:
            self._img_sub.unregister()
        self._img_sub = rospy.Subscriber(
            topic, Image, self._img_cb, queue_size=1, buff_size=2 ** 24)
        self._img_label.setText('waiting for\n%s' % topic)

    def _img_cb(self, msg):
        if self._bridge is None:
            return
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            h, w = cv.shape[:2]
            qimg = QImage(cv.data, w, h, cv.strides[0], QImage.Format_RGB888).copy()
            self._image_ready.emit(qimg)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, 'image convert failed: %s', e)

    @Slot(QImage)
    def _on_image(self, qimg):
        pix = QPixmap.fromImage(qimg)
        self._img_label.setPixmap(pix.scaled(
            self._img_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    # ----------------------------------------------------------- shutdown
    def shutdown_plugin(self):
        self._shutting_down = True
        if getattr(self, '_timer', None) is not None:
            self._timer.stop()
        for sub in (self._img_sub, self._tag_sub, self._sub_diag, self._sub_hw):
            if sub is not None:
                try:
                    sub.unregister()
                except Exception:  # noqa: BLE001
                    pass
