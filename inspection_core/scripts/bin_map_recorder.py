#!/usr/bin/env python3
"""bin_map_recorder — build the semantic BIN map during the survey / mapping pass.

Snapshots the drone's CURRENT world pose as a BIN's standoff pose and emits
warehouse_bins.yaml (+ a tag_id -> world_pose anchor table). See
docs/bin_map_building.md for the method.

Modes
-----
* Manual (default, most robust): teleop the drone to each BIN's viewing pose, then
      rostopic pub -1 /bin_map_recorder/record std_msgs/String "data: 'A2'"
  and when finished
      rostopic pub -1 /bin_map_recorder/save   std_msgs/Empty "{}"
* QR auto (optional, _qr_enabled:=true): when a beam QR decodes on the camera image,
  auto-records a BIN keyed by the decoded value using the current pose (approximate).

AprilTag anchors are collected automatically if apriltag_ros publishes the tag topic;
each tag pose is transformed into the world frame and stored.

Params (~private):
  world_frame   (str,  "world_origin")  fixed warehouse frame the map is expressed in
  base_frame    (str,  "uav1/fcu")      drone body frame
  standoff_distance (float, 0.8)        bin_pose is projected this far ahead of the drone
  output_file   (str,  required)        where warehouse_bins.yaml is written
  qr_enabled    (bool, false)           enable QR auto-record
  image_topic   (str,  "")              camera image topic for QR mode
  tag_topic     (str,  "/tag_detections")  apriltag_ros detections topic
"""
import math
import re
import rospy
import yaml

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)
from std_msgs.msg import String, Empty
from tf.transformations import euler_from_quaternion

_ID_RE = re.compile(r"^[A-Za-z]*?(?P<col>\d+)?[Ll](?P<lvl>\d+)$")  # e.g. C2L4
_AC_RE = re.compile(r"^(?P<col>[A-Za-z]+)(?P<lvl>\d+)$")            # e.g. A2


class BinMapRecorder(object):
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world_origin")
        self.base_frame = rospy.get_param("~base_frame", "uav1/fcu")
        self.standoff = float(rospy.get_param("~standoff_distance", 0.8))
        self.output_file = rospy.get_param("~output_file", "/tmp/warehouse_bins.yaml")
        self.qr_enabled = bool(rospy.get_param("~qr_enabled", False))
        self.tag_topic = rospy.get_param("~tag_topic", "/tag_detections")

        self.bins = {}      # id -> record dict
        self.anchors = {}   # tag_id (int) -> (x, y, z)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("~record", String, self._on_record, queue_size=10)
        rospy.Subscriber("~save", Empty, self._on_save, queue_size=1)

        self._init_apriltag()
        if self.qr_enabled:
            self._init_qr()

        rospy.loginfo("bin_map_recorder up: world=%s base=%s -> %s (qr=%s)",
                      self.world_frame, self.base_frame, self.output_file, self.qr_enabled)

    # ----- current drone pose in the world frame -----------------------------
    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame,
                                                rospy.Time(0), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF %s<-%s unavailable: %s", self.world_frame, self.base_frame, e)
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (tr.x, tr.y, tr.z, yaw)

    def nearest_anchor(self, x, y, z):
        if not self.anchors:
            return None
        return min(self.anchors,
                   key=lambda k: (self.anchors[k][0] - x) ** 2
                   + (self.anchors[k][1] - y) ** 2 + (self.anchors[k][2] - z) ** 2)

    # ----- recording ---------------------------------------------------------
    def record_bin(self, bin_id):
        bin_id = bin_id.strip()
        if not bin_id:
            return
        pose = self.current_pose()
        if pose is None:
            rospy.logwarn("skip '%s': no pose", bin_id)
            return
        x, y, z, yaw = pose
        bx = x + self.standoff * math.cos(yaw)
        by = y + self.standoff * math.sin(yaw)
        rec = {
            "id": bin_id,
            "tag_id": self.nearest_anchor(bx, by, z),
            "bin_pose": {"x": round(bx, 3), "y": round(by, 3), "z": round(z, 3)},
            "standoff_pose": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                              "yaw": round(yaw, 4)},
            "sap_slot": self._sap_slot(bin_id),
        }
        col, lvl = self._parse_addr(bin_id)
        if col is not None:
            rec["column"] = col
        if lvl is not None:
            rec["level"] = lvl
        new = bin_id not in self.bins
        self.bins[bin_id] = rec
        rospy.loginfo("%s BIN %s @ standoff (%.2f, %.2f, %.2f, %.1f deg)  [%d total]",
                      "recorded" if new else "updated", bin_id, x, y, z,
                      math.degrees(yaw), len(self.bins))

    @staticmethod
    def _parse_addr(bin_id):
        m = _ID_RE.match(bin_id)
        if m and (m.group("col") or m.group("lvl")):
            return (int(m.group("col")) if m.group("col") else None,
                    int(m.group("lvl")) if m.group("lvl") else None)
        m = _AC_RE.match(bin_id)
        if m:
            return None, int(m.group("lvl"))
        return None, None

    @staticmethod
    def _sap_slot(bin_id):
        return "RACK-%s" % bin_id  # placeholder; software team maps to the real SAP bin

    def _on_record(self, msg):
        self.record_bin(msg.data)

    def _on_save(self, _msg):
        self.save()

    def save(self):
        data = {
            "bins": list(self.bins.values()),
            "anchors": {int(k): {"x": round(v[0], 3), "y": round(v[1], 3), "z": round(v[2], 3)}
                        for k, v in self.anchors.items()},
        }
        with open(self.output_file, "w") as f:
            f.write("# Generated by bin_map_recorder during the mapping pass.\n")
            f.write("# Poses are in the '%s' world frame [m], yaw [rad].\n" % self.world_frame)
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("wrote %d BINs + %d anchors -> %s",
                      len(self.bins), len(self.anchors), self.output_file)

    # ----- optional: AprilTag anchors ---------------------------------------
    def _init_apriltag(self):
        try:
            from apriltag_ros.msg import AprilTagDetectionArray
        except ImportError:
            rospy.loginfo("apriltag_ros not available -> anchors disabled (manual map still works)")
            return
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tags, queue_size=10)
        rospy.loginfo("collecting AprilTag anchors from %s", self.tag_topic)

    def _on_tags(self, msg):
        from geometry_msgs.msg import PoseStamped
        for det in msg.detections:
            if not det.id:
                continue
            tag_id = int(det.id[0])
            ps = PoseStamped()
            ps.header = det.pose.header
            ps.pose = det.pose.pose.pose
            try:
                w = self.tf_buffer.transform(ps, self.world_frame, rospy.Duration(0.3))
            except Exception as e:  # noqa: BLE001 (TF not ready / extrapolation)
                rospy.logwarn_throttle(5.0, "tag %d transform failed: %s", tag_id, e)
                continue
            p = w.pose.position
            self.anchors[tag_id] = (p.x, p.y, p.z)

    # ----- optional: QR auto-record -----------------------------------------
    def _init_qr(self):
        try:
            import cv2
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except ImportError:
            rospy.logwarn("cv2/cv_bridge unavailable -> QR auto-record disabled")
            return
        self._cv2 = cv2
        self._bridge = CvBridge()
        self._qr = cv2.QRCodeDetector()
        topic = rospy.get_param("~image_topic", "")
        if not topic:
            rospy.logwarn("qr_enabled but ~image_topic empty -> QR disabled")
            return
        rospy.Subscriber(topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("QR auto-record on %s", topic)

    def _on_image(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            data, _pts, _ = self._qr.detectAndDecode(img)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "QR decode failed: %s", e)
            return
        if data and data not in self.bins:
            rospy.loginfo("QR decoded '%s' -> recording", data)
            self.record_bin(data)


if __name__ == "__main__":
    rospy.init_node("bin_map_recorder")
    BinMapRecorder()
    rospy.spin()
