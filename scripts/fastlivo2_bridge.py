#!/usr/bin/env python3
"""
FAST-LIVO2 → Gaussian-LIC Bridge Node

Converts FAST-LIVO2 outputs to the topics expected by Gaussian-LIC backend:
  /aft_mapped_to_init  (nav_msgs/Odometry,  IMU pose in world)
  /rgb_img             (sensor_msgs/Image,  undistorted camera image from FAST-LIVO2)
  /cloud_registered    (sensor_msgs/PointCloud2, XYZ-only point cloud in world frame)

→

  /pose_for_gs         (geometry_msgs/PoseStamped, camera pose in world)
  /image_for_gs        (sensor_msgs/Image)
  /points_for_gs       (sensor_msgs/PointCloud2, XYZRGB – colors projected from image)

Key transformation: IMU pose → Camera pose using LiDAR-IMU and Camera-LiDAR extrinsics.

Point cloud coloring: Each 3D world-frame point is projected onto the current camera
image to sample its color.  Only points visible from the camera (positive depth and
within image bounds) are forwarded – this also satisfies the backend's assert(depth>0)
requirement in gaussian.cpp.

Extrinsic convention (same as FAST-LIVO2 config):
  extrin_R  (9 floats, row-major): R_il  – rotation FROM LiDAR TO IMU body frame
  extrin_T  (3 floats):            P_il  – translation of LiDAR origin in IMU frame
  R_cl      (9 floats, row-major): R_cl  – rotation FROM LiDAR TO Camera frame
  P_cl      (3 floats):            P_cl  – translation part of LiDAR-in-Camera extrinsic

Derived quantities:
  R_li = R_il^T,        P_li = -R_il^T * P_il    (IMU → LiDAR)
  R_ci = R_cl * R_li,   P_ci = R_cl * P_li + P_cl (IMU → Camera)
  R_ic = R_ci^T,        t_ic = -R_ci^T * P_ci     (Camera in IMU frame)

Camera world pose:
  R_wc = R_wi * R_ic
  t_wc = P_wi + R_wi * t_ic
"""

import rospy
import numpy as np
from threading import Lock

import cv2
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Quaternion, Point


def quat_from_mat(R):
    """Return (x, y, z, w) quaternion from a 3×3 rotation matrix."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def quat_to_mat(q):
    """Convert geometry_msgs/Quaternion to 3×3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z),  2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


class FastLIVO2Bridge:
    def __init__(self):
        rospy.init_node('fastlivo2_bridge', anonymous=True)

        # ── Extrinsic parameters ─────────────────────────────────────────────
        # Read as individual scalar params to avoid ROS array-param parsing issues.
        # extrin_R_i : R_il (LiDAR → IMU), row-major, indices 0..8
        # extrin_T_i : P_il (LiDAR origin in IMU frame), indices 0..2
        # R_cl_i     : rotation FROM LiDAR TO Camera, row-major, indices 0..8
        # P_cl_i     : LiDAR-in-Camera translation, indices 0..2

        def _read_mat3(prefix, default_identity=True):
            I = [1, 0, 0, 0, 1, 0, 0, 0, 1] if default_identity else [0]*9
            return [rospy.get_param(f'~{prefix}_{i}', float(I[i])) for i in range(9)]

        def _read_vec3(prefix):
            return [rospy.get_param(f'~{prefix}_{i}', 0.0) for i in range(3)]

        R_il = np.array(_read_mat3('extrin_R', default_identity=True),
                        dtype=np.float64).reshape(3, 3)
        P_il = np.array(_read_vec3('extrin_T'), dtype=np.float64)
        R_cl = np.array(_read_mat3('R_cl', default_identity=False),
                        dtype=np.float64).reshape(3, 3)
        P_cl = np.array(_read_vec3('P_cl'), dtype=np.float64)

        # IMU → LiDAR transform
        R_li = R_il.T
        P_li = -R_li @ P_il

        # IMU → Camera transform
        self.R_ci = R_cl @ R_li
        self.P_ci = R_cl @ P_li + P_cl

        # Camera-in-IMU: T_ic
        R_ic = self.R_ci.T
        t_ic = -R_ic @ self.P_ci
        self._R_ic = R_ic
        self._t_ic = t_ic

        rospy.loginfo(
            '[Bridge] R_ci =\n%s\nP_ci = %s\nt_ic = %s',
            self.R_ci, self.P_ci, self._t_ic
        )

        # ── Camera intrinsics (for point-cloud coloring projection) ──────────
        self._fx    = rospy.get_param('~fx',    431.795259219)
        self._fy    = rospy.get_param('~fy',    431.550090267)
        self._cx    = rospy.get_param('~cx',    310.833037316)
        self._cy    = rospy.get_param('~cy',    266.985989326)
        self._img_w = int(rospy.get_param('~img_w', 640))
        self._img_h = int(rospy.get_param('~img_h', 512))

        # ── Lens distortion coefficients (OpenCV model: k1,k2,p1,p2,k3) ────
        # camera_pinhole.yaml: cam_d0=k1, cam_d1=k2, cam_d2=p1, others=0
        d0 = rospy.get_param('~d0', 0.0)
        d1 = rospy.get_param('~d1', 0.0)
        d2 = rospy.get_param('~d2', 0.0)
        d3 = rospy.get_param('~d3', 0.0)
        d4 = rospy.get_param('~d4', 0.0)
        self._dist_coeffs = np.array([d0, d1, d2, d3, d4], dtype=np.float64)
        self._K = np.array([[self._fx, 0.0, self._cx],
                            [0.0, self._fy, self._cy],
                            [0.0, 0.0, 1.0]], dtype=np.float64)
        # Precompute undistortion remap (efficient per-frame use)
        self._undist_map1, self._undist_map2 = cv2.initUndistortRectifyMap(
            self._K, self._dist_coeffs, None, self._K,
            (self._img_w, self._img_h), cv2.CV_16SC2)
        self._apply_undistort = abs(d0) > 1e-7  # skip remap if no distortion

        self._cv_bridge = CvBridge()

        # ── Frame subsampling (publish every N-th LiDAR frame) ───────────────
        # subsample_factor=4 means publish 1 in 4 frames ⇒ ~505 from 2022
        self._subsample_factor = int(rospy.get_param('~subsample_factor', 1))
        self._frame_count      = 0

        # ── Sharpness filter (skip blurry frames) ────────────────────────────
        # Laplacian variance < threshold → skip frame (0 = disabled)
        self._sharpness_thresh = float(rospy.get_param('~sharpness_thresh', 0.0))

        # ── Point-cloud size cap (random downsample for faster processing) ───
        # 0 = disabled (keep all points)
        self._max_cloud_pts = int(rospy.get_param('~max_cloud_pts', 0))

        # ── Pre-filter: keep only the N nearest points before projection ──────
        # FAST-LIVO2's accumulated cloud grows over time (potentially >1M pts).
        # Projecting all points onto the camera is O(N) and takes >10s for large
        # clouds → exceeds gs_mapping's exit timeout → late frames never received.
        # We keep the closest pre_filter_pts points (most relevant to current
        # view) before doing any projection.  0 = disabled.
        self._pre_filter_pts = int(rospy.get_param('~pre_filter_pts', 60000))

        # ── Brightness normalisation (eliminates auto-exposure variation) ────
        # target_brightness > 0: scale image so mean pixel value matches target
        # 0 = disabled.  90.0 ≈ reference brightness for HKU campus sequence.
        self._target_brightness = float(rospy.get_param('~target_brightness', 0.0))

        # ── Topic names (can be overridden via params) ───────────────────────
        odom_topic   = rospy.get_param('~odom_topic',   '/aft_mapped_to_init')
        image_topic  = rospy.get_param('~image_topic',  '/rgb_img')
        points_topic = rospy.get_param('~points_topic', '/cloud_registered')

        # ── Shared state (latest received messages) ──────────────────────────
        self._lock        = Lock()
        self._latest_pose   = None   # nav_msgs/Odometry
        self._latest_image  = None   # sensor_msgs/Image

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_pose   = rospy.Publisher('/pose_for_gs',   PoseStamped,  queue_size=10000)
        self._pub_image  = rospy.Publisher('/image_for_gs',  Image,        queue_size=10000)
        self._pub_points = rospy.Publisher('/points_for_gs', PointCloud2,  queue_size=10000)

        # ── Subscribers ──────────────────────────────────────────────────────
        # High queue sizes so we don't drop data during burst processing
        rospy.Subscriber(odom_topic,   Odometry,     self._odom_cb,   queue_size=10000)
        rospy.Subscriber(image_topic,  Image,        self._image_cb,  queue_size=10000)
        rospy.Subscriber(points_topic, PointCloud2,  self._points_cb, queue_size=10000)

        rospy.loginfo('[Bridge] Listening: odom=%s  image=%s  points=%s',
                      odom_topic, image_topic, points_topic)
        rospy.loginfo('[Bridge] Publishing: /pose_for_gs  /image_for_gs  /points_for_gs')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._latest_pose = msg

    def _image_cb(self, msg: Image):
        with self._lock:
            self._latest_image = msg

    def _points_cb(self, points_msg: PointCloud2):
        """Triggered when FAST-LIVO2 publishes a new point cloud.

        Projects all world-frame 3D points onto the current camera image to
        assign RGB colors.  Only points visible from the camera (positive depth
        and within image bounds) are forwarded.  This satisfies the backend's
        assert(depth>0) in gaussian.cpp and provides correct initial colors for
        Gaussian SH initialization.
        """
        # ── Frame subsampling: skip all-but-every-N frames before any work ──
        self._frame_count += 1
        if self._subsample_factor > 1 and self._frame_count % self._subsample_factor != 0:
            return

        with self._lock:
            pose_msg  = self._latest_pose
            image_msg = self._latest_image

        if pose_msg is None or image_msg is None:
            return

        # ── Convert IMU pose → Camera pose ───────────────────────────────────
        q_wi = pose_msg.pose.pose.orientation
        P_wi = np.array([
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            pose_msg.pose.pose.position.z,
        ])

        R_wi = quat_to_mat(q_wi)
        R_wc = R_wi @ self._R_ic
        t_wc = P_wi + R_wi @ self._t_ic

        qx, qy, qz, qw = quat_from_mat(R_wc)

        # Use the point cloud header stamp as the synchronised timestamp
        stamp = points_msg.header.stamp

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp    = stamp
        pose_stamped.header.frame_id = 'camera_init'
        pose_stamped.pose.position   = Point(t_wc[0], t_wc[1], t_wc[2])
        pose_stamped.pose.orientation = Quaternion(qx, qy, qz, qw)

        # Re-stamp image with the same timestamp so the backend sees alignment
        image_out = Image()
        image_out.header          = image_msg.header
        image_out.header.stamp    = stamp
        image_out.height          = image_msg.height
        image_out.width           = image_msg.width
        image_out.encoding        = image_msg.encoding
        image_out.is_bigendian    = image_msg.is_bigendian
        image_out.step            = image_msg.step
        image_out.data            = image_msg.data

        # ── Decode and undistort image ────────────────────────────────────────
        try:
            image_bgr = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, '[Bridge] cv_bridge decode failed: %s – forwarding uncolored cloud', e)
            self._pub_pose.publish(pose_stamped)
            self._pub_image.publish(image_out)
            self._pub_points.publish(points_msg)
            return

        # Apply lens undistortion so that gs_mapping GT images align with the
        # pinhole model (fx/fy/cx/cy without distortion) used for rendering.
        if self._apply_undistort:
            image_bgr = cv2.remap(image_bgr, self._undist_map1, self._undist_map2,
                                  cv2.INTER_LINEAR)

        # ── Sharpness filter: skip motion-blurred frames ─────────────────────
        if self._sharpness_thresh > 0:
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            if lap_var < self._sharpness_thresh:
                rospy.loginfo_throttle(5.0,
                    '[Bridge] Blurry frame (lap=%.1f < %.1f) – skipping',
                    lap_var, self._sharpness_thresh)
                return

        # ── Brightness normalisation: eliminate auto-exposure variation ──────
        # Scale image so mean pixel value matches target (e.g. 90 out of 255).
        # This makes all frames consistent regardless of scene/exposure changes.
        if self._target_brightness > 0:
            actual_mean = float(image_bgr.astype(np.float32).mean())
            if actual_mean > 5.0:
                scale = self._target_brightness / actual_mean
                image_bgr = np.clip(
                    image_bgr.astype(np.float32) * scale, 0, 255
                ).astype(np.uint8)

        xyz_world = self._parse_xyz_from_cloud(points_msg)
        if xyz_world is None or len(xyz_world) == 0:
            rospy.logwarn_throttle(5.0, '[Bridge] Empty or unparseable point cloud – skipping frame')
            return

        # NOTE: max_cloud_pts downsampling is done INSIDE _color_and_pack_cloud,
        # AFTER visibility filtering.  This ensures we always forward up to
        # max_cloud_pts VISIBLE points regardless of how large the accumulated
        # cloud grows; early random downsampling would starve late frames.
        colored_cloud = self._color_and_pack_cloud(xyz_world, R_wc, t_wc, image_bgr, points_msg.header)
        if colored_cloud is None:
            rospy.logwarn_throttle(5.0, '[Bridge] No visible points this frame – skipping')
            return

        # Rebuild image_out from the (possibly undistorted) image_bgr so that
        # gs_mapping stores the undistorted frame as its ground-truth image.
        try:
            image_out = self._cv_bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            image_out.header.stamp    = stamp
            image_out.header.frame_id = image_msg.header.frame_id
        except CvBridgeError:
            pass  # fallback: keep original image_out

        self._pub_pose.publish(pose_stamped)
        self._pub_image.publish(image_out)
        self._pub_points.publish(colored_cloud)

    # ── Point-cloud coloring helpers ──────────────────────────────────────────

    def _parse_xyz_from_cloud(self, cloud_msg: PointCloud2):
        """Extract XYZ as (N, 3) float32 ndarray from PointCloud2. Returns None on failure."""
        fields = {f.name: f for f in cloud_msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            rospy.logwarn_throttle(10.0, '[Bridge] PointCloud2 missing x/y/z fields')
            return None
        ox = fields['x'].offset
        oy = fields['y'].offset
        oz = fields['z'].offset
        ps = cloud_msg.point_step
        n  = cloud_msg.width * cloud_msg.height
        if n == 0:
            return None
        raw = np.frombuffer(bytes(cloud_msg.data), dtype=np.uint8).reshape(n, ps)
        x = np.frombuffer(raw[:, ox:ox+4].copy().tobytes(), dtype=np.float32)
        y = np.frombuffer(raw[:, oy:oy+4].copy().tobytes(), dtype=np.float32)
        z = np.frombuffer(raw[:, oz:oz+4].copy().tobytes(), dtype=np.float32)
        xyz = np.stack([x, y, z], axis=1)
        return xyz[np.all(np.isfinite(xyz), axis=1)]

    def _color_and_pack_cloud(self, xyz_world, R_wc, t_wc, image_bgr, header):
        """Project world-frame XYZ onto camera, sample BGR colors, return XYZRGB PointCloud2.

        Only forwards points that are visible from the camera (positive depth and
        within image bounds).  This guarantees depth > 0 for all output points,
        satisfying the backend assert in gaussian.cpp::Dataset::addFrame().
        """
        H, W  = image_bgr.shape[:2]
        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        # ── Distance-based pre-filter ───────────────────────────────────────────
        # FAST-LIVO2 accumulates a growing point cloud.  By frame 200+, the
        # cloud can have >500k points; projecting all of them takes >10s and
        # trips the gs_mapping exit timeout.  Keep only the pre_filter_pts
        # nearest points to the camera centre before doing any projection.
        # Nearest points are most likely to be visible and most useful for
        # Gaussian initialisation, so quality loss is minimal.
        if self._pre_filter_pts > 0 and len(xyz_world) > self._pre_filter_pts:
            sq_dists = np.sum((xyz_world - t_wc) ** 2, axis=1)
            keep_idx = np.argpartition(sq_dists, self._pre_filter_pts)[:self._pre_filter_pts]
            xyz_world = xyz_world[keep_idx]

        # world → camera: p_c = R_wc^T @ (p_w - t_wc)
        R_cw  = R_wc.T
        diff  = xyz_world - t_wc[np.newaxis, :]   # (N, 3)
        p_c   = (R_cw @ diff.T).T                  # (N, 3)

        # Front-of-camera filter (small positive margin)
        mask_z = p_c[:, 2] > 0.05
        p_c_vis   = p_c[mask_z]
        xyz_vis   = xyz_world[mask_z]
        if len(xyz_vis) == 0:
            return None

        # Project onto image plane
        inv_z = 1.0 / p_c_vis[:, 2]
        u = fx * p_c_vis[:, 0] * inv_z + cx
        v = fy * p_c_vis[:, 1] * inv_z + cy

        # Image-bounds filter (1-pixel margin so clip below stays safe)
        mask_uv = (u >= 0) & (u <= W - 1) & (v >= 0) & (v <= H - 1)
        xyz_vis = xyz_vis[mask_uv]
        u_vis   = u[mask_uv]
        v_vis   = v[mask_uv]
        if len(xyz_vis) == 0:
            return None

        # ── Post-visibility downsampling: cap to max_cloud_pts VISIBLE points ─
        # Done HERE (after visibility filter) so late frames are never starved:
        # early frames have small clouds (all pass), late frames have huge
        # accumulated clouds but we still forward max_cloud_pts visible points.
        if self._max_cloud_pts > 0 and len(xyz_vis) > self._max_cloud_pts:
            keep = np.random.choice(len(xyz_vis), self._max_cloud_pts, replace=False)
            xyz_vis = xyz_vis[keep]
            u_vis   = u_vis[keep]
            v_vis   = v_vis[keep]

        # Bilinear color sampling
        u0 = np.floor(u_vis).astype(np.int32).clip(0, W - 2)
        v0 = np.floor(v_vis).astype(np.int32).clip(0, H - 2)
        u1 = u0 + 1
        v1 = v0 + 1
        wu = (u_vis - u0.astype(np.float32))[:, np.newaxis]  # (N,1)
        wv = (v_vis - v0.astype(np.float32))[:, np.newaxis]
        colors = (
            image_bgr[v0, u0].astype(np.float32) * (1 - wv) * (1 - wu) +
            image_bgr[v0, u1].astype(np.float32) * (1 - wv) *      wu  +
            image_bgr[v1, u0].astype(np.float32) *      wv  * (1 - wu) +
            image_bgr[v1, u1].astype(np.float32) *      wv  *      wu
        ).astype(np.uint8)  # (N, 3) BGR

        # Pack XYZRGB PointCloud2:
        #   Layout: x(f32) y(f32) z(f32) rgb(f32-packed) → 16 bytes/point
        #   rgb packing: 0x00RRGGBB stored as IEEE-754 float bits (PCL convention)
        N = len(xyz_vis)
        r = colors[:, 2].astype(np.uint32)  # BGR → R
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 0].astype(np.uint32)  # BGR → B
        rgb_u32   = (r << 16) | (g << 8) | b
        rgb_f32   = rgb_u32.view(np.float32)

        data = np.empty((N, 4), dtype=np.float32)
        data[:, 0] = xyz_vis[:, 0]
        data[:, 1] = xyz_vis[:, 1]
        data[:, 2] = xyz_vis[:, 2]
        data[:, 3] = rgb_f32

        cloud_out = PointCloud2()
        cloud_out.header       = header
        cloud_out.height       = 1
        cloud_out.width        = N
        cloud_out.fields       = [
            PointField('x',   0, PointField.FLOAT32, 1),
            PointField('y',   4, PointField.FLOAT32, 1),
            PointField('z',   8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]
        cloud_out.is_bigendian = False
        cloud_out.point_step   = 16
        cloud_out.row_step     = N * 16
        cloud_out.data         = data.tobytes()
        cloud_out.is_dense     = True
        return cloud_out


def main():
    bridge = FastLIVO2Bridge()
    rospy.spin()


if __name__ == '__main__':
    main()
