"""ROS subscriptions and fail-closed manual control. No device drivers here."""
import math
import threading
import time
from collections import deque


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class RosBridge:
    def __init__(self):
        self.ready = False
        self.error = '等待 ROS 连接'
        self.online = False
        self.nodes, self.publishers, self.topics = [], {}, []
        self.seen, self.messages = {}, {}
        self.logs, self.trail = deque(maxlen=250), deque(maxlen=1500)
        self.lock = threading.RLock()
        self.mode, self.estop = 'auto', True
        self.owner, self.deadline = None, 0
        self.image, self.image_at = None, 0
        self.map = None
        self.paths = {}
        self.pose, self.odom, self.scan = None, None, []
        self.closed = False
        self.emergency_active = False
        threading.Thread(target=self.connect_loop, daemon=True).start()

    def connect_loop(self):
        while not self.closed:
            try:
                import rosgraph
                master = rosgraph.Master('/smart_community_infra')
                state = master.getSystemState()
                self.publishers = dict(state[0])
                self.nodes = sorted({n for group in state for _, ns in group for n in ns})
                self.topics = master.getPublishedTopics('')
                self.online = True
                if not self.ready:
                    self.setup()
                self.error = ''
            except Exception as exc:
                self.online, self.error = False, str(exc)
                if self.ready and self.mode == 'manual':
                    self.halt()
            time.sleep(1)

    def setup(self):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from sensor_msgs.msg import Image, Imu, LaserScan
        from rosgraph_msgs.msg import Log
        from actionlib_msgs.msg import GoalID, GoalStatusArray
        from cv_bridge import CvBridge
        rospy.init_node('smart_community_infra', disable_signals=True)
        self.ros, self.cv = rospy, CvBridge()
        self.Twist, self.PoseStamped, self.Initial, self.GoalID = Twist, PoseStamped, PoseWithCovarianceStamped, GoalID
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.subs = []
        def subscribe(topic, cls, callback):
            def receive(msg):
                self.seen[topic] = time.monotonic()
                self.messages[topic] = msg
                callback(msg)
            self.subs.append(rospy.Subscriber(topic, cls, receive, queue_size=1))
        subscribe('/odom', Odometry, self.on_odom)
        subscribe('/odom_raw', Odometry, lambda m: None)
        subscribe('/wit/imu', Imu, lambda m: None)
        subscribe('/scan', LaserScan, lambda m: None)
        subscribe('/map', OccupancyGrid, self.on_map)
        subscribe('/rosout_agg', Log, lambda m: self.logs.append(dict(node=m.name, level=m.level, text=m.msg)))
        subscribe('/move_base/status', GoalStatusArray, lambda m: None)
        for topic in ['/move_base/GlobalPlanner/plan', '/move_base/local_plan']:
            subscribe(topic, Path, lambda m, t=topic: self.paths.update({t:m}))
        self.select_camera('/usb_cam/image_raw')
        self.ready = True
        threading.Thread(target=self.watchdog, daemon=True).start()

    def select_camera(self, topic):
        from sensor_msgs.msg import Image
        if hasattr(self, 'camera'):
            self.camera.unregister()
        self.image, self.image_at = None, 0
        def receive(msg):
            self.image, self.image_at = msg, time.monotonic()
        self.camera = self.ros.Subscriber(topic, Image, receive, queue_size=1, buff_size=16*1024*1024)
        self.camera_topic = topic

    def on_odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        self.odom = dict(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation),
                         vx=t.linear.x, vy=t.linear.y, wz=t.angular.z)
        self.trail.append([p.position.x, p.position.y])

    def on_map(self, m):
        self.map = dict(width=m.info.width, height=m.info.height, resolution=m.info.resolution,
                        origin=[m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)],
                        data=list(m.data), frame=m.header.frame_id)

    def conflicts(self):
        return [n for n in self.publishers.get('/cmd_vel', []) if n != '/smart_community_infra']

    def halt(self):
        with self.lock:
            self.deadline, self.owner = 0, None
            if self.ready:
                self.cmd.publish(self.Twist())

    def emergency(self):
        with self.lock:
            self.estop = True
            self.emergency_active = True
            self.halt()
            if self.ready:
                self.cancel.publish(self.GoalID())

    def manual(self, owner, x, y, z):
        with self.lock:
            if not self.ready or not self.online or self.estop or self.mode != 'manual' or self.conflicts():
                self.halt()
                raise ValueError('控制锁定、离线或存在其他速度发布者；请先停止导航及其他控制器')
            if self.owner not in (None, owner):
                raise ValueError('手动控制已被其他页面占用')
            self.owner, self.deadline = owner, time.monotonic()+0.35
            m = self.Twist()
            m.linear.x, m.linear.y, m.angular.z = x, y, z
            self.cmd.publish(m)

    def watchdog(self):
        while not self.closed:
            with self.lock:
                if self.emergency_active:
                    self.cmd.publish(self.Twist())
                    self.cancel.publish(self.GoalID())
                if self.mode == 'manual' and (time.monotonic() > self.deadline or self.conflicts() or not self.online):
                    self.halt()
            time.sleep(0.05)

    def frame(self):
        if self.image is None or time.monotonic()-self.image_at > 2:
            raise ValueError('相机离线或图像已过期')
        return self.cv.imgmsg_to_cv2(self.image, 'bgr8')

    def transform_points(self, frame, points):
        if frame == 'map':
            return points
        t = self.tf.lookup_transform('map', frame, self.ros.Time(0), self.ros.Duration(0.05)).transform
        a = yaw(t.rotation)
        return [[t.translation.x+math.cos(a)*x-math.sin(a)*y,
                 t.translation.y+math.sin(a)*x+math.cos(a)*y] for x,y in points]

    def snapshot(self):
        now = time.monotonic()
        checks = {t: self.online and now-v < 3 for t,v in self.seen.items()}
        paths, scan, pose = {}, [], None
        if self.ready and self.online:
            try:
                t = self.tf.lookup_transform('map','base_footprint',self.ros.Time(0),self.ros.Duration(0.05)).transform
                pose = [t.translation.x,t.translation.y,yaw(t.rotation)]
                checks['TF map→base_footprint'] = True
            except Exception:
                checks['TF map→base_footprint'] = False
            try:
                m = self.messages.get('/scan')
                if m and checks.get('/scan'):
                    scan = self.transform_points(m.header.frame_id, [[r*math.cos(m.angle_min+i*m.angle_increment),r*math.sin(m.angle_min+i*m.angle_increment)] for i,r in enumerate(m.ranges) if i%3==0 and math.isfinite(r) and m.range_min <= r <= m.range_max])
                for k,m in list(self.paths.items()):
                    paths[k] = self.transform_points(m.header.frame_id, [[p.pose.position.x,p.pose.position.y] for p in m.poses])
            except Exception:
                pass
        return dict(online=self.online,error=self.error,nodes=self.nodes if self.online else [],
                    topics=self.topics if self.online else [],checks=checks,mode=self.mode,estop=self.estop,
                    conflicts=self.conflicts(),odom=self.odom if checks.get('/odom') else None,
                    trail=list(self.trail),pose=pose,scan=scan,paths=paths,logs=list(self.logs),
                    camera_online=self.online and now-self.image_at<2)

    def navigate(self, kind, x,y,a):
        if not self.online or self.estop or self.mode != 'auto':
            raise ValueError('请联网并切换自动模式、解除软件急停')
        m = self.Initial() if kind == 'initial' else self.PoseStamped()
        m.header.frame_id, m.header.stamp = 'map', self.ros.Time.now()
        p = m.pose.pose if kind == 'initial' else m.pose
        p.position.x,p.position.y = x,y
        p.orientation.z,p.orientation.w = math.sin(a/2),math.cos(a/2)
        if kind == 'initial':
            m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35] = .25,.25,.0685
            self.initial.publish(m)
        else:
            self.goal.publish(m)
