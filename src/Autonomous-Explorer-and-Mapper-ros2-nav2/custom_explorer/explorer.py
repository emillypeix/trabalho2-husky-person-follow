import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from tf2_ros import Buffer, TransformListener

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/a200_0000/map', self.map_callback, 10)
        
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/a200_0000/navigate_to_pose')

        # Detection person
        self.person_found = False
        self.bbox = None

        # Visited frontiers set
        self.visited_frontiers = set()
        self.visited_goals = []
        self.goal_in_progress = False
        self.current_goal = None
        self.nav_goal_handle = None
        self.chosen_frontier = None


        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Timer for periodic exploration
        self.timer = self.create_timer(5.0, self.explore)

        self.bridge = CvBridge()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(DetectionArray, "/yolo/detections", self.detection_callback, 10)
        
        self.create_subscription(CameraInfo, "/a200_0000/sensors/camera_0/depth/camera_info", self.camera_info_callback, 10) 
        self.depth_sub = self.create_subscription( Image, "/a200_0000/sensors/camera_0/depth/image", self.depth_callback, 10 ) 
        self.depth_image = None 
        self.fx = self.fy = self.cx = self.cy = None 

        # Publisher de la posición 3D de la persona
        self.pub_person = self.create_publisher(PointStamped, "/person_position", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/person_side", 10)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def camera_info_callback(self, msg): 
        # Intrínsecos 
        self.fx = 390 
        self.fy = 390 
        self.cx = 320 
        self.cy = 240 
    
    def depth_callback(self, msg): 
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") 
    
    def detection_callback(self, msg: DetectionArray):

        # ---------- 1. Validaciones básicas ----------
        if self.depth_image is None or self.fx is None:
            return

        if not msg.detections:
            return

        # ---------- 2. Seleccionar mejor detección de persona ----------
        persons = [
            d for d in msg.detections
            if d.class_name == "person" and d.score > 0.5
        ]

        if not persons:
            return

        best_det = max(persons, key=lambda d: d.score)

        # ---------- 3. Detener exploración una sola vez ----------
        if not self.person_found:
            self.get_logger().info("👤 Persona detectada. Deteniendo exploración.")
            self.stop_robot()
            self.person_found = True

        # ---------- 4. Coordenadas píxel (pies de la persona) ----------
        px = int(best_det.bbox.center.position.x)
        py = int(
            best_det.bbox.center.position.y +
            best_det.bbox.size.y / 2.0
        )

        h, w = self.depth_image.shape
        if px < 3 or py < 3 or px >= w - 3 or py >= h - 3:
            return

        # ---------- 5. Profundidad robusta (7x7) ----------
        window = self.depth_image[py-3:py+4, px-3:px+4]
        valid = window[window > 0.0]

        if len(valid) < 5:
            return

        depth = float(np.percentile(valid, 30))

        # ---------- 6. Proyección 3D (frame óptico) ----------
        x_cam = (px - self.cx) * depth / self.fx
        y_cam = (py - self.cy) * depth / self.fy
        z_cam = depth

        # ---------- 7. Punto en frame de cámara ----------
        point_cam = PointStamped()
        point_cam.header.stamp = msg.header.stamp
        point_cam.header.frame_id = "camera_0_color_optical_frame"
        point_cam.point.x = x_cam
        point_cam.point.y = y_cam
        point_cam.point.z = z_cam

        # ---------- 8. Transformar a MAP ----------
        try:
            tf_cam_to_map = self.tf_buffer.lookup_transform(
                "map",
                point_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            point_map = do_transform_point(point_cam, tf_cam_to_map)

        except Exception as e:
            self.get_logger().warn(f"TF cam→map error: {e}")
            return

        # ---------- 9. Publicar posición real de la persona ----------
        self.pub_person.publish(point_map)

        Px = point_map.point.x
        Py = point_map.point.y

        self.get_logger().info(
            f"Persona MAP: x={Px:.2f}, y={Py:.2f}"
        )

        # ============================================================
        # === NUEVA LÓGICA CORRECTA: DESPLAZAMIENTO LATERAL PURO ===
        # ============================================================

        # ---------- 10. Transformar persona a base_link ----------
        try:
            tf_map_to_base = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            person_base = do_transform_point(point_map, tf_map_to_base)

        except Exception as e:
            self.get_logger().warn(f"TF map→base error: {e}")
            return

        # ---------- 11. Verificar cercanía ----------
        if person_base.point.x < 0.3:
            self.get_logger().info("🤝 Robot ya está junto a la persona")
            return

        # ---------- 12. Desplazamiento lateral en base_link ----------
        offset = 1.0  # metros al costado

        if person_base.point.y >= 0.0:
            side = "IZQUIERDA"
            goal_base_y = person_base.point.y + offset
        else:
            side = "DERECHA"
            goal_base_y = person_base.point.y - offset

        goal_base = PointStamped()
        goal_base.header.frame_id = "base_link"
        goal_base.header.stamp = msg.header.stamp
        goal_base.point.x = person_base.point.x
        goal_base.point.y = goal_base_y
        goal_base.point.z = 0.0

        # ---------- 13. Transformar goal a MAP ----------
        try:
            tf_base_to_map = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            goal_map = do_transform_point(goal_base, tf_base_to_map)

        except Exception as e:
            self.get_logger().warn(f"TF base→map error: {e}")
            return

        # ---------- 14. Publicar goal ----------
        goal_person = PoseStamped()
        goal_person.header.frame_id = "map"
        goal_person.header.stamp = self.get_clock().now().to_msg()
        goal_person.pose.position.x = goal_map.point.x
        goal_person.pose.position.y = goal_map.point.y
        goal_person.pose.orientation.w = 1.0

        self.pub_goal.publish(goal_person)

        self.get_logger().info(
            f"🧭 Navegando al lado {side}: "
            f"x={goal_map.point.x:.2f}, y={goal_map.point.y:.2f}"
        )

    
    def stop_robot(self):

        if self.goal_in_progress and self.nav_goal_handle is not None:
            self.get_logger().warn("🚨 PERSON DETECTED → CANCELING NAVIGATION 🚨")

            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _: self.get_logger().info("Navigation canceled")
            )

    def navigate_to(self, x, y):
        """
        Send navigation goal to Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")
        self.goal_in_progress = True
        self.current_goal = (x, y)
        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response and attach a callback to the result.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.goal_in_progress = False
            return
        
        self.nav_goal_handle = goal_handle

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Goal SUCCEEDED")
            self.visited_goals.append(self.current_goal)

        elif status == 3 or status == 5:  # ABORTED
            self.get_logger().warn("Goal ABORTED")
            self.visited_goals.append(self.current_goal)

        elif status == 6:  # CANCELED
            self.get_logger().warn("Goal CANCELED")
            self.visited_goals.append(self.current_goal)

        self.current_goal = None
        self.goal_in_progress = False
        self.chosen_frontier = None

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map.
        """
        frontiers = []
        rows, cols = map_array.shape

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier_a = None

        for cell in frontiers:

            wx, wy = self.map_to_world(cell)
            if self.is_visited(wx, wy):
                continue
            
            distance = np.sqrt((robot_row - wx)**2 + (robot_col - wy)**2)
            if distance < min_distance:
                min_distance = distance
                chosen_frontier_a = cell

            return chosen_frontier_a

        return None
    
    def map_to_world(self, chosen_frontier_a):

        r, c = chosen_frontier_a
        info = self.map_data.info

        x = c * info.resolution + info.origin.position.x
        y = r * info.resolution + info.origin.position.y

        return x, y

    def is_visited(self, x, y, threshold=3.0):

        for vx, vy in self.visited_goals:
            if math.hypot(x - vx, y - vy) < threshold:
                return True

        return False

    def explore(self):
        # Do nothing if navigating
        if self.goal_in_progress:
            return

        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            # self.shutdown_robot()
            return

        # Choose the closest frontier
        self.chosen_frontier = self.choose_frontier(frontiers)

        if not self.chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return
        
        if self.person_found:
            self.get_logger().info("Exploration stopped: person already found")
            return

        # Convert the chosen frontier to world coordinates
        goal_x, goal_y = self.map_to_world(self.chosen_frontier)


        # Navigate to the chosen frontier
        self.navigate_to(goal_x, goal_y)



    # def shudown_robot(self):
    #     
    #
    #
    #     self.get_logger().info("Shutting down robot exploration")


def main(args=None):
    remappings = [
            '--ros-args', 
            '-r', '/tf:=/a200_0000/tf', 
            '-r', '/tf_static:=/a200_0000/tf_static'
        ]
    
    # Combinamos con los args del sistema si existieran
    actual_args = remappings + (args if args else [])
    
    rclpy.init(args=actual_args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()