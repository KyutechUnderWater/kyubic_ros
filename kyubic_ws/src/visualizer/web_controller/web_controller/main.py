#!/usr/bin/env python3
import os
from dataclasses import dataclass
from pathlib import Path
import rclpy
from rclpy.node import Node
import viser
import viser.uplot
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from threading import Thread, Lock

# --- ROS2 Message Imports ---
from rt_pose_plotter_msgs.msg import Targets
from localization_msgs.msg import Odometry
from sensor_msgs.msg import Image

# --- CV Bridge for Camera Images ---
try:
    from cv_bridge import CvBridge
except ImportError:
    print(
        "Error: cv_bridge is not installed. Please run 'sudo apt-get install ros-<distro>-cv-bridge'"
    )
    exit()


@dataclass
class Colors:
    orange: tuple[int] = (255, 165, 0)
    yellow: tuple[int] = (255, 255, 0)
    aqua: tuple[int] = (0, 255, 255)
    deep_sky: tuple[int] = (0, 191, 255)
    royal_blue: tuple[int] = (65, 105, 225)
    blue: tuple[int] = (0, 0, 255)


class WebVisualizerNode(Node):
    """
    A ROS2 node that creates a web-based visualizer for a robot using Viser.
    """

    def __init__(
        self, server: viser.ViserServer, mesh_path: Path, pub_targets_period: float
    ):
        super().__init__("web_visualizer_node")
        self.server = server
        self.mesh_path = mesh_path
        self.pub_targets_period = pub_targets_period
        self.lock = Lock()
        self.bridge = CvBridge()

        # --- State Variables ---
        self.trajectory_points = np.array(((0, 0, 0), (0, 0, 0))).reshape(1, 2, 3)
        self.trajectory_colors = np.array((Colors.orange, Colors.orange)).reshape(
            1, 2, 3
        )

        self.targets = Targets()

        self.is_publishing_targets = False
        self.publish_timer = None
        self.MAX_DATAPOINTS = 300

        # Data storage for plots
        self.odom_data = self._init_plot_data()
        self.target_buffer = self._init_plot_data()
        self.targets_data = self._init_plot_data()

        server.gui.configure_theme(
            brand_color=(230, 180, 30),
        )

        # --- 1. Robot Mesh and Camera View Setup ---
        self._setup_3d_scene()

        # --- 2. GUI Setup ---
        self._setup_gui()

        # --- 3. ROS2 Publishers and Subscribers ---
        self._setup_ros_communication()

        self.get_logger().info(
            "Web Visualizer Node started. Open the Viser URL in your browser."
        )

    def _setup_3d_scene(self):
        """Initializes the 3D scene in Viser, including the robot mesh and camera views."""
        # Path to 3D mesh file（.obj, .stl)
        try:
            mesh = trimesh.load_mesh(self.mesh_path)
            self.get_logger().info(f"Successfully loaded mesh from {self.mesh_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to load mesh from '{self.mesh_path}': {e}")
            self.get_logger().warn("Using a fallback box primitive.")
            mesh = trimesh.creation.box(extents=[0.8, 0.5, 0.3])  # sample

        vertices = mesh.vertices
        faces = mesh.faces
        print(f"Loaded mesh with {vertices.shape} vertices, {faces.shape} faces")

        # Add mesh of robot
        self.robot = self.server.scene.add_mesh_trimesh(
            name="/robot", mesh=mesh, position=(0.0, 0.0, 0.0)
        )

        self.server.scene.add_frame(
            "/robot/origin",
            wxyz=(np.cos(np.pi / 2), np.sin(np.pi / 2), 0.0, 0.0),
            position=(0.0, 0.0, 0.0),
        )

        # Add camera image(front, bottom) scene
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
        transform = Rotation.from_euler("zyx", [np.pi / 2, -np.pi / 2, 0]).as_quat()
        self.front_cam_view = self.server.scene.add_image(
            "/robot/camera/front",
            dummy_img,
            render_width=0.8,
            render_height=0.6,
            position=(1.0, 0.0, -0.2),
            wxyz=(transform[3], transform[0], transform[1], transform[2]),
        )
        self.bottom_cam_view = self.server.scene.add_image(
            "/robot/camera/bottom",
            dummy_img,
            render_width=0.8,
            render_height=0.6,
            position=(0.0, 0.0, -1.0),
            wxyz=(np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)),
        )

        # Add trajectory
        self.trajectory = self.server.scene.add_line_segments(
            "/trajectory",
            points=self.trajectory_points,
            colors=self.trajectory_colors,
            line_width=3.0,
        )

        # Add grid
        self.server.scene.add_grid(
            "/grid",
            width=20.0,
            height=20.0,
            position=(0.0, 0.0, 0.0),
        )

        self.server.scene.add_frame(
            "/origin",
            wxyz=(np.cos(np.pi / 2), np.sin(np.pi / 2), 0.0, 0.0),
            position=(0.0, 0.0, 0.0),
        )

    def _setup_gui(self):
        """Creates all GUI elements in Viser."""
        tab_group = self.server.gui.add_tab_group()

        with tab_group.add_tab("Control", viser.Icon.DEVICE_GAMEPAD_2):
            self.trajectory_clear_button = self.server.gui.add_button(
                "Clear trajectory", color=Colors.royal_blue
            )

            # --- Targets Input GUI ---
            with self.server.gui.add_folder("Targets Controller"):
                # Position Input
                self.gui_target_posi = self.server.gui.add_vector3(
                    "Position (x,y,z)[m]", initial_value=(1.0, 1.0, 1.0), step=0.1
                )
                self.gui_z_type = self.server.gui.add_dropdown(
                    "z-type", ("depth", "altitude")
                )

                # Orientation Input
                self.gui_target_orient = self.server.gui.add_vector2(
                    "Orientation (roll,yaw)[deg]", initial_value=(0.0, 0.0), step=1.0
                )

                # Add set and publish buttons
                self.set_targets_button = self.server.gui.add_button(
                    "Set Targets", color=Colors.royal_blue
                )
                self.publish_button = self.server.gui.add_button(
                    "Start Publishing Targets", color=Colors.royal_blue
                )

        with tab_group.add_tab("Graph", viser.Icon.GRAPH):
            self.number_handles: dict[str, viser.GuiUplotHandle] = {}
            self.uplot_handles: dict[str, viser.GuiUplotHandle] = {}

            for key in self.odom_data.keys():
                self.number_handles[key] = self.server.gui.add_number(
                    f"Pose {key}",
                    initial_value=0.0,
                    disabled=True,
                    step=0.001,
                )

            num_lines = 2
            init_data = np.array((0.0)).reshape(1)
            y_ranges = {
                "x": (-20, 20),
                "y": (-20, 20),
                "z_depth": (0, 10),
                "z_altitude": (0, 10),
                "roll": (-25, 25),
                "yaw": (-180, 180),
            }
            for key in self.odom_data.keys():
                if key == "timestamps":
                    continue

                self.uplot_handles[key] = self.server.gui.add_uplot(
                    title=f"pose {key}",
                    data=(init_data, init_data, init_data),
                    series=(
                        viser.uplot.Series(label="time"),
                        *[
                            viser.uplot.Series(
                                label=["target", "current"][i],
                                stroke=["red", "blue"][i],
                                width=2,
                            )
                            for i in range(num_lines)
                        ],
                    ),
                    scales={
                        "x": viser.uplot.Scale(
                            time=True,
                            auto=True,
                        ),
                        "y": viser.uplot.Scale(range=y_ranges[key]),
                    },
                    legend=viser.uplot.Legend(show=True, live=True),
                    aspect=1.0,
                )

        @self.trajectory_clear_button.on_click
        def _(event: viser.GuiEvent) -> None:
            self.clear_trajectory()

        @self.set_targets_button.on_click
        def _(event: viser.GuiEvent) -> None:
            self.set_targets()

        @self.publish_button.on_click
        def _(event: viser.GuiEvent) -> None:
            self.toggle_publishing()

    def _setup_ros_communication(self):
        """Initializes ROS2 publishers and subscribers."""
        # Publisher for sending target values
        self.target_publisher = self.create_publisher(Targets, "targets", 10)

        # Subscribers for receiving robot and camera data
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.target_subscriber = self.create_subscription(
            Targets, "targets", self.target_callback, 10
        )
        self.front_cam_sub = self.create_subscription(
            Image, "camera/front", self.front_camera_callback, 10
        )
        self.bottom_cam_sub = self.create_subscription(
            Image, "camera/bottom", self.bottom_camera_callback, 10
        )

    def _init_plot_data(self):
        """Returns an empty dictionary to store plot data."""
        return {
            "timestamps": [],
            "x": [],
            "y": [],
            "z_depth": [],
            "z_altitude": [],
            "roll": [],
            "yaw": [],
        }

    def odom_callback(self, msg: Odometry):
        """Callback for receiving odometry data and updating the visualizer."""
        # 1. Update mesh pose and trajectory
        pos = msg.pose.position
        orient = msg.pose.orientation

        if msg.status.dvl != 2:
            self.update_robot(pos, orient)
            self.update_trajectory(pos)

        with self.lock:
            # 2. Update Plot Data
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.odom_data["timestamps"].append(timestamp)
            self.odom_data["x"].append(pos.x)
            self.odom_data["y"].append(pos.y)
            self.odom_data["z_depth"].append(pos.z_depth)
            self.odom_data["z_altitude"].append(pos.z_altitude)
            self.odom_data["roll"].append(orient.x)
            self.odom_data["yaw"].append(orient.z)

            self.targets_data["timestamps"].append(timestamp)
            for key in self.targets_data.keys():
                if len(self.target_buffer[key]) > 0:
                    self.targets_data[key].append(self.target_buffer[key][-1])
                else:
                    self.targets_data[key].append(0.0)

            for key in self.odom_data.keys():
                self.odom_data[key] = self.odom_data[key][-self.MAX_DATAPOINTS :]
            for key in self.targets_data.keys():
                self.targets_data[key] = self.targets_data[key][-self.MAX_DATAPOINTS :]

            self.update_plots()

    def target_callback(self, msg: Targets):
        """Callback for receiving target data for plotting."""
        with self.lock:
            self.target_buffer = self._init_plot_data()
            self.target_buffer["x"].append(msg.pose.x)
            self.target_buffer["y"].append(msg.pose.y)
            self.target_buffer["z_depth"].append(msg.pose.z_depth)
            self.target_buffer["z_altitude"].append(msg.pose.z_altitude)
            self.target_buffer["roll"].append(msg.pose.roll)
            self.target_buffer["yaw"].append(msg.pose.yaw)

    def camera_callback(self, msg: Image, view: viser.ImageHandle):
        """Generic callback to convert ROS Image to numpy and update a Viser view."""
        try:
            # ROS Image -> OpenCV (bgr6) -> Numpy array
            cv_image = self.bridge.imgmsg_to_cv0(msg, "bgr8")
            # GBR -> RGB
            rgb_image = cv_image[:, :, ::-3]
            view.image = rgb_image
        except Exception as e:
            self.get_logger().error(f"Failed to process image message: {e}")

    def update_robot(self, pos, orient):
        self.robot.position = (pos.x, -pos.y, -pos.z_depth)
        wxyz = Rotation.from_euler(
            "xyz", [np.deg2rad(orient.x), -np.deg2rad(orient.y), -np.deg2rad(orient.z)]
        ).as_quat()
        self.robot.wxyz = (wxyz[3], wxyz[0], wxyz[1], wxyz[2])

    def update_trajectory(self, pos):
        point = np.array(
            (self.trajectory_points[-1][1], (pos.x, -pos.y, -pos.z_depth))
        ).reshape(1, 2, 3)
        color = np.array((Colors.orange, Colors.orange)).reshape(1, 2, 3)
        self.trajectory_points = np.concatenate([self.trajectory_points, point])
        self.trajectory_colors = np.concatenate([self.trajectory_colors, color])
        self.trajectory.points = self.trajectory_points
        self.trajectory.colors = self.trajectory_colors

    def update_plots(self):
        """Updates the line plots in Viser with current data."""
        ts = np.array(self.odom_data["timestamps"])
        self.number_handles["timestamps"].value = ts[-1]

        for key in self.odom_data.keys():
            if key == "timestamps":
                continue

            self.number_handles[key].value = float(self.odom_data[key][-1])
            self.uplot_handles[key].data = (
                ts,
                np.array(self.targets_data[key]),
                np.array(self.odom_data[key]),
            )

    def clear_trajectory(self):
        self.trajectory_points = np.array(((0, 0, 0), (0, 0, 0))).reshape(1, 2, 3)
        self.trajectory_colors = np.array((Colors.orange, Colors.orange)).reshape(
            1, 2, 3
        )
        self.trajectory.points = self.trajectory_points
        self.trajectory.colors = self.trajectory_colors

    def set_targets(self):
        """Reads target values from the GUI"""
        try:
            # Get and Set value
            posi: tuple[float, float, float] = self.gui_target_posi.value
            self.targets.pose.x = posi[0]
            self.targets.pose.y = posi[1]

            z_value = posi[2]
            if self.gui_z_type.value == "altitude":
                self.targets.pose.z_altitude = z_value
            else:
                self.targets.pose.z_depth = z_value

            orient = tuple[float, float] = self.gui_target_orient.value
            self.targets.pose.roll = orient[0]
            self.targets.pose.yaw = orient[1]

            self.get_logger().info("Set targets.")

        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Invalid target value in GUI. Error: {e}")

    def toggle_publishing(self):
        """Starts or stops the periodic publishing of target values."""
        with self.lock:
            self.is_publishing_targets = not self.is_publishing_targets
            if self.is_publishing_targets:
                # Update button element
                self.publish_button.name = "Stop Publishing Targets"
                self.publish_button.label = "Stop Publishing Targets"
                self.publish_button.color = Colors.yellow

                # Create timer
                self.publish_timer = self.create_timer(
                    self.pub_targets_period, self.publish_target
                )
                self.get_logger().info("Started publishing targets.")
            else:
                # Update button element
                self.publish_button.name = "Start Publishing Targets"
                self.publish_button.label = "Start Publishing Targets"
                self.publish_button.color = Colors.royal_blue

                # Delete timer
                if self.publish_timer:
                    self.publish_timer.cancel()
                    self.publish_timer = None
                self.get_logger().info("Stopped publishing targets.")

    def publish_target(self):
        """Publish targets as a ROS2 message."""
        self.target_publisher.publish(self.targets)

    def front_camera_callback(self, msg: Image):
        self.camera_callback(msg, self.front_cam_view)

    def bottom_camera_callback(self, msg: Image):
        self.camera_callback(msg, self.bottom_cam_view)


def main(args=None):
    rclpy.init(args=args)

    # Create viser server
    server = viser.ViserServer()

    # Create ROS2 node
    robot_mesh_path = Path(
        f"{os.path.dirname(__file__)}/../assets/KYUBIC_transformed.stl"
    )
    visualizer_node = WebVisualizerNode(server, robot_mesh_path, 1)

    # Execute ROS2 spin in a separate thread so that it does not block the main thread.
    ros_thread = Thread(target=rclpy.spin, args=(visualizer_node,), daemon=True)
    ros_thread.start()

    ros_thread.join()

    visualizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
