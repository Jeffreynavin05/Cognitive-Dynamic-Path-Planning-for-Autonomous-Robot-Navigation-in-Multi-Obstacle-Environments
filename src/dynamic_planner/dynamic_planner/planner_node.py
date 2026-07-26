"""Risk-aware local path planner for Phase 1.

Hosts a nav2_msgs/action/NavigateToPose action server directly -- no real
Nav2 bringup (bt_navigator, lifecycle-managed servers, a C++ costmap plugin)
is required to build, test, or run this module (approved design decision,
see PROJECT_CONTEXT.md section 9c/16). Turns interfaces/ObstacleRiskArray,
interfaces/PredictedTrajectoryArray, and the robot's own /wheel/odom into
velocity commands on /cmd_vel_nav -- the fixed contract robot_controller
(Module 8) consumes. Any planner backend (this project's deterministic
weighted-scoring local planner, a future real Nav2/MPPI stack) must publish
this exact shape so nothing downstream ever has to change.

Staged the same way risk_node/prediction_node are, extended for an
action-server + timer-driven control loop instead of a single synchronous
subscription callback (see the control-loop timing design decision in
section 16 -- a velocity-command publisher cannot go silent between upstream
messages the way a pass-through stage can):
    _odom_callback            -- INPUT (cache). Robot position/yaw/velocity.
    _risks_callback           -- INPUT (cache). Latest ObstacleRiskArray, by track_id.
    _trajectories_callback    -- INPUT (cache). Latest PredictedTrajectoryArray, by track_id.
    _goal_callback/_cancel_callback/_execute_callback
                                -- INPUT (goal). Standard NavigateToPose action-server
                                   handlers; _execute_callback records the goal,
                                   publishes the (visualization-only) global path once,
                                   then waits for the control loop to report success/
                                   cancellation.
    _control_loop              -- CONTROL (drive). Timer callback (control_rate_hz);
                                   entry point for every stage below, run once per cycle
                                   whenever a goal is active.
    _join_obstacles             -- JOIN.        Combines cached trajectories (geometry)
                                   with cached risks (explainable priority), by track_id.
    _compute_command             -- PLAN.         dynamic_planner.local_planner.select_command(),
                                   swappable for a learned/MPPI backend later without
                                   touching any other stage.
    _build_cmd_vel                -- ASSEMBLE.     Pure message-building, no I/O.
    _publish_cmd                   -- OUTPUT.       The only method that touches the
                                   cmd_vel publisher.

ObstacleRiskArray's threat_level/time_to_collision are used only as a coarse
emergency-stop safety gate (_emergency_stop_triggered), never folded into
per-candidate scoring -- local_planner.py only ever sees obstacle geometry,
per the approved "clean division of labour" decision for this module.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.msg import ObstacleRisk, ObstacleRiskArray, PredictedTrajectory, PredictedTrajectoryArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from dynamic_planner.global_path import generate_straight_line_path
from dynamic_planner.local_planner import ObstacleView, PlannedCommand, PlannerParams, select_command, \
    yaw_from_quaternion

ODOM_TOPIC = '/wheel/odom'
RISKS_TOPIC = '/risk/obstacle_risks'
TRAJECTORIES_TOPIC = '/prediction/trajectories'
CMD_VEL_TOPIC = '/cmd_vel_nav'
GLOBAL_PATH_TOPIC = '/planner/global_path'
NAVIGATE_ACTION = 'navigate_to_pose'


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_linear_speed_mps', 0.5)
        self.declare_parameter('max_angular_speed_radps', 1.5)
        self.declare_parameter('max_linear_accel_mps2', 0.5)
        self.declare_parameter('max_angular_accel_radps2', 2.0)
        self.declare_parameter('num_linear_samples', 5)
        self.declare_parameter('num_angular_samples', 7)
        self.declare_parameter('local_horizon_sec', 1.5)
        self.declare_parameter('local_step_sec', 0.1)
        self.declare_parameter('robot_radius_m', 0.2)
        self.declare_parameter('obstacle_radius_m', 0.4)
        self.declare_parameter('safety_margin_m', 1.0)
        self.declare_parameter('weight_progress', 0.5)
        self.declare_parameter('weight_heading', 0.2)
        self.declare_parameter('weight_clearance', 0.3)
        self.declare_parameter('goal_tolerance_m', 0.15)
        self.declare_parameter('emergency_stop_ttc_sec', 0.5)
        self.declare_parameter('waypoint_spacing_m', 0.5)

        gp = self.get_parameter
        control_rate_hz = gp('control_rate_hz').value
        self._params = PlannerParams(
            max_linear_speed_mps=gp('max_linear_speed_mps').value,
            max_angular_speed_radps=gp('max_angular_speed_radps').value,
            max_linear_accel_mps2=gp('max_linear_accel_mps2').value,
            max_angular_accel_radps2=gp('max_angular_accel_radps2').value,
            num_linear_samples=gp('num_linear_samples').value,
            num_angular_samples=gp('num_angular_samples').value,
            local_horizon_sec=gp('local_horizon_sec').value,
            local_step_sec=gp('local_step_sec').value,
            collision_radius_m=gp('robot_radius_m').value + gp('obstacle_radius_m').value,
            safety_margin_m=gp('safety_margin_m').value,
            weight_progress=gp('weight_progress').value,
            weight_heading=gp('weight_heading').value,
            weight_clearance=gp('weight_clearance').value,
            control_period_sec=1.0 / control_rate_hz,
        )
        self._waypoint_spacing_m = gp('waypoint_spacing_m').value
        self._goal_tolerance_m = gp('goal_tolerance_m').value
        self._emergency_stop_ttc_sec = gp('emergency_stop_ttc_sec').value

        # Cached from the most recent /wheel/odom message, body-frame v/omega
        # (correct input for local_planner's unicycle simulation, which
        # starts from robot_yaw -- no world-frame rotation needed here,
        # unlike risk_node's use of odometry).
        self._robot_position = np.zeros(3)
        self._robot_yaw = 0.0
        self._robot_linear_velocity = 0.0
        self._robot_angular_velocity = 0.0
        self._have_odom = False

        self._latest_risks: dict[int, ObstacleRisk] = {}
        self._latest_trajectories: dict[int, PredictedTrajectory] = {}
        self._latest_trajectories_header = Header()

        # Simple attribute reads/writes, not lock-protected: both are shared
        # between the control-loop timer and the action-server execute
        # callback, which run on different executor threads (see main()).
        # CPython's GIL makes each individual assignment atomic, which is
        # sufficient for this Phase-1 reference implementation -- a single
        # active goal, no compound updates that need to be atomic together.
        self._active_goal_position: np.ndarray | None = None
        self._goal_reached = False

        control_group = MutuallyExclusiveCallbackGroup()
        action_group = MutuallyExclusiveCallbackGroup()

        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_callback, 10, callback_group=control_group)
        self._risks_sub = self.create_subscription(
            ObstacleRiskArray, RISKS_TOPIC, self._risks_callback, 10, callback_group=control_group)
        self._trajectories_sub = self.create_subscription(
            PredictedTrajectoryArray, TRAJECTORIES_TOPIC, self._trajectories_callback, 10,
            callback_group=control_group)

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._global_path_pub = self.create_publisher(Path, GLOBAL_PATH_TOPIC, 10)

        self._control_timer = self.create_timer(
            1.0 / control_rate_hz, self._control_loop, callback_group=control_group)

        self._action_server = ActionServer(
            self, NavigateToPose, NAVIGATE_ACTION,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_group)

        self.get_logger().info(
            f'planner_node ready: {NAVIGATE_ACTION} action server up, publishing {CMD_VEL_TOPIC} at '
            f'{control_rate_hz} Hz once a goal is active (collision_radius={self._params.collision_radius_m}m).')

    # ---- input: cache only, no processing --------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._robot_position = np.array([position.x, position.y, position.z])
        self._robot_yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        self._robot_linear_velocity = msg.twist.twist.linear.x
        self._robot_angular_velocity = msg.twist.twist.angular.z
        self._have_odom = True

    def _risks_callback(self, msg: ObstacleRiskArray) -> None:
        self._latest_risks = {risk.track_id: risk for risk in msg.risks}

    def _trajectories_callback(self, msg: PredictedTrajectoryArray) -> None:
        self._latest_trajectories_header = msg.header
        self._latest_trajectories = {trajectory.track_id: trajectory for trajectory in msg.trajectories}

    # ---- input: goal lifecycle (standard NavigateToPose action-server shape) ---

    def _goal_callback(self, goal_request: NavigateToPose.Goal) -> GoalResponse:
        # One active goal at a time in Phase 1 -- a new goal while one is
        # already executing is rejected rather than preempting it.
        if self._active_goal_position is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        goal_point = goal_handle.request.pose.pose.position
        self._active_goal_position = np.array([goal_point.x, goal_point.y, goal_point.z])
        self._goal_reached = False
        self._publish_global_path(self._active_goal_position)

        feedback = NavigateToPose.Feedback()
        rate = self.create_rate(2.0)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._active_goal_position = None
                self._stop()
                return NavigateToPose.Result()
            if self._goal_reached:
                goal_handle.succeed()
                self._active_goal_position = None
                return NavigateToPose.Result()

            if self._have_odom and self._active_goal_position is not None:
                feedback.distance_remaining = float(
                    np.linalg.norm(self._active_goal_position - self._robot_position))
                goal_handle.publish_feedback(feedback)
            rate.sleep()

        self._active_goal_position = None
        self._stop()
        return NavigateToPose.Result()

    # ---- control: drives every stage below, in order, once per cycle ----------

    def _control_loop(self) -> None:
        if self._active_goal_position is None or not self._have_odom:
            return

        distance_to_goal = float(np.linalg.norm(self._active_goal_position - self._robot_position))
        if distance_to_goal <= self._goal_tolerance_m:
            self._goal_reached = True
            self._stop()
            return

        if self._emergency_stop_triggered():
            self._stop()
            return

        obstacles = self._join_obstacles()
        command = self._compute_command(obstacles)

        twist = self._build_cmd_vel(command)
        self._publish_cmd(twist)

    def _emergency_stop_triggered(self) -> bool:
        return any(
            risk.threat_level == ObstacleRisk.THREAT_CRITICAL and 0.0 <= risk.time_to_collision <= self._emergency_stop_ttc_sec
            for risk in self._latest_risks.values())

    # ---- join (swappable independently of every other stage) -------------------

    def _join_obstacles(self) -> list[ObstacleView]:
        obstacles = []
        for track_id, trajectory in self._latest_trajectories.items():
            if track_id not in self._latest_risks:
                # Documented Phase-1 simplification: an obstacle risk_node
                # hasn't scored yet this cycle is skipped rather than
                # planned around blind -- negligible given both topics are
                # published from the same upstream cadence.
                continue
            offsets, positions = self._extract_points(trajectory)
            obstacles.append(ObstacleView(track_id=track_id, offsets=offsets, positions=positions))
        return obstacles

    def _extract_points(self, trajectory: PredictedTrajectory) -> tuple[list[float], list[np.ndarray]]:
        base_stamp = Time.from_msg(self._latest_trajectories_header.stamp)
        offsets = []
        positions = []
        for point in trajectory.points:
            offsets.append((Time.from_msg(point.stamp) - base_stamp).nanoseconds / 1e9)
            positions.append(np.array([point.position.x, point.position.y, point.position.z]))
        return offsets, positions

    # ---- plan (swappable for a learned/MPPI backend later) ---------------------

    def _compute_command(self, obstacles: list[ObstacleView]) -> PlannedCommand:
        return select_command(
            self._robot_position, self._robot_yaw, self._robot_linear_velocity, self._robot_angular_velocity,
            self._active_goal_position, obstacles, self._params)

    # ---- assemble (pure, no I/O -- directly unit-testable) ----------------------

    def _build_cmd_vel(self, command: PlannedCommand) -> Twist:
        twist = Twist()
        twist.linear.x = command.linear_velocity
        twist.angular.z = command.angular_velocity
        return twist

    def _build_global_path(self, goal_position: np.ndarray) -> Path:
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for waypoint in generate_straight_line_path(self._robot_position, goal_position, self._waypoint_spacing_m):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = waypoint.tolist()
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    # ---- output -------------------------------------------------------------------

    def _publish_cmd(self, twist: Twist) -> None:
        self._cmd_pub.publish(twist)

    def _publish_global_path(self, goal_position: np.ndarray) -> None:
        self._global_path_pub.publish(self._build_global_path(goal_position))

    def _stop(self) -> None:
        self._publish_cmd(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    # MultiThreadedExecutor, not the single-threaded rclpy.spin() every other
    # node in this workspace uses -- the action server's long-running
    # _execute_callback and the fixed-rate _control_loop timer must run
    # concurrently (see the control-loop timing design decision, section 16).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
