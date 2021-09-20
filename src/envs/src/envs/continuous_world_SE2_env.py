from Box2D import b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color
import Box2D
from backends.rendering import Viewer, Transform
import sys
import os
import rospkg

sys.path.append(os.path.join(rospkg.RosPack().get_path("simulators"), "scripts"))
from corrective_mode_switch_utils import *
import numpy as np
import collections
import itertools
import rospy
import threading
from envs.srv import PASAllG, PASAllGRequest, PASAllGResponse
from envs.msg import PASSingleG
from mdp.mdp_discrete_SE2_gridworld_with_modes import MDPDiscreteSE2GridWorldWithModes
from scipy.spatial import KDTree
import time


class ContinuousWorldSE2Env(object):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": FPS}

    def __init__(self, env_params):
        self.viewer = None
        self.world = Box2D.b2World()
        self.world.gravity = (0, 0)

        self.robot = None
        self.robot_position = None
        self.robot_orientation = None
        self.goal_poses = None
        self.world_bounds = None

        self.env_params = env_params

        assert self.env_params is not None
        assert "num_goals" in self.env_params
        assert "robot_position" in self.env_params
        assert "robot_orientation" in self.env_params

        assert "goal_poses" in self.env_params
        assert "start_mode" in self.env_params
        assert "world_bounds" in self.env_params

        self.service_initialized = False

        self.ready_for_new_prompt = True
        self.text_display_delay = 2
        self.start_msg_ind = 0

    def get_prob_a_s_all_g(self, req):
        response = PASAllGResponse()
        current_discrete_mdp_state = rospy.get_param("current_discrete_mdp_state", [0, 0, 0, 1])
        current_discrete_mdp_state = tuple(current_discrete_mdp_state)  # [x,y,t,m]
        p_a_s_all_g = []
        optimal_action_s_g = []
        for g in range(self.num_goals):
            mdp_g = self.mdp_list[g]
            p_a_s_g_msg = PASSingleG()
            p_a_s_g_msg.goal_id = g
            for task_level_action in TASK_LEVEL_ACTIONS:
                p_a_s_g_msg.p_a_s_g.append(mdp_g.get_prob_a_given_s(current_discrete_mdp_state, task_level_action))
            p_a_s_all_g.append(p_a_s_g_msg)
            # optimal task_level action to take in current state for goal g. Used to modify phm in inference node
            optimal_action_s_g.append(mdp_g.get_optimal_action(current_discrete_mdp_state, return_optimal=True))

        response.p_a_s_all_g = p_a_s_all_g
        response.optimal_action_s_g = optimal_action_s_g
        response.status = True
        return response

    def _continuous_orientation_to_discrete_orientation(self):
        cont_orientation = self.robot.get_angle()
        # wrap angle back to  0 to 2PI
        while not (cont_orientation >= 0.0 and cont_orientation < 2 * PI):
            if cont_orientation >= 2 * PI:
                cont_orientation -= 2 * PI
            if cont_orientation < 0.0:
                cont_orientation += 2 * PI

        # discretize to current discrete MDP orientation
        angle_resolution = (2 * PI) / self.mdp_num_discrete_orientations
        mdp_discrete_orientation = int(round(cont_orientation / angle_resolution))
        # in the mdp class "move_p" results in "positive" change in angle which is counterclockwise motion
        # however, in the environment, a move_p command results in clockwise motion.
        # either change the transition model in MDP or change mapping in env.
        # print(mdp_discrete_orientation)
        return mdp_discrete_orientation

    def _transform_continuous_robot_pose_to_discrete_state(self):
        data_index = self.continuous_kd_tree.query(self.robot.get_position())[1]
        nearest_continuous_position = self.continuous_kd_tree.data[data_index, :]
        mdp_discrete_position = self.continuous_position_to_loc_coord[tuple(nearest_continuous_position)]
        mdp_discrete_orientation = self._continuous_orientation_to_discrete_orientation()
        current_mode_index = rospy.get_param("mode")  # 0,1,2

        mdp_discrete_state = [
            mdp_discrete_position[0],
            mdp_discrete_position[1],
            mdp_discrete_orientation,
            current_mode_index + 1,  # plus 1 because in MDP the modes are 1-indexed.
        ]
        # add mode to state info. mode 1,2,3
        return mdp_discrete_state

    def _render_bounds(self):
        x_lb = self.world_bounds["xrange"]["lb"]
        x_ub = self.world_bounds["xrange"]["ub"]
        y_lb = self.world_bounds["yrange"]["lb"]
        y_ub = self.world_bounds["yrange"]["ub"]
        self.viewer.draw_line((x_lb, y_lb), (x_ub, y_lb), linewidth=5.0)  # bottom
        self.viewer.draw_line((x_ub, y_lb), (x_ub, y_ub), linewidth=5.0)  # right
        self.viewer.draw_line((x_ub, y_ub), (x_lb, y_ub), linewidth=5.0)  # top
        self.viewer.draw_line((x_lb, y_ub), (x_lb, y_lb), linewidth=5.0)  # left

    def _render_grid_lines(self):
        if self.is_visualize_grid:
            # continuous bounds
            x_lb = self.world_bounds["xrange"]["lb"]
            x_ub = self.world_bounds["xrange"]["ub"]
            y_lb = self.world_bounds["yrange"]["lb"]
            y_ub = self.world_bounds["yrange"]["ub"]
            cell_size_x = self.all_mdp_env_params["cell_size"]["x"]
            cell_size_y = self.all_mdp_env_params["cell_size"]["y"]
            grid_width = self.all_mdp_env_params["grid_width"]
            grid_height = self.all_mdp_env_params["grid_height"]

            # starting from one because bounds are already created
            iw_list = list(range(1, grid_width))
            ih_list = list(range(1, grid_height))
            # f draw vertical lines
            for i in iw_list:
                start_point = (x_lb + i * cell_size_x, y_lb)
                end_point = (x_lb + i * cell_size_x, y_ub)
                self.viewer.draw_line(start_point, end_point, linewidth=2.0)

            for i in ih_list:
                start_point = (x_lb, y_lb + i * cell_size_y)
                end_point = (x_ub, y_lb + i * cell_size_y)
                self.viewer.draw_line(start_point, end_point, linewidth=2.0)

    def _render_goal(self, shape, goal_color, goal_pose):  # potentially add default values for these args
        goal_position = (goal_pose[0], goal_pose[1])
        goal_orientation = goal_pose[2]
        if shape == "circle":
            t = Transform(translation=goal_position)
            self.viewer.draw_circle(GOAL_RADIUS / SCALE, 30, color=goal_color, filled=True).add_attr(t)
            self.viewer.draw_line(
                goal_position,
                (
                    goal_position[0] + 2 * (GOAL_RADIUS / SCALE) * math.cos(goal_orientation),
                    goal_position[1] + 2 * (GOAL_RADIUS / SCALE) * math.sin(goal_orientation),
                ),
                linewidth=3.0,
            )

    def _render_goals(self):
        for i in range(self.num_goals):
            shape = "circle"
            goal_color = self.GOAL_COLORS[i]
            goal_pose = tuple(self.goal_poses[i])
            self._render_goal(shape, goal_color, goal_pose)

    def _render_goal_constraints(self): 
        # for goal_const_spec in self.goal_constraints: 
        #     v = 
        #     self.viewer.draw_polygon(v)
        pass

    def _render_obstacles(self):
        for obs_spec in self.obstacles:
            # bottom left corner and top-right corner
            bottom_left = obs_spec["bottom_left"]  # 2d list
            top_right = obs_spec["top_right"]
            bottom_right = [top_right[0], bottom_left[1]]
            top_left = [bottom_left[0], top_right[1]]
            v = [bottom_left, bottom_right, top_right, top_left]
            self.viewer.draw_polygon(v)

    def _render_robot(self):
        robot = self.robot.robot
        for f in robot.fixtures:
            trans = f.body.transform
            if type(f.shape) is b2CircleShape:
                t = Transform(translation=trans.position)
                self.viewer.draw_circle(f.shape.radius, 30, color=self.robot.robot_color, filled=True).add_attr(t)

    def _render_robot_direction_indicators(self):
        ep_markers = self.robot.get_direction_marker_end_points()
        self.viewer.draw_line(ep_markers[0], ep_markers[1], linewidth=3.0)

    def _render_mode_display(self):
        for i, d in enumerate(self.DIMENSIONS):
            if d == "y":
                t = Transform(
                    translation=(
                        MODE_DISPLAY_CIRCLE_START_POSITION_S[0] + i * MODE_DISPLAY_CIRCLE_X_OFFSET_S,
                        MODE_DISPLAY_CIRCLE_START_POSITION_S[1],
                    )
                )
            else:
                t = Transform(
                    translation=(
                        MODE_DISPLAY_CIRCLE_START_POSITION_S[0] + i * MODE_DISPLAY_CIRCLE_X_OFFSET_S,
                        MODE_DISPLAY_CIRCLE_START_POSITION_S[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S,
                    )
                )
            if d == self.current_mode:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS / SCALE, 30, True, color=ACTIVE_MODE_COLOR).add_attr(t)
            else:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS / SCALE, 30, True, color=NONACTIVE_MODE_COLOR).add_attr(t)

    def _render_mode_display_text(self):
        """
        Note that the coordinates of the text should in real pixels. Which is why here there is a multiplicative factor of SCALE.
        """
        self.viewer.draw_text(
            "Horizontal",
            x=MODE_DISPLAY_TEXT_START_POSITION[0],
            y=MODE_DISPLAY_TEXT_START_POSITION[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S * SCALE,
            font_size=MODE_DISPLAY_TEXT_FONTSIZE,
            color=MODE_DISPLAY_TEXT_COLOR,
            anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR,
        )
        self.viewer.draw_text(
            "Vertical",
            x=MODE_DISPLAY_TEXT_START_POSITION[0] + MODE_DISPLAY_TEXT_X_OFFSET,
            y=MODE_DISPLAY_TEXT_START_POSITION[1],
            font_size=MODE_DISPLAY_TEXT_FONTSIZE,
            color=MODE_DISPLAY_TEXT_COLOR,
            anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR,
        )
        self.viewer.draw_text(
            "Rotation",
            x=MODE_DISPLAY_TEXT_START_POSITION[0] + 2 * MODE_DISPLAY_TEXT_X_OFFSET,
            y=MODE_DISPLAY_TEXT_START_POSITION[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S * SCALE,
            font_size=MODE_DISPLAY_TEXT_FONTSIZE,
            color=MODE_DISPLAY_TEXT_COLOR,
            anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR,
        )

    def _render_timer_text(self):
        if self.current_time < TIMER_WARNING_THRESHOLD:
            self.viewer.draw_text(
                str(self.current_time),
                x=TIMER_DISPLAY_POSITION[0],
                y=TIMER_DISPLAY_POSITION[1],
                font_size=TIMER_DISPLAY_FONTSIZE,
                color=TIMER_COLOR_NEUTRAL,
                anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR,
                bold=True,
            )
        elif self.current_time < TIMER_DANGER_THRESHOLD:
            self.viewer.draw_text(
                str(self.current_time),
                x=TIMER_DISPLAY_POSITION[0],
                y=TIMER_DISPLAY_POSITION[1],
                font_size=TIMER_DISPLAY_FONTSIZE,
                color=TIMER_COLOR_WARNING,
                anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR,
                bold=True,
            )
        else:
            self.viewer.draw_text(
                str(self.current_time),
                x=TIMER_DISPLAY_POSITION[0],
                y=TIMER_DISPLAY_POSITION[1],
                font_size=TIMER_DISPLAY_FONTSIZE,
                color=TIMER_COLOR_DANGER,
                anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR,
                bold=True,
            )

    def _render_text(self, msg, position, font_size=TRIAL_OVER_TEXT_FONTSIZE, color=COMMAND_TEXT_COLOR):
        self.viewer.draw_text(msg, x=position[0], y=position[1], font_size=font_size, color=color, bold=True)

    def _render_timer(self, period):
        while not rospy.is_shutdown():
            start = rospy.get_rostime()
            self.lock.acquire()
            if self.viewer is not None:
                self.current_time += 1
            else:
                pass
            self.lock.release()
            end = rospy.get_rostime()
            if end - start < period:
                rospy.sleep(period - (end - start))
            else:
                rospy.loginfo("took more time")

    def render(self, mode="human"):
        self._render_bounds()
        self._render_grid_lines()
        self._render_obstacles()
        self._render_goals()
        # self._render_goal_constraints()

        self._render_robot()
        self._render_robot_direction_indicators()
        self._render_mode_display()
        self._render_mode_display_text()
        # render timer
        self._render_timer_text()
        # #render assistance block label
        # self._render_text('Condition: '+self.assistance_type, LABEL_DISPLAY_POSITION, MODE_DISPLAY_TEXT_FONTSIZE+5)

        return self.viewer.render(False)

    def initialize_viewer(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W / SCALE, 0, VIEWPORT_H / SCALE)
            self.viewer.window.set_location(650, 300)
            self.timer_thread.start()

    def start_countdown(self):
        if self.ready_for_new_prompt:
            self.msg_prompt = EXPERIMENT_START_COUNTDOWN[self.start_msg_ind]
            self.start_msg_ind += 1
            self.ts = time.time()
            self.ready_for_new_prompt = False
        if (time.time() - self.ts) >= self.text_display_delay:
            self.ready_for_new_prompt = True

        self._render_text(self.msg_prompt, COMMAND_DISPLAY_POSITION)
        self.viewer.render(False)

        if self.start_msg_ind == len(EXPERIMENT_START_COUNTDOWN):
            return 1
        else:
            return 0

    def _destroy(self):
        if self.robot is None:
            return
        self.world.DestroyBody(self.robot.robot)
        self.robot = None
        self.robot_position = None
        self.robot_orientation = None
        self.goal_poses = None
        self.world_bounds = None
        self.DIMENSIONS = []
        self.DIMENSION_INDICES = []

    def initialize(self):
        self.start_session = self.env_params["start"]
        self.period = rospy.Duration(1.0)
        # timer for trial time. Run on separate thread
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,))
        self.lock = threading.Lock()
        self.current_time = 0
        self.max_time = 50

    def _create_kd_tree_locations(self):
        grid_width = self.all_mdp_env_params["grid_width"]
        grid_height = self.all_mdp_env_params["grid_height"]
        cell_size_x = self.all_mdp_env_params["cell_size"]["x"]
        cell_size_y = self.all_mdp_env_params["cell_size"]["y"]
        # offset for bottom left corner.
        world_x_lb = self.world_bounds["xrange"]["lb"]
        world_y_lb = self.world_bounds["yrange"]["lb"]
        cell_center_list = []
        data = np.zeros((grid_width * grid_height, 2))
        self.coord_to_continuous_position_dict = collections.OrderedDict()
        for i in range(grid_width):
            for j in range(grid_height):
                # create center continuous position for i, j
                # append it to cell_center_list
                data[i * grid_height + j, 0] = i * cell_size_x + cell_size_x / 2.0 + world_x_lb
                data[i * grid_height + j, 1] = j * cell_size_y + cell_size_y / 2.0 + world_y_lb
                self.coord_to_continuous_position_dict[(i, j)] = tuple(data[i * grid_height + j, :])

        self.continuous_position_to_loc_coord = {v: k for k, v in self.coord_to_continuous_position_dict.items()}
        # create kd tree with the cell_center_list. Use euclidean distance in 2d space for nearest neight
        self.continuous_kd_tree = KDTree(data)

    def reset(self):
        self._destroy()

        # red, green, blue, tello, cyan
        self.GOAL_COLORS = {0: (1, 0, 0), 1: (0, 1, 0), 2: (0, 0, 1), 3: (1, 1, 0), 4: (0, 1, 1)}
        # For each goal there needs to be an MDP under the hood.

        self.all_mdp_env_params = self.env_params["all_mdp_env_params"]
        self.mdp_list = self.env_params["mdp_list"]
        self.mdp_num_discrete_orientations = self.all_mdp_env_params["num_discrete_orientations"]

        # create KDTree with cell center locations

        #
        self.is_visualize_grid = self.env_params["is_visualize_grid"]
        self.num_goals = self.env_params["num_goals"]

        # continuous starting robot pose
        self.robot_position = self.env_params["robot_position"]
        self.robot_orientation = self.env_params["robot_orientation"]
        # starting discrete mode
        self.start_mode = self.env_params["start_mode"]

        # continuous goal poses.
        self.goal_poses = self.env_params["goal_poses"]
        # self.goal_constraints = self.env_params["goal_constraints"]

        # continuous world boundaries
        self.world_bounds = self.env_params["world_bounds"]
        self._create_kd_tree_locations()
        # continuous obstacle boundaries.
        self.obstacles = self.env_params["obstacles"]
        # self.assistance_type = ASSISTANCE_CODE_NAME[self.env_params["assistance_type"]]  # label assistance type

        self.DIMENSIONS = ["x", "y", "t"]  # set of dimensions or modes
        self.DIMENSION_INDICES = np.array([0, 1, 2])  # set of mode indices (needed for look up)

        self.current_mode_index = DIM_TO_MODE_INDEX[self.start_mode]
        self.current_mode = self.start_mode
        self.mode_display_label = None

        # create robot
        self.robot = RobotSE2(
            self.world,
            position=self.robot_position,
            orientation=self.robot_orientation,
            robot_color=ROBOT_COLOR_WHEN_COMMAND_REQUIRED,
            radius=ROBOT_RADIUS / SCALE,
        )

        if not self.service_initialized:
            rospy.Service("/sim_env/get_prob_a_s_all_g", PASAllG, self.get_prob_a_s_all_g)
            self.service_initialized = True

        # self.LOCS_FOR_CONTROL_DIM_DISPLAY = collections.OrderedDict()
        # for i, cd in enumerate(range(self.robot_dim)):
        #     self.LOCS_FOR_CONTROL_DIM_DISPLAY[cd] = collections.OrderedDict()
        #     self.LOCS_FOR_CONTROL_DIM_DISPLAY[cd]["position"] = (
        #         MODE_DISPLAY_CIRCLE_START_POSITION_S[0] + i * MODE_DISPLAY_CIRCLE_X_OFFSET_S,
        #         MODE_DISPLAY_CIRCLE_START_POSITION_S[1],
        #     )

    def _check_if_robot_in_bounds(self):
        robot_position = self.robot.get_position()
        x_lb = self.world_bounds["xrange"]["lb"] + ROBOT_RADIUS / SCALE
        x_ub = self.world_bounds["xrange"]["ub"] - ROBOT_RADIUS / SCALE

        y_lb = self.world_bounds["yrange"]["lb"] + ROBOT_RADIUS / SCALE
        y_ub = self.world_bounds["yrange"]["ub"] - ROBOT_RADIUS / SCALE
        if (
            robot_position[0] < x_lb
            or robot_position[0] > x_ub
            or robot_position[1] < y_lb
            or robot_position[1] > y_ub
        ):
            return False
        else:
            return True

    def _check_if_robot_in_obstacle(self):
        robot_position = self.robot.get_position()
        in_obstacle = False
        for obs_spec in self.obstacles:
            bottom_left = obs_spec["bottom_left"]  # 2d list
            top_right = obs_spec["top_right"]
            # corners of the obstacle
            x_lb = bottom_left[0] - ROBOT_RADIUS / SCALE
            x_ub = top_right[0] + ROBOT_RADIUS / SCALE
            y_lb = bottom_left[1] - ROBOT_RADIUS / SCALE
            y_ub = top_right[1] + ROBOT_RADIUS / SCALE

            if (
                robot_position[0] > x_lb
                and robot_position[0] < x_ub
                and robot_position[1] > y_lb
                and robot_position[1] < y_ub
            ):
                in_obstacle = True
                break

        if in_obstacle:
            return True
        else:
            return False

    def step(self, input_action):
        assert "human" in input_action.keys()
        # transform the continuous robot position to a discrete state representation.
        # Easier for computing the optimal action etc

        # restrict the nonzero components of the velocity only to the allowed modes.
        self.current_mode_index = rospy.get_param("mode")  # 0,1,2 #get current mode index
        self.current_mode = MODE_INDEX_TO_DIM[self.current_mode_index]  # x,y,t, #get current mode
        # print(self.current_mode, self.current_mode_index)

        # x,y,t #for the given location, retrieve what is the allowed mode of motion.
        # current_allowed_mode = self._retrieve_current_allowed_mode()
        # 0,1,2 #get the mode index of the allowed mode of motion
        # current_allowed_mode_index = DIM_TO_MODE_INDEX[current_allowed_mode]
        prev_robot_position = self.robot.get_position()
        user_vel = np.array(
            [
                input_action["human"].velocity.data[0],
                input_action["human"].velocity.data[1],
                input_action["human"].velocity.data[2],
            ]
        )  # numpyify the velocity data. note the negative sign on the 3rd component.To account for proper counterclockwise motion
        # zero out all velocities except the ones for the allowed mode
        # user_vel[np.setdiff1d(self.DIMENSION_INDICES, current_allowed_mode_index)] = 0.0

        self.robot.robot.linearVelocity = b2Vec2(user_vel[0], user_vel[1])  # update robot velocity
        # the negative sign is included so that 'move_p' which is "positive direction"
        self.robot.robot.angularVelocity = -user_vel[2]
        # results in clockwise motion! In degrees this is a negative change.

        self.world.Step(1.0 / FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS)  # call box2D step function

        if self._check_if_robot_in_obstacle():
            self.robot.set_position(prev_robot_position)

        if not self._check_if_robot_in_bounds():
            self.robot.set_position(prev_robot_position)

        self.current_discrete_mdp_state = self._transform_continuous_robot_pose_to_discrete_state()
        rospy.set_param("current_discrete_mdp_state", self.current_discrete_mdp_state)
        # print("Discrete state", self.current_discrete_mdp_state)

        return (
            self.robot.get_position(),
            self.robot.get_angle(),
            [user_vel[0], user_vel[1]],
            -user_vel[2],
            self.current_mode,
            False,
        )
