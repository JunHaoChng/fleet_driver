Copyright [2020] [name of copyright owner]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


# Code attribution from https://github.com/osrf/rmf/blob/master/ros2/fleet_adapter_mir/fleet_adapter_mir/fleet_adapter_mir/

import enum
import math
import time
import argparse
import json
import threading
import nudged

import mir100_client
from mir100_client.rest import ApiException
from mir100_client.models import PostMissionQueues, PostMissions, PostMissionActions, PutStatus
import urllib3

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import PathRequest, ModeRequest, RobotState, FleetState, \
    Location, RobotMode, ModeParameter


class Robot():
    def __init__(self, parent):
        self.parent = parent
        self.name = None
        self.api = None
        self.missions = {}
        self.maps = {}
        self.positions = {}
        self.current_map = None
        self.current_target = None
        self.prev_target = None
        self.place_sub = None
        self.mode_sub = None
        self.mode = None
        self.current_task_id = 'idle'
        self.remaining_path = []

        self.docking_executed = False
        self.docking_requested = False

        # Variables for managing the path queue execution thread
        self._path_following_thread = None
        self._path_quit_event = threading.Event()
        self._path_quit_cv = threading.Condition()


    def cancel_path(self):
        self.remaining_path.clear()
        self.mode = RobotMode.MODE_PAUSED

        if self._path_following_thread is not None:
            self._path_quit_event.set()
            self._path_quit_cv.acquire()
            self._path_quit_cv.notify_all()
            self._path_quit_cv.release()
            self._path_following_thread.join()

        self.api.mission_queue_delete()


    def follow_new_path(self, msg):
        self.docking_requested = False
        self.docking_executed = False
        self.current_task_id = msg.task_id
        self.cancel_path()

        # Set MiR state from PAUSE (if) to READY everytime receives new path
        # pick up from commit 15b2bfc
        status = PutStatus(state_id = MirState.READY)
        self.api.status_put(status)

        def path_following_closure():
            # This function defines a worker thread that will wakeup at whatever times are needed
            while (not self._path_quit_event.is_set()) and self.remaining_path:
                next_ros_time = self.remaining_path[0].t
                next_mission_time = next_ros_time.sec + next_ros_time.nanosec/1e9

                next_mission_wait = next_mission_time - time.time()
                # print(f'next_mission_time: {next_mission_time}, \
                #       next_mission_wait: {next_mission_wait}')
                if next_mission_wait <= 0 and self.mode == MirState.READY and self.remaining_path:
                    self.remaining_path.pop(0)

                    if not self.remaining_path:
                        return

                    next_mission_location = self.remaining_path[0]
                    mir_p = self.parent.rmf2mir_transform.transform(
                        [next_mission_location.x, next_mission_location.y])

                    mir_location = Location()
                    mir_location.x = mir_p[0]
                    mir_location.y = mir_p[1]
                    yaw = math.degrees(
                        next_mission_location.yaw
                        + self.parent.rmf2mir_transform.get_rotation()
                    )
                    print(f'RMF location x:{next_mission_location.x} y:{next_mission_location.y}')
                    if yaw > 180.0:
                        yaw = yaw - 360.0
                    elif yaw <= -180.0:
                        yaw = yaw + 360.0

                    mir_location.yaw = yaw

                    print(f'location: {mir_location}')
                    #Check whether mission is in mission list
                    mission_name= f'move_coordinate_to_{mir_location.x:.2f}_{mir_location.y:.2f}_{mir_location.yaw:.2f}'
                    if mission_name not in self.missions:
                        print(f'Creating a new mission named {mission_name}')
                        mission_id = self.parent.create_move_coordinate_mission(self, mir_location)
                    else:
                        mission_id = self.missions[mission_name]

                    try:
                        mission = PostMissionQueues(mission_id=mission_id)
                        self.api.mission_queue_post(mission)
                    except KeyError:
                        self.parent.get_logger().error(
                            f'no mission to move coordinates to [{mir_location.x:3f}_{mir_location.y:.3f}]!'
                        )
                    continue

                self._path_quit_cv.acquire()
                self._path_quit_cv.wait(next_mission_wait)
                self._path_quit_cv.release()

        self.remaining_path = msg.path

        self._path_quit_event.clear()
        self._path_following_thread = threading.Thread(target=path_following_closure)
        self._path_following_thread.start()


class MirState(enum.IntEnum):
    READY = 3
    PAUSE = 4
    EXECUTING = 5
    MANUAL_CONTROL = 11
    ERROR = 12


class MirPositionTypes(enum.IntEnum):
    ROBOT = 0
    CHARGING_STATION = 7
    CHARGING_STATION_ENTRY = 8


class FleetDriverMir(Node):
    FLEET_NAME = 'mir100'
    STATUS_PUB_PERIOD = 1.0

    def __init__(self, fleet_config):
        super().__init__('fleet_driver_mir')
        self.fleet_config = fleet_config
        self.robots = {}
        self.api_clients = []
        self.status_pub = self.create_publisher(FleetState, 'fleet_states', 1)
        self.pub_timer = self.create_timer(
            self.STATUS_PUB_PERIOD, self.pub_fleet
        )
        self.ref_coordinates_rmf = [[26.95, -20.23], [29.26, -22.38], [11.4, -16.48],
                                    [12.46, -16.99]]
        self.ref_coordinates_mir = [[7.2, 16.6], [5.15, 18.35], [23, 12.35],
                                    [22.05, 12.95]]
        self.rmf2mir_transform = nudged.estimate(
            self.ref_coordinates_rmf,
            self.ref_coordinates_mir
        )
        self.mir2rmf_transform = nudged.estimate(
            self.ref_coordinates_mir,
            self.ref_coordinates_rmf
        )
        mse = nudged.estimate_error(self.rmf2mir_transform,
                                    self.ref_coordinates_rmf,
                                    self.ref_coordinates_mir)
        self.get_logger().info(f'transformation estimate error: {mse}')

        for api_client in self.create_all_api_clients(self.fleet_config):
            self.get_logger().info(f'initializing robot from \
                                   {api_client.configuration.host}')
            robot = Robot(self)
            robot.api = mir100_client.DefaultApi(api_client)

            # temporary retry configuration to workaround startup race condition while launching
            connection_pool_kw = robot.api.api_client.rest_client.pool_manager.connection_pool_kw
            orig_retries = connection_pool_kw.get('retries')
            retries = urllib3.Retry(10)
            retries.backoff_factor = 1
            retries.status_forcelist = (404,)
            connection_pool_kw['retries'] = retries

            mir_status = robot.api.status_get()
            robot.name = mir_status.robot_name

            self.load_missions(robot)
            self.update_positions(robot)
            # reset retries
            if orig_retries is not None:
                connection_pool_kw['retries'] = orig_retries
            else:
                del connection_pool_kw['retries']

            self.robots[robot.name] = robot
            self.get_logger().info(f'successfully initialized robot \
                                   {robot.name}')

        # Setup fleet driver ROS2 topic subscriptions
        self.path_request_sub = self.create_subscription(
            PathRequest, '/robot_path_requests', self.on_path_request, 1
        )
        self.mode_sub = self.create_subscription(
            ModeRequest, f'/robot_mode_requests', self.on_robot_mode_request, 1
        )

    def pub_fleet(self):
        fleet_state = FleetState()
        fleet_state.name = self.FLEET_NAME
        now = time.time()
        now_sec = int(now)
        now_ns = int((now - now_sec) * 1e9)

        try:
            for robot in self.robots.values():
                api_response = robot.api.status_get()
                robot_state = RobotState()
                robot_state.name = robot.name
                robot_state.task_id = robot.current_task_id
                robot_state.battery_percent = api_response.battery_percentage
                location = Location()
                location.x = api_response.position.x
                location.y = api_response.position.y
                location.yaw = api_response.position.orientation
                # TODO Transform yaw from MiR frame to RMF frame
                mir_pos = [location.x, location.y]
                rmf_pos = self.mir2rmf_transform.transform(mir_pos)
                rmf_location = Location()
                rmf_location.x = rmf_pos[0]
                rmf_location.y = rmf_pos[1]
                rmf_location.yaw = math.radians(location.yaw) + self.mir2rmf_transform.get_rotation()
                robot_state.location = rmf_location
                robot_state.path = robot.remaining_path
                robot_state.location.t.sec = now_sec
                robot_state.location.t.nanosec = now_ns

                if api_response.mission_text.startswith('Charging'):
                    robot_state.mode.mode = RobotMode.MODE_CHARGING
                    robot.mode = MirState.READY
                elif api_response.state_id == MirState.PAUSE:
                    robot_state.mode.mode = RobotMode.MODE_PAUSED
                    robot.mode = MirState.PAUSE
                elif api_response.state_id == MirState.EXECUTING and \
                    not api_response.mission_text.startswith('Charging'):
                    robot_state.mode.mode = RobotMode.MODE_MOVING
                    robot.mode = MirState.EXECUTING
                elif api_response.state_id == MirState.READY:
                    robot_state.mode.mode = RobotMode.MODE_IDLE
                    robot.mode = MirState.READY

                # print(f'[{api_response.state_id}] [{api_response.state_text}] [{api_response.mission_text}]')
                fleet_state.robots.append(robot_state)

                if robot.docking_requested:
                    if not robot.docking_executed:
                        robot.docking_executed = ('docking' in api_response.mission_text.lower())

                    if robot.docking_executed and api_response.state_id == MirState.READY:
                        robot_state.mode.mode = RobotMode.MODE_IDLE
                    else:
                        robot_state.mode.mode = RobotMode.MODE_DOCKING

            self.status_pub.publish(fleet_state)

        except ApiException as e:
            self.get_logger().warn('Exception when calling \
                                   DefaultApi->status_get: %s\n' %e)

    def on_robot_mode_request(self, msg):
        robot = self.robots.get(msg.robot_name)
        if not robot:
            self.get_logger().info(f'Could not find a robot named [{msg.robot_name}]')
            return

        if robot.current_task_id == msg.task_id:
            self.get_logger().info(f'Already following task [{msg.task_id}]')
            return


        robot.cancel_path()

        # Mapping from RMF modes to MiR modes
        # manual control is MiR mode 11
        mir_mode_request_dict = {
            RobotMode.MODE_MOVING: MirState.READY,
            RobotMode.MODE_PAUSED: MirState.PAUSE
        }

        desired_mir_mode = mir_mode_request_dict.get(msg.mode.mode)
        if desired_mir_mode:
            self.get_logger().info(f'setting robot {msg.robot_name} mode to {msg.mode}')
            status = PutStatus(state_id=desired_mir_mode)
            robot.api.status_put(status)
            return

        if not msg.parameters:
            self.get_logger().info(
                f'Mode [{msg.mode.mode}] not recognized or requires additional parameters'
            )
            return


        # Find the mission
        mission_str = f'{msg.parameters[0].name}_{msg.parameters[0].value}'
        self.get_logger().info(f'Attempting to send mission [{mission_str}] to robot [{msg.robot_name}]')
        try:
            mission_id = robot.missions[mission_str].guid
        except KeyError:
            self.get_logger().error(f'Cannot find mission [{mission_str}]')
            return

        # Execute the mission
        try:
            mission = PostMissionQueues(mission_id = mission_id)
            robot.api.mission_queue_post(mission)
        except KeyError:
            self.get_logger().error('Error when posting charging mission')
            return

        if msg.parameters[0].name == 'docking':
            robot.docking_requested = True
            robot.docking_executed = False
            print(' === We are in docking mode')


        robot.current_task_id = msg.task_id

    def calculate_path_request_yaw(self, location_request, location_request_next):
        dx = location_request_next.x - location_request.x
        dy = location_request_next.y - location_request.y
        return math.atan2(dy, dx)

    def on_path_request(self, msg):
        robot = self.robots.get(msg.robot_name)

        if not robot:
            self.get_logger().info(f'Could not find robot with the name [{msg.robot_name}]')
            return

        if robot.current_task_id == msg.task_id:
            self.get_logger().info(f'Already received task [{msg.task_id}].')
            return

        self.get_logger().info(f'Issuing task [{msg.task_id}] to robot [{msg.robot_name}]')
        robot.follow_new_path(msg)

    def load_missions(self, robot):
        self.get_logger().info('retrieving missions...')
        robot_missions_ls = robot.api.missions_get()
        for i in robot_missions_ls:
            if i.name not in robot.missions:
                robot.missions[i.name]=i
            else:
                if "move_coordinate" in i.name:
                    print("removing {}".format(i.name))
                    robot.api.missions_guid_delete(i.guid)

        self.get_logger().info(f'retrieved {len(robot.missions)} missions')

    def create_move_coordinate_mission(self, robot, location, retries=10):
        mission = PostMissions(
            group_id='mirconst-guid-0000-0001-missiongroup',
            name=f'move_coordinate_to_{location.x:.3f}_{location.y:.3f}',
            description='automatically created by mir fleet adapter',
        )
        response = robot.api.missions_post(mission)
        action = PostMissionActions(
            action_type='move_to_position',
            mission_id=response.guid,
            parameters=[
                {'id': 'x', 'value': location.x},
                {'id': 'y', 'value': location.y},
                {'id': 'orientation', 'value': location.yaw},
                {'id': 'retries', 'value': retries},
                {'id': 'distance_threshold', 'value': 0.1},
            ],
            priority = 1
        )
        response2 = robot.api.missions_mission_id_actions_post(
            mission_id = response.guid,
            body = action
        )
        self.get_logger().info(f'created mission to move coordinate to "{location}"')
        return response.guid

    def create_dock_mission(self, robot, dock_name):
        mission = PostMissions(
            # mir const, retrieved with GET /mission_groups
            group_id='mirconst-guid-0000-0001-missiongroup',
            name=f'dock_to_{dock_name}',
            description='automatically created by mir fleet adapter',
        )
        response = robot.api.missions_post(mission)
        action = PostMissionActions(
            action_type='docking',
            mission_id=response.guid,
            parameters=[
                {'id': 'marker', 'value': dock_name},
            ],
            priority=1
        )
        response2 = robot.api.missions_mission_id_actions_post(
            mission_id=response.guid,
            body=action
        )
        self.get_logger().info(f'created mission to move to "{dock_name}"')
        return response.guid

    def create_move_mission(self, robot, place_name, retries=10):
        '''
        creates a mission to move to metamap place
        '''
        mission = PostMissions(
            # mir const, retrieved with GET /mission_groups
            group_id='mirconst-guid-0000-0001-missiongroup',
            name=f'move_to_{place_name}',
            description='automatically created by mir fleet adapter',
        )
        response = robot.api.missions_post(mission)
        dist_threshold = 0.1
        action = PostMissionActions(
            action_type='move',
            mission_id=response.guid,
            parameters=[
                {'id': 'position', 'value': robot.positions[place_name].guid},
                {'id': 'retries', 'value': retries},
                {'id': 'distance_threshold', 'value': dist_threshold},
            ],
            priority=1
        )
        response2 = robot.api.missions_mission_id_actions_post(
            mission_id=response.guid,
            body=action
        )
        self.get_logger().info(f'created mission to move to "{place_name}"')
        return response.guid

    def update_positions(self, robot):
        self.get_logger().info('retrieving positions...')
        count = 0
        for pos in robot.api.positions_get():
            if pos.name not in robot.positions or pos.guid != robot.positions[pos.name].guid:
                if pos.type_id == MirPositionTypes.ROBOT or \
                        pos.type_id == MirPositionTypes.CHARGING_STATION_ENTRY:
                    robot.positions[pos.name] = robot.api.positions_guid_get(pos.guid)
                    count += 1
        self.get_logger().info(f'updated {count} positions')

    def create_all_api_clients(self, config):
        # self.api_clients = []
        for i in range(len(config['robots'])):
            self.api_clients.append(self.create_single_api_client(config, i))
        return self.api_clients

    def create_single_api_client(self, config, idx):
        robot_config = config['robots'][idx]
        configuration = mir100_client.Configuration()
        configuration.host = robot_config['base_url']
        configuration.username = robot_config['user']
        configuration.password = robot_config['password']
        api_client = mir100_client.ApiClient(configuration)
        api_client.default_headers['Accept-Language'] = 'en-US'
        return api_client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fleet_config_file", nargs=1)
    args = parser.parse_args()

    with open(args.fleet_config_file[0], 'r') as f:
        fleet_config = json.load(f)

    rclpy.init()
    node = FleetDriverMir(fleet_config)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
