# Code attribution from https://github.com/osrf/rmf/blob/master/ros2/fleet_adapter_mir/fleet_adapter_mir/fleet_adapter_mir/

import enum
import math
import time
import argparse
import json
import nudged

import mir100_client
from mir100_client.rest import ApiException
from mir100_client.models import PostMissionQueues, PostMissions, PostMissionActions
import urllib3

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import PathRequest, ModeRequest, RobotState, FleetState, \
    Location, RobotMode


class Robot():
    def __init__(self):
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
    STATUS_PUB_RATE = 0.1

    def __init__(self, fleet_config):
        super().__init__('fleet_driver_mir')
        self.fleet_config = fleet_config
        self.robots = {}
        self.api_clients = []
        self.status_pub = self.create_publisher(FleetState, 'fleet_states')
        self.pub_timer = self.create_timer(
            self.STATUS_PUB_RATE, self.pub_fleet
        )
        self.ref_coordinates_rmf = [[26.95, -20.23], [29.26, -22.38], [11.4, -16.48]]
        self.ref_coordinates_mir = [[7.2, 16.6], [5.15, 18.35], [23, 12.35]]
        self.rmf2mir_transform = nudged.estimate(
            self.ref_coordinates_rmf,
            self.ref_coordinates_mir
        )

        for api_client in self.create_all_api_clients(self.fleet_config):
            self.get_logger().info(f'initializing robot from \
                                   {api_client.configuration.host}')
            robot = Robot()
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
            PathRequest, '/robot_path_requests', self.on_path_request
        )
        self.mode_sub = self.create_subscription(
            ModeRequest, f'/robot_mode_requests', self.on_robot_mode_request
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
                robot_state.battery_percent = api_response.battery_percentage
                location = Location()
                # TODO Transform from MiR frame to RMF frame
                robot_state.location = location

                m = RobotMode()
                if api_response.mission_text.startswith('Charging'):
                    m.mode = RobotMode.MODE_CHARGING
                    robot_state.mode = m
                elif api_response.state_id == MirState.PAUSE:
                    m.mode = RobotMode.MODE_PAUSED
                    robot_state.mode = m
                else:
                    self.get_logger().info(f'status_text: \
                                           {api_response.mission_text}')
                    # raise ValueError('unknown robot mode')
                fleet_state.robots.append(robot_state)

            self.status_pub.publish(fleet_state)

        except ApiException as e:
            self.get_logger().warn('Exception when calling \
                                   DefaultApi->status_get: %s\n' %e)

    def on_robot_mode_request(self, msg):
        pass

    def on_path_request(self, msg):
        # msg.robot_name = 'MiR_R1442'
        self.get_logger().info(f'PathReuest received: \
        sending "{msg.robot_name}" to "{msg.path}"')

        # Abort the current mission and all pending missions
        robot = self.robots[msg.robot_name]
        robot.api.mission_queue_delete()

        # Create each path request and put in mission queue
        index = 0
        for location_request in msg.path:
            # Translate path request from RMF frame to MiR frame
            location_rmf = [location_request.x, location_request.y]
            location_mir = self.rmf2mir_transform.transform(location_rmf)

            location_request_mir = Location()
            location_request_mir.x = location_mir[0]
            location_request_mir.y = location_mir[1]
            location_request_mir.yaw = 180.0
            # Create the corresponding mission
            if index > 1:
                if index == 6:
                    # Call the docking mission directly
                    mission_id = robot.missions['docking_ot'].guid
                else:
                    mission_id = self.create_move_coordinate_mission(
                        robot, location_request_mir
                    )

                # Execute the mission
                try:
                    # mission_id = robot.missions[
                    #     f'move_coordinate_to_{location_request_mir.x:.3f}_{location_request_mir.y:.3f}'
                    # ].guid
                    mission = PostMissionQueues(mission_id=mission_id)
                    robot.api.mission_queue_post(mission)
                except KeyError:
                    self.get_logger().error(
                        f'no mission to move coordinate to \
                        "{location_request_mir.x:.3f}_{location_request_mir.y:.3f}"!')

            index += 1

    def load_missions(self, robot):
        self.get_logger().info('retrieving missions...')
        robot.missions = {m.name: m for m in robot.api.missions_get()}
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
