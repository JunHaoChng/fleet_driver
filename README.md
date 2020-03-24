# fleet_driver_mir
## Requirements

This fleet driver requires the following packages

|Packages|
|---|
|[Mir100-client](https://github.com/osrf/mir100-client)|
|[rmf_core](https://github.com/osrf/rmf_core)|

## Instructions 

The fleet driver subscribes to ros topics using the message types:
`ModeRequest.msg`
`PathRequest.msg`

and gives commands to the mir fleet.

Each mir requires login details that are to be described in a json. (one is provided in fleet_driver_mir/; it is used as in the example)

To run the fleet driver, use the following command:

`ros2 run fleet_driver_mir fleet_driver_mir fleet_driver_mir/mir_fleet_config_cgh_chart.json`

## Utilities

Utilities were made in python.
|Utility|Use|
|---|---|
|remove_move_coordinate.py|Removes the move_coordinate missions created by the driver from the MIR.|
|report_positions_in_rmf_coord.py|Saves all positions on the MIR into a json file.|
|jsontocsv.py|Converts json into a csv for easy viewing|

