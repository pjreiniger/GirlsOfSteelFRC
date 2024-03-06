
import os
import json
import math
import shutil
import re


def load_path(path_filename, waypoints, indent):
    with open(path_filename, 'r') as f:
        path_data = json.load(f)

    path_waypoints = path_data["waypoints"]

    if waypoints:
        if path_waypoints[0]['anchor']["x"] == waypoints[-1]["x"] and path_waypoints[0]['anchor']["y"] == waypoints[-1]["y"]:
            path_waypoints.pop(0)
        else:
            print(f"{indent}The don't match!")
            raise

    for path_waypoint in path_waypoints:
        print(f"{indent}{path_waypoint}")

        point_x = path_waypoint['anchor']['x']
        point_y = path_waypoint['anchor']['y']

        heading = 0
        if path_waypoint['nextControl']:
            next_control = path_waypoint['nextControl']
            control_x = next_control["x"]
            control_y = next_control['y']
            heading = math.atan2(control_y - point_y, control_x - point_x)

        if heading == 0 and path_waypoint['prevControl']:
            next_control = path_waypoint['prevControl']
            control_x = next_control["x"]
            control_y = next_control['y']
            heading = math.atan2(control_y - point_y, control_x - point_x)

        waypoints.append({
            "x": point_x,
            "y": point_y,
            "heading": heading,
            "isInitialGuess": False,
            "translationConstrained": True,
            "headingConstrained": True,
            "controlIntervalCount": 200
        })

    goalEndState = path_data["goalEndState"]
    waypoints[-1]["heading"] = math.radians(goalEndState["rotation"])

    return waypoints


def __handle_command(pp_dir, command, waypoints, indent):
    command_type = command["type"]

    if command_type in ["named"]:
        return []
    if command_type == "deadline":
        print(f"{indent}Deadline Command:")
        for subcommand in command["data"]["commands"]:
            __handle_command(pp_dir, subcommand, waypoints, indent + "  ")

        return waypoints
    if command_type == "race":
        print(f"{indent}Race Command:")
        for subcommand in command["data"]["commands"]:
            __handle_command(pp_dir, subcommand, waypoints, indent + "  ")

        return waypoints

    if command_type != "path":
        raise Exception(command_type)

    path_file = os.path.join(pp_dir, "paths", command['data']['pathName'] + ".path")
    print(f"{indent}Loading path {path_file}")
    load_path(path_file, waypoints, indent + "  ")


def load_auto(pp_dir, auto_data):
    path = {}
    waypoints = []

    for command in auto_data["command"]["data"]["commands"]:
        __handle_command(pp_dir, command, waypoints, "  ")

    path["waypoints"] = waypoints
    path["trajectory"] = []
    path["trajectoryWaypoints"] = []
    path["constraints"] = [
        {
            "scope": ["first"],
            "type": "StopPoint"
        },
        {
            "scope": ["last"],
            "type": "StopPoint"
        },
    ]

    for i in range(1, len(waypoints) - 1):
        path["constraints"].append(dict(scope=[i], type="StopPoint"))

    for i in range(1, len(waypoints)):
        path['constraints'].append(
            {
                "scope": [
                    i - 1,
                    i
                ],
                "type": "MaxVelocity",
                "velocity": 1
            })

    path["usesControlIntervalGuessing"] =  False
    path["defaultControlIntervalCount"] =  200
    path["usesDefaultFieldObstacles"] =  True
    path["circleObstacles"] =  []
    path["eventMarkers"] = []
    path["isTrajectoryStale"] = True

    return path


def should_keep_auto(auto_data, f):
    folder = auto_data["folder"]
    return folder != "TwoPieceExperimental" and f != "JustShoot.auto" and not auto_data['choreoAuto']


def load_autos(pp_dir):
    paths = {}

    for root, _, files in os.walk(os.path.join(pp_dir, "autos")):
        for f in files:
            full_file = os.path.join(root, f)
            with open(full_file, 'r') as ifs:
                auto_data = json.load(ifs)

            if should_keep_auto(auto_data, f):
                print(f"Loading {full_file}")
                base = os.path.basename(f)
                maybe_path = load_auto(pp_dir, auto_data)
                if maybe_path:
                    paths[os.path.splitext(base)[0]] = maybe_path
            else:
                print(f"Skipping {full_file}")


    return paths


def make_choreo_autos(pp_dir, choreo_config):

    # def __handle_command2(command, cmd_ctr, command_config):
    #     print(command)
    #     return cmd_ctr
    #
    for root, _, files in os.walk(os.path.join(pp_dir, "autos")):
        for f in files:
            full_file = os.path.join(root, f)
            with open(full_file, 'r') as ifs:
                auto_data = json.load(ifs)

            if should_keep_auto(auto_data, f):
                base = os.path.splitext(os.path.basename(f))[0]
                choreo_file = os.path.join(root, base + "Choreo.auto")
                shutil.copy(full_file, choreo_file)

                with open(choreo_file, 'r') as ff:
                    contents = ff.read()

                contents = contents.replace('"choreoAuto": false', '"choreoAuto": true')
                contents = re.sub('"folder": ".*"', '"folder": null', contents)

                prev_contents = None
                path_ctr = 1
                while contents != prev_contents:
                    prev_contents = contents
                    contents = re.sub('"pathName": .*', f'"pathNameHACK": "{base}.{path_ctr}"', contents, count=1)
                    path_ctr += 1
                    print(f"RAN A REPLA {path_ctr}")


                contents = contents.replace('pathNameHACK', 'pathName')
                # start_idx = 0
                # print(re.search('"pathName": .*', contents))

                with open(choreo_file, 'w') as ff:
                    ff.write(contents)

    #
    #             base = os.path.splitext(os.path.basename(f))[0]
    #
    #             config = {}
    #             config['version'] = 1.0
    #
    #             starting_pose = {}
    #             starting_pose["position"] = dict(x=0, y=0)
    #             starting_pose["rotation"] = 0
    #
    #             config['startingPose'] = starting_pose
    #             config["command"] = {}
    #
    #             cmd_ctr = 0
    #             for command in auto_data["command"]["data"]["commands"]:
    #                 cmd_ctr = __handle_command2(command, cmd_ctr, config["command"])
    #
    #             config['folder'] = None
    #             config['choreoAuto'] = True
    #
    #             with open(os.path.join(root, base + "Choreo.auto"), 'w') as f:
    #                 json.dump(config, f, indent=2)



def main():
    crescendo_dir = os.path.dirname(os.path.realpath(__file__))
    pp_dir = os.path.join(crescendo_dir, "src", "main", "deploy", "pathplanner")
    choreo_config = os.path.join("ChoreoAutos.chor")

    paths = load_autos(pp_dir)

    config = {}

    config["version"] = "v0.3"

    robot_configuration = {}
    robot_configuration["mass"] =  74.08797700309194
    robot_configuration["rotationalInertia"] =  8.259589028886236
    robot_configuration["motorMaxTorque"] =  1.162295081967213
    robot_configuration["motorMaxVelocity"] =  4800
    robot_configuration["gearing"] =  4.71
    robot_configuration["wheelbase"] =  0.6349996571001851
    robot_configuration["trackWidth"] =  0.6349996571001851
    robot_configuration["bumperLength"] =  0.8762995267982555
    robot_configuration["bumperWidth"] =  0.8762995267982555
    robot_configuration["wheelRadius"] =  0.03809997942601111
    config["robotConfiguration"] = robot_configuration
    config["paths"] = paths
    config["splitTrajectoriesAtStopPoints"] = True
    config["usesObstacles"] = False

    with open(choreo_config, 'w') as f:
        json.dump(config, f, indent=4)


    make_choreo_autos(pp_dir, config)




if __name__ == "__main__":
    main()