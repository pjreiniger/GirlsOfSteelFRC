
import os
import json
import math


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


def load_paths(pp_dir):
    paths = {}

    for root, _, files in os.walk(os.path.join(pp_dir, "autos")):
        for f in files:
            full_file = os.path.join(root, f)
            with open(full_file, 'r') as ifs:
                auto_data = json.load(ifs)
            folder = auto_data["folder"]

            if folder != "TwoPieceExperimental" and f != "JustShoot.auto":
                print(f"Loading {full_file}")
                base = os.path.basename(f)
                maybe_path = load_auto(pp_dir, auto_data)
                if maybe_path:
                    paths[os.path.splitext(base)[0]] = maybe_path
            else:
                print(f"Skipping {full_file}")


    return paths

def main():
    pp_dir = r'C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\src\main\deploy\pathplanner'
    choreo_config = r"C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\ChoreoAutos.chor"

    paths = load_paths(pp_dir)

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




if __name__ == "__main__":
    main()