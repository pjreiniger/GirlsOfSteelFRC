
import os
import json


def load_path(path_filename):
    with open(path_filename, 'r') as f:
        path_data = json.load(f)

    output = []

    for waypoint in path_data["waypoints"]:
        output.append({
            "x": waypoint['anchor']['x'],
            "y": waypoint['anchor']['y'],
            "heading": 0,
            "isInitialGuess": False,
            "translationConstrained": True,
            "headingConstrained": True,
            "controlIntervalCount": 40
        })
    return output


def __handle_command(pp_dir, command, indent):
    command_type = command["type"]
    waypoints = []

    if command_type in ["named"]:
        return []
    if command_type == "deadline":
        for subcommand in command["data"]["commands"]:
            waypoints.extend(__handle_command(pp_dir, subcommand, indent + "  "))

        return waypoints
    if command_type == "race":
        for subcommand in command["data"]["commands"]:
            waypoints.extend(__handle_command(pp_dir, subcommand, indent + "  "))

        return waypoints

    if command_type != "path":
        raise Exception(command_type)

    path_file = os.path.join(pp_dir, "paths", command['data']['pathName'] + ".path")
    print(f"{indent}Loading path {path_file}")
    path_waypoints = load_path(path_file)
    waypoints.extend(path_waypoints)

    return waypoints


def load_auto(pp_dir, auto_data):
    path = {}
    waypoints = []

    for command in auto_data["command"]["data"]["commands"]:
        waypoints.extend(__handle_command(pp_dir, command, ""))
        # if command["type"] == "path":

    path["waypoints"] = waypoints
    path["trajectory"] = []
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

    path["usesControlIntervalGuessing"] =  True
    path["defaultControlIntervalCount"] =  40
    path["usesDefaultFieldObstacles"] =  True
    path["circleObstacles"] =  []

    return path


def load_paths(pp_dir):
    paths = {}

    for root, _, files in os.walk(os.path.join(pp_dir, "autos")):
        for f in files:
            full_file = os.path.join(root, f)
            with open(full_file, 'r') as ifs:
                auto_data = json.load(ifs)
            folder = auto_data["folder"]

            if folder == "TwoPiece":
                base = os.path.basename(f)
                paths[os.path.splitext(base)[0]] = load_auto(pp_dir, auto_data)


    return paths

def main():
    pp_dir = r'C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\src\main\deploy\pathplanner'
    choreo_config = r"C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\ChoreoAutos.chor"

    paths = load_paths(pp_dir)

    config = {}

    config["version"] = "v0.2.2"

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
    config["usesObstacles"] = True

    with open(choreo_config, 'w') as f:
        json.dump(config, f, indent=4)




if __name__ == "__main__":
    main()