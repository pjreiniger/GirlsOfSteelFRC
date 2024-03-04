
import json


def load_paths(pp_autos_dir):
    paths = {}

    path = {}
    path["waypoints"] = [{
        "x":  1.0267819166183472,
        "y": 4.359738826751709,
        "heading": 0,
        "isInitialGuess": False,
        "translationConstrained": True,
        "headingConstrained": True,
        "controlIntervalCount": 40,
    },{
        "x":  7.480413436889648,
        "y": 7.521584987640381,
        "heading": 0,
        "isInitialGuess": False,
        "translationConstrained": True,
        "headingConstrained": True,
        "controlIntervalCount": 40,
    }]
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

    paths["New Path"] = path

    return paths

def main():
    pp_autos_dir = r'C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\src\main\deploy\pathplanner\autos'
    choreo_config = r"C:\Users\PJ\git\gos\GirlsOfSteelFRC\y2024\Crescendo\ChoreoAutos.chor"

    paths = load_paths(pp_autos_dir)

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