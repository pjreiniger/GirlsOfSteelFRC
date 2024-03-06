
import os
import json

def main():
    crescendo_dir = os.path.dirname(os.path.realpath(__file__))
    choreo_config = os.path.join(crescendo_dir, "ChoreoAutos.chor")

    with open(choreo_config, 'r') as f:
        json_data = json.load(f)

    # print(json_data)
    for path_name, path_config in json_data["paths"].items():
        path_config["trajectory"] = []
        path_config["trajectoryWaypoints"] = []

    with open(choreo_config, 'w') as f:
        json.dump(json_data, f, indent=4)

if __name__ == "__main__":
    main()