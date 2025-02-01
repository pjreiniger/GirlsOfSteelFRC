
import json
import pathlib
from jinja2 import Template

def load_choreo_variables(choreo_file: pathlib.Path):
    data = json.loads(choreo_file.read_text())
    variables = {}
    for var_name, var in data['variables']['poses'].items():
        print(var_name)
        variables[var_name] = dict(name=var_name, x=var['x']['val'], y=var['y']['val'], heading=var['heading']['val'])

    return variables


def create_path(choreo_dir, variables, first_variable, second_variable):
    filename = f"{first_variable}To{second_variable}"

    contents = Template(TRAJECTORY_TEMPLATE).render(
        name = filename,
        first_pose = variables[first_variable],
        second_pose = variables[second_variable],
    )

    path_to_write = choreo_dir / f"{filename}.traj"
    path_to_write.write_text(contents)


def main():
    root_dir = pathlib.Path(r".")
    choreo_dir = root_dir / r"y2025\Reefscape\src\main\deploy\choreo"
    variables = load_choreo_variables(choreo_dir / r"ChoreoAutos.chor")

    for reef_position in ["C"]:
        starting_position = "HumanPlayerRight"
        create_path(choreo_dir, variables, starting_position, reef_position)

    for reef_position in ["E"]:
        starting_position = "HumanPlayerRight"
        create_path(choreo_dir, variables, reef_position, starting_position)

    for reef_position in ["L"]:
        starting_position = "HumanPlayerLeft"
        create_path(choreo_dir, variables, starting_position, reef_position)

    for reef_position in ["J"]:
        starting_position = "HumanPlayerLeft"
        create_path(choreo_dir, variables, reef_position, starting_position)

    for algae_position in ["EF"]:
        create_path(choreo_dir, variables, algae_position, "Processor")
        create_path(choreo_dir, variables, "Processor", algae_position)

    for algae_position in ["GH"]:
        create_path(choreo_dir, variables, algae_position, "Processor")


    for reef_position in ["H"]:
        starting_position = "StartingPosCenter"
        create_path(choreo_dir, variables, starting_position, reef_position)

    for reef_position in ["J"]:
        starting_position = "StartingPosLeft"
        create_path(choreo_dir, variables, starting_position, reef_position)

    for reef_position in ["E"]:
        starting_position = "StartingPosRight"
        create_path(choreo_dir, variables, starting_position, reef_position)


TRAJECTORY_TEMPLATE = """{
 "name":"{{ name }}",
 "version":1,
 "snapshot":{
  "waypoints":[],
  "constraints":[],
  "targetDt":0.05
 },
 "params":{
  "waypoints":[
    {"x":{"exp":"{{ first_pose.name }}.x", "val":{{ first_pose.x }}}, "y":{"exp":"{{ first_pose.name }}.y", "val":{{ first_pose.y }}}, "heading":{"exp":"{{ first_pose.name }}.heading", "val":{{ first_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false},
    {"x":{"exp":"{{ second_pose.name }}.x", "val":{{ second_pose.x }}}, "y":{"exp":"{{ second_pose.name }}.y", "val":{{ second_pose.y }}}, "heading":{"exp":"{{ second_pose.name }}.heading", "val":{{ second_pose.heading }}}, "intervals":40, "split":false, "fixTranslation":true, "fixHeading":true, "overrideIntervals":false}],
  "constraints":[
    {"from":"first", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":"last", "to":null, "data":{"type":"StopPoint", "props":{}}, "enabled":true},
    {"from":0, "to":1, "data":{"type":"MaxVelocity", "props":{"max":{"exp":"DefaultMaxVelocity", "val":1.524}}}, "enabled":true}],
  "targetDt":{
   "exp":"0.05 s",
   "val":0.05
  }
 },
 "trajectory":{
  "sampleType":null,
  "waypoints":[],
  "samples":[],
  "splits":[]
 },
 "events":[]
}

"""

if __name__ == "__main__":
    main()