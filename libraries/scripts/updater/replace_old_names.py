"""
Runs a regex replace on any of the class names that got changed in the past year updates of wpilib / vendor deps
"""
import json
from libraries.scripts.updater.utils import (
    PINNED_VSCODE_WPILIB_COMMITISH,
    walk_for_extension,
    regex_replace_file,
    auto_retry_download,
)
from libraries.scripts.git.git_python_wrappers import commit_all_changes


def __run_replacement(replacements, root=".", dir_blacklist=None):
    java_files = walk_for_extension(root, "java", dir_blacklist=dir_blacklist)

    for java_file in java_files:
        if "y2013" in java_file:
            continue
        regex_replace_file(java_file, replacements)


def run_standard_replacement(auto_commit):
    # Last sync Dec 19, 2021
    wpilib_replacements_url = f"https://raw.githubusercontent.com/wpilibsuite/vscode-wpilib/{PINNED_VSCODE_WPILIB_COMMITISH}/vscode-wpilib/resources/java_replacements.json"

    raw_json_data = auto_retry_download(wpilib_replacements_url).decode("utf-8")
    json_data = json.loads(raw_json_data)

    replacements = []
    for replacement_json in json_data[0]["replacements"]:
        replacement_to = replacement_json["to"]
        # Python-ize the replacement substitution
        replacement_to = replacement_to.replace("$1", r"\1").replace("$2", r"\2")
        if "$" in replacement_to:
            raise Exception(f"Make this smarter. To = '{replacement_to}")
        replacements.append((replacement_json["from"], replacement_to))

    # Run these on all the files
    __run_replacement(replacements)

    if auto_commit:
        commit_all_changes("Auto-Update: Ran standard vscode replacements")


def run_our_additional_replacements(auto_commit):
    replacements = []

    # Put our smarter-than-wpilib replacements here
    # fmt: off
    replacements.append(("GosCommand", "GosCommandBase"))
    replacements.append(("import com.revrobotics.CANSparkLowLevel;", "import com.revrobotics.spark.SparkLowLevel;"))
    replacements.append(("import com.revrobotics.CANSparkMax;", "import com.revrobotics.spark.SparkMax;"))
    replacements.append(("import com.revrobotics.SparkPIDController;", "import com.revrobotics.spark.SparkClosedLoopController;"))
    replacements.append(("import com.revrobotics.CANSparkBase;", "import com.revrobotics.spark.SparkBase;"))
    replacements.append(("import com.revrobotics.SimableCANSparkFlex;", "import com.revrobotics.spark.SparkFlex;"))
    replacements.append(("import com.revrobotics.SimableCANSparkMax;", "import com.revrobotics.spark.SparkMax;"))


    replacements.append(("CANSparkMax.ControlType", "ControlType"))
    replacements.append(("CANSparkMax.IdleMode", "IdleMode"))

#     import com.revrobotics.SimableCANSparkFlex;
# import com.revrobotics.SimableCANSparkMax;
    replacements.append(("SimableCANSparkMax", "SparkMax"))
    replacements.append(("SimableCANSparkFlex", "SparkFlex"))
    replacements.append(("CANSparkBase", "SparkBase"))
    replacements.append(("CANSparkLowLevel", "SparkBase"))
    # # replacements.append(("SimableSparkMax", "SparkMax"))
    # replacements.append(("CANSparkMax", "SparkMax"))
    # # replacements.append(("CANSparkLowLevel.MotorType.kBrushless", "SparkLowLevel.MotorType.kBrushless"))
    replacements.append(("SparkPIDController", "SparkClosedLoopController"))
    replacements.append(("getPIDController", "getClosedLoopController"))

    replacements.append((".getAbsoluteEncoder\(SparkAbsoluteEncoder.Type.kDutyCycle\)", ".getAbsoluteEncoder()"))
    replacements.append(("AbsEncoder.setInverted", ".absoluteEncoder.inverted"))
    replacements.append(("AbsEncoder.setZeroOffset", ".absoluteEncoder.zeroOffset"))
    replacements.append(("m_(.*).burnFlash\(\)", r"m_\1.configure(\1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)"))

    replacements.append(("m_(.*).setPeriodicFramePeriod\(SparkBase.PeriodicFrame.kStatus5, ", r"\1Config.signals.absoluteEncoderPositionPeriodMs("))
    replacements.append(("m_(.*).setPeriodicFramePeriod\(SparkBase.PeriodicFrame.kStatus6, ", r"\1Config.signals.absoluteEncoderVelocityPeriodMs("))
    #
    replacements.append(("m_(.*).setIdleMode\(IdleMode", r"\1Config.idleMode(IdleMode"))
    replacements.append(("m_(.*).setSmartCurrentLimit", r"\1Config.smartCurrentLimit"))
    replacements.append(("m_(.*).setPositionConversionFactor", r"\1Config._encoder_.positionConversionFactor"))
    replacements.append(("m_(.*).setVelocityConversionFactor", r"\1Config._encoder_.velocityConversionFactor"))
    replacements.append(("m_(.*).enableVoltageCompensation", r"\1Config.voltageCompensation"))
    # replacements.append(("m_(.*).follow\(", r"\1Config.follow("))

    replacements.append((r"SparkBase.MotorType\.", "MotorType."))

    replacements.append(("CANSparkMax", "SparkMax"))


    # fmt: on

    # Run these on all the files
    __run_replacement(replacements)
    # __run_replacement(replacements, root="libraries")
    # __run_replacement(replacements, root="y2024")
    # __run_replacement(replacements, root=r"C:\Users\PJ\git\snobotsim\SnobotSimV2")

    if auto_commit:
        commit_all_changes("Auto-Update: Ran our additional replacements")


def run_all_replacements(auto_commit=True):
    # run_standard_replacement(auto_commit=auto_commit)
    run_our_additional_replacements(auto_commit=auto_commit)


if __name__ == "__main__":
    #  py -m libraries.scripts.updater.replace_old_names
    run_all_replacements(auto_commit=False)
