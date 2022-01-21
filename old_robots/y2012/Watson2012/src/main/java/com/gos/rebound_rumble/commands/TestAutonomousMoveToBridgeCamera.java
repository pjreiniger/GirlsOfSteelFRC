package com.gos.rebound_rumble.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import com.gos.rebound_rumble.subsystems.Bridge;
import com.gos.rebound_rumble.subsystems.Chassis;

public class TestAutonomousMoveToBridgeCamera extends CommandGroup {

    public TestAutonomousMoveToBridgeCamera(Chassis chassis, Bridge bridge) {
        requires(chassis);
        //        SmartDashboard.putBoolean("Camera?", false);
        //        SmartDashboard.putNumber("Distc ance from backboard to bridge", 0.0);
        //        SmartDashboard.putNumber("Distance from robot to bridge", 0.0);
        //        if(Camera.isConnected() && SmartDashboard.getBoolean("Camera?", false)){
        //            double distance = SmartDashboard.getDouble("Distance from backboard to bridge",
        //                0.0) - Camera.getXDistance();
        //            addSequential(new MoveToSetPoint(distance),2.0);
        //        }else{
        addSequential(new MoveToSetPoint(chassis, 2.7), 2.0);
        //        }
        addSequential(new AutoBridgeDown(bridge), 2.0);
        addSequential(new MoveToSetPoint(chassis, -0.5), 3.0);
        addSequential(new BridgeUp(bridge));
    }

}