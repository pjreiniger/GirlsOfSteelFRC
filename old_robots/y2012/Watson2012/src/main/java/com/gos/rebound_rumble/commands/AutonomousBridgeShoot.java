package com.gos.rebound_rumble.commands;

//moves to bridge, shoots, pushes bridge down

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.gos.rebound_rumble.OI;
import com.gos.rebound_rumble.objects.Camera;
import com.gos.rebound_rumble.subsystems.Chassis;
import com.gos.rebound_rumble.subsystems.Shooter;


public class AutonomousBridgeShoot extends SequentialCommandGroup {

    public AutonomousBridgeShoot(OI oi, Chassis chassis, Shooter shooter) {
        if (Camera.foundTarget()) {
            double distance = Chassis.DISTANCE_BACKBOARD_TO_BRIDGE
                - Camera.getXDistance();
            addCommands(new MoveToSetPoint(chassis, distance), 3.0);
        } else {
            addCommands(new MoveToSetPoint(chassis, Chassis.DISTANCE_KEY_TO_BRIDGE), 3.0);
        }
        if (Camera.foundTarget()) {
            addParallel(new ShootUsingTable(shooter, oi, false));
        } else {
            addParallel(new Shoot(shooter, oi, Shooter.BRIDGE_SPEED));
        }
    }

}
