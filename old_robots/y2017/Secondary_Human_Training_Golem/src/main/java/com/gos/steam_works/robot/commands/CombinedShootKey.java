package com.gos.steam_works.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.gos.steam_works.robot.subsystems.Agitator;
import com.gos.steam_works.robot.subsystems.Loader;
import com.gos.steam_works.robot.subsystems.Shooter;

/**
 *
 */
public class CombinedShootKey extends SequentialCommandGroup {

    public CombinedShootKey(Agitator agitator, Shooter shooter, Loader loader) {
        addParallel(new Shoot(shooter, Shooter.AUTO_SHOOTER_SPEED_KEY));
        addCommands(new TimeDelay(0.75));
        addParallel(new Agitate(agitator));
        addCommands(new LoadBall(loader));
    }
}
