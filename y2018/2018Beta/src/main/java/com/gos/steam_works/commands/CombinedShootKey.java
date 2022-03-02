package com.gos.steam_works.commands;

import com.gos.steam_works.subsystems.Agitator;
import com.gos.steam_works.subsystems.Loader;
import com.gos.steam_works.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 *
 */
public class CombinedShootKey extends SequentialCommandGroup {

    public CombinedShootKey(Loader loader, Shooter shooter, Agitator agitator) {
        addParallel(new Shoot(shooter, Shooter.AUTO_SHOOTER_SPEED_KEY));
        addCommands(new TimeDelay(0.75));
        addParallel(new Agitate(agitator));
        addCommands(new LoadBall(loader));
    }
}
