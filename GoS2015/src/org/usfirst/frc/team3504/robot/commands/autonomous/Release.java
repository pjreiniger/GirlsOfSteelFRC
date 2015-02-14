package org.usfirst.frc.team3504.robot.commands.autonomous;

import org.usfirst.frc.team3504.robot.commands.collector.AngleCollectorOut;
import org.usfirst.frc.team3504.robot.commands.doors.DoorsOut;
import org.usfirst.frc.team3504.robot.commands.fingers.FingerDown;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Release extends CommandGroup {
    
    public  Release() {
    	addSequential(new AutoReleaseGripper());
    	addSequential(new AngleCollectorOut());
    	addSequential(new FingerDown());
    	//addSequential(new AutoLift());
    	//addSequential(new AutoLowerLifter());
    	//addSequential(new AutoBringInGrippers());
    	addSequential(new DoorsOut());
    	//addSequential(new AutoReleaseGripper());
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
