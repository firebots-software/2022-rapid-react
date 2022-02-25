package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooter.LaunchBall;
import frc.robot.commands.shooter.SpinUpShooter;

public class SpinWheelAndRunRoller extends ParallelCommandGroup {
    
    public SpinWheelAndRunRoller() {
        addCommands(
                new SpinUpShooter(),
                new LaunchBall()
        );
    }

}
