package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.AlignToTarget;
import frc.robot.commands.shooter.SpinUpShooter;

public class AimAndShoot extends SequentialCommandGroup  {
    
    public AimAndShoot() {
        addCommands(
            new AlignToTarget(),
            new SpinUpShooter(),
            new SpinWheelAndRunRoller()
        );
    }
}
