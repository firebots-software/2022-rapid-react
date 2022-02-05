package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.shooter.SpinUpShooter;

public class AimAndShoot extends SequentialCommandGroup  {
    
    public AimAndShoot() {
        addCommands(
            // Aim Turret w/ Limelight
            new SpinUpShooter(Constants.Shooter.SHOOTER_TARGET_SPEED),
            new SpinWheelAndRunRoller()
        );
    }
}
