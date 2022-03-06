package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.AlignToTarget;
import frc.robot.commands.shooter.SpinUpShooterNoStop;
import frc.robot.commands.shooter.StopShooter;

public class AimAndShoot extends SequentialCommandGroup  {
    
    public AimAndShoot() {
        addCommands(
            new SpinUpShooterNoStop(),
            new AlignToTarget(),
            new RunSpaghetAndRoll().withTimeout(7),
            new StopShooter()
        );
    }
}
