package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SpinUpShooter;

public class AimAndShoot extends SequentialCommandGroup  {
    private final double SHOOTER_TARGET_SPEED = 0.7; // TODO: Change target speed later


    public AimAndShoot() {
        addCommands(
            // Aim Turret w/ Linelight
            new SpinUpShooter(SHOOTER_TARGET_SPEED),
            new SpinWheelAndRunRoller()
        );
    }
}
