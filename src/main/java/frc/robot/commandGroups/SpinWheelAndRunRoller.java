package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.LaunchBall;
import frc.robot.commands.shooter.SpinUpShooter;

public class SpinWheelAndRunRoller extends ParallelCommandGroup {

    private final double SHOOTER_TARGET_SPEED = 0.7; // TODO: Change target speed later

    public SpinWheelAndRunRoller() {
        addCommands(
                new SpinUpShooter(SHOOTER_TARGET_SPEED),
                new LaunchBall()
        );
    }

}
