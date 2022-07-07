package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.util.Collections;
import java.util.Set;

public class ToggleSlowMode extends CommandBase {

    private final Drivetrain drivetrain;

    public ToggleSlowMode() {
        this.drivetrain = Drivetrain.getInstance();
    }

    @Override
    // runs constantly until isFinished condition is met
    public void execute() {
        drivetrain.setSlowMode(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSlowMode(false);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
        // do not require drivetrain
    }
}