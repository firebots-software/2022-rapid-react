package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.util.Collections;
import java.util.Set;

public class FlipOrientation implements Command {
    private Drivetrain drivetrain;

    public FlipOrientation() {
        this.drivetrain = Drivetrain.getInstance();
    }

    public void initialize() {
      drivetrain.toggleDriveOrientation();
    }

    @Override
    public void execute() {
       
    }


    @Override
    public boolean isFinished() {
        return true; // don't need to loop, just change value once then end
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
        // do not require drivetrain here
    }
}