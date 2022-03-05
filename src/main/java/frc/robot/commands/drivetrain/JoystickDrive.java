package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class JoystickDrive implements Command {
    private final Drivetrain drivetrain;
    private final DoubleSupplier frontBackSpeed;
    private final DoubleSupplier rotationSpeed;


    public JoystickDrive(DoubleSupplier frontBackSpeed, DoubleSupplier rotationSpeed) {
        this.drivetrain = Drivetrain.getInstance();
        this.frontBackSpeed = frontBackSpeed;
        this.rotationSpeed = rotationSpeed;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(-frontBackSpeed.getAsDouble(), rotationSpeed.getAsDouble());

    }

  
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
