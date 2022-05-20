package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class JoystickArcadeDrive implements Command {
    private final Drivetrain drivetrain;
    private final DoubleSupplier frontBackSpeed;
    private final DoubleSupplier rotationSpeed;

  /**
   * Teleop joystick driving using normal arcade drive.
   *  
   * @param frontBackSpeed = joystick axis for front-back movement
   * @param rotationSpeed = joystick axis for rotation
   */
    public JoystickArcadeDrive(DoubleSupplier frontBackSpeed, DoubleSupplier rotationSpeed) {
        this.drivetrain = Drivetrain.getInstance();
        this.frontBackSpeed = frontBackSpeed;
        this.rotationSpeed = rotationSpeed;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(-frontBackSpeed.getAsDouble(), -rotationSpeed.getAsDouble()); // todo: figure out why we need negation
    }

  
    @Override
    public boolean isFinished() {
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
