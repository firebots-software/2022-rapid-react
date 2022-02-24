package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.sql.Driver;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurvatureDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier frontBackSpeed;
  private final DoubleSupplier rotationSpeed;

  public CurvatureDrive(DoubleSupplier frontBackSpeed, DoubleSupplier rotationSpeed) {
    this.drivetrain = Drivetrain.getInstance();
    this.frontBackSpeed = frontBackSpeed;
    this.rotationSpeed = rotationSpeed;
  }

  
  @Override
  public void initialize() {}
  
  
  @Override
  public void execute() {
    drivetrain.curvatureDrive(-frontBackSpeed.getAsDouble(), rotationSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
