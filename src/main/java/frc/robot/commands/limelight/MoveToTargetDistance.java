// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class MoveToTargetDistance extends CommandBase {
  private Limelight limelight; 
  private Drivetrain drivetrain; 
  private PIDController pid; 
  private double tempTargetMeters; 

  /** Creates a new MoveToTargetDistance. */
  public MoveToTargetDistance() {
    limelight = Limelight.getInstance(); 
    drivetrain = Drivetrain.getInstance();
    pid = new PIDController(Constants.Drivetrain.driveP, Constants.Drivetrain.driveI, Constants.Drivetrain.driveD); 
    tempTargetMeters = 2; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.refreshValues();
    drivetrain.resetEncoders();
    // pid.setSetpoint(limelight.getDistanceToTarget());
    pid.setSetpoint(tempTargetMeters);
    pid.setTolerance(Constants.Turret.pidPositionToleranceDegrees, Constants.Turret.pidVelToleranceDegPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = pid.calculate(drivetrain.getAvgEncoderCountMeters()); 
    drivetrain.PIDarcadeDrive(pidOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
