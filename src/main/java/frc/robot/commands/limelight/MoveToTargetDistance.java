// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class MoveToTargetDistance extends CommandBase {
  private Limelight limelight; 
  private Drivetrain drivetrain; 
  private PIDController pid; 

  /** Creates a new MoveToTargetDistance. */
  public MoveToTargetDistance() {
    limelight = Limelight.getInstance(); 
    drivetrain = Drivetrain.getInstance();
    pid = new PIDController(kp, ki, kd); 
    pid.setSetpoint(getDistanceToTarget());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.refreshValues();
    drivetrain.resetEncoders();
  }

  public double getDistanceToTarget() {
    double limelightYOffset = limelight.getTx(); 
    double ratio = Math.tan(limelightYOffset + Constants.Limelight.limelightAngleOffset); 
    return ratio * Constants.Limelight.heightOfTarget; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.PIDarcadeDrive(pid.calculate(drivetrain.getAverageEncoderPosition()), 0);
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
