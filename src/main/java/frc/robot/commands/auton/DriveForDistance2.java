// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance2 extends CommandBase {
  private Drivetrain drivetrain;
  private PIDController pidLeft, pidRight;

  /** Creates a new DriveForDistance2. */
  public DriveForDistance2(double targetMeters) {
    drivetrain = Drivetrain.getInstance();
    pidLeft = new PIDController(Constants.Drivetrain.driveP, Constants.Drivetrain.driveI, Constants.Drivetrain.driveD);
    pidLeft.setSetpoint(targetMeters);
    pidLeft.setTolerance(Constants.Drivetrain.distanceToleranceMeters, Constants.Drivetrain.velocityToleranceMetersPerSec);
    

    pidRight = new PIDController(Constants.Drivetrain.driveP, Constants.Drivetrain.driveI, Constants.Drivetrain.driveD);
    pidRight.setSetpoint(targetMeters);
    pidRight.setTolerance(Constants.Drivetrain.distanceToleranceMeters, Constants.Drivetrain.velocityToleranceMetersPerSec);

    // System.out.println("drive for dist constructor");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outputLeft = pidLeft.calculate(drivetrain.getLeftEncoderCountMeters());
    double outputRight = pidRight.calculate(drivetrain.getRightEncoderCountMeters());
    // System.out.print("pid left output: " + outputLeft);
    // System.out.print("pid right output: " + outputRight);
    SmartDashboard.putNumber("Left Position Error: ", pidLeft.getPositionError());
    SmartDashboard.putNumber("Right Position Error: ", pidRight.getPositionError());
    drivetrain.PIDtankDrive(outputLeft, outputRight);

    SmartDashboard.putNumber("pid left output", outputLeft);
    SmartDashboard.putNumber("pid right output", outputRight);

    SmartDashboard.putNumber("Right encoder count meters", drivetrain.getRightEncoderCountMeters());
    SmartDashboard.putNumber("Right encoder velocity", drivetrain.getRightEncoderVelocityMetersPerSec());
    SmartDashboard.putNumber("dfd2 pid error", drivetrain.getRightEncoderCountMeters());
    SmartDashboard.putNumber("dfd2 pid setpoint", drivetrain.getRightEncoderVelocityMetersPerSec());
 
    SmartDashboard.putBoolean("at setpoint", pidLeft.atSetpoint());

    SmartDashboard.putNumber("Average encoder value", drivetrain.getLeftEncoderCountMeters());
    //SmartDashboard.putNumber("Left encoder velocity", drivetrain.getLeftEncoderVelocityMetersPerSec());
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("done with pid loop electric boogaloo");
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pidLeft.atSetpoint() && pidRight.atSetpoint());
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
