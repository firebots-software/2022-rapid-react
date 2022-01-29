// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends PIDCommand {

  private Drivetrain drivetrain;

  /** Creates a new DriveForDistance. */
  public DriveForDistance(Drivetrain drivetrain, double targetMeters) {
    super(
        new PIDController(Constants.Drivetrain.driveP, Constants.Drivetrain.driveI, Constants.Drivetrain.driveD),

        drivetrain::getAvgEncoderVal,

        targetMeters,

        output -> {
          drivetrain.PIDarcadeDrive(output, 0);
          SmartDashboard.putNumber("pid output", output);

          SmartDashboard.putNumber("Right encoder count meters", drivetrain.getRightEncoderCountMeters());
          SmartDashboard.putNumber("Right encoder velocity", drivetrain.getRightEncoderVelocityMetersPerSec());

          SmartDashboard.putNumber("Left encoder count meters", drivetrain.getLeftEncoderCountMeters());
          SmartDashboard.putNumber("Left encoder velocity", drivetrain.getLeftEncoderVelocityMetersPerSec());
        },
        drivetrain);
    System.out.println("pid command constructor");

    this.drivetrain = drivetrain;
    drivetrain.resetEncoders();
    getController().setTolerance(Constants.Drivetrain.distanceToleranceMeters,
        Constants.Drivetrain.velocityToleranceMetersPerSec);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    super.initialize();
    System.out.println("got into pid command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("isFinished");
    return getController().atSetpoint();
  }

  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
