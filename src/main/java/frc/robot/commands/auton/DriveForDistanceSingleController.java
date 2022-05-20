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

public class DriveForDistanceSingleController extends CommandBase {
  private Drivetrain drivetrain;
  private PIDController pid;

  /**
   * Drive the robot for a set distance. Uses 1 PID controller that uses the avg left-right encoder value
   * 
   * @param targetMeters = target distance in meters. positive is forward
   */
  public DriveForDistanceSingleController(double targetMeters) {
    drivetrain = Drivetrain.getInstance();
    pid = new PIDController(Constants.Drivetrain.driveP, Constants.Drivetrain.driveI, Constants.Drivetrain.driveD);
    pid.setSetpoint(targetMeters);
    pid.setTolerance(Constants.Drivetrain.distanceToleranceMeters, Constants.Drivetrain.velocityToleranceMetersPerSec);
    

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
    double output = pid.calculate(drivetrain.getAvgEncoderCountMeters());
    SmartDashboard.putNumber("Position Error: ", pid.getPositionError());
    drivetrain.PIDarcadeDrive(output);
     
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

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
