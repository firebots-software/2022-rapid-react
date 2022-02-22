// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnForAngle extends CommandBase {
  private Drivetrain drivetrain;
  private PIDController pid;

  /** Creates a new DriveForDistanceSingleController. */
  public TurnForAngle(double targetAngleDegrees) {
    drivetrain = Drivetrain.getInstance();
    pid = new PIDController(Constants.Drivetrain.angleP, Constants.Drivetrain.angleI, Constants.Drivetrain.angleD);
    pid.setSetpoint(targetAngleDegrees);
    pid.setTolerance(Constants.Drivetrain.angleToleranceDegrees, Constants.Drivetrain.velocityToleranceDegreesPerSec);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = -pid.calculate(drivetrain.getHeading());
    drivetrain.PIDarcadeDriveAngle(output);
    SmartDashboard.putNumber("PID output", output); 
    SmartDashboard.putNumber("PID error", pid.getPositionError());
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
