// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToTarget extends CommandBase {
  private Limelight limelight; 
  private Drivetrain drivetrain; 
  private PIDController pid; 

  private double kp, ki, kd; 
  private double tolerance; 
  private double velocityTol; 
  int counter; 

  /** Creates a new AlignToTarget. */
  public AlignToTarget() {
    limelight = Limelight.getInstance(); 
    drivetrain = Drivetrain.getInstance(); 

    kp = Constants.Drivetrain.smallTurnP; 
    ki = Constants.Drivetrain.smallTurnI; 
    kd = Constants.Drivetrain.smallTurnD; 
    pid = new PIDController(kp, ki, kd); 
    pid.setTolerance(tolerance, velocityTol);
    counter = 0; 
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    limelight.refreshValues();
    pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(0, pid.calculate(limelight.getTX()));
    if (counter % 10 == 0) {
      limelight.refreshValues();
      pid.setSetpoint(limelight.getTX());
    }

    counter++; 
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
      // TODO Auto-generated method stub
      return Set.of(limelight, drivetrain);
  }
}
