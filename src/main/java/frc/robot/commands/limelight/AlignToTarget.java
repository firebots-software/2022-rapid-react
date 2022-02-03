// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AlignToTarget extends CommandBase {
  private Limelight limelight; 
  private Turret turret; 
  private PIDController pid; 

  private int counter; 

  /** Creates a new AlignToTarget. */
  public AlignToTarget() {
    limelight = Limelight.getInstance();
    turret = Turret.getInstance(); 

    pid = new PIDController(Constants.Limelight.alignP, Constants.Limelight.alignI, Constants.Limelight.alignD); 
    pid.setTolerance(Constants.Limelight.angleTolerance, Constants.Limelight.velocityTolerance);   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.resetEncoders();
    limelight.refreshValues();
    pid.setSetpoint(0);

    counter = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter % 10 == 0) {
      limelight.refreshValues();
    }

    turret.setMotorSpeed(pid.calculate(limelight.getTX()*Constants.Turret.encoderTicksPerDegree)); 

    counter++; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return Set.of(limelight, turret);
  }
}
