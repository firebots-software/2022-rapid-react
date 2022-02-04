// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  private final Shooter shooter;
  private double desiredSpeed, currentSpeed, error; //goal angle/point
  private double P = 1/2000; //CONSTANT - magic number, figure out thru testing; should be pretty small

  /** Creates a new SpinUpShooter. */
  public SpinUpShooter(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
    desiredSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentSpeed = shooter.getRPM();
    error = desiredSpeed - currentSpeed; // desired = desired aimed position we want
    double newVal = P * error; //magic number P (proportionality constant)
    shooter.setSpeed(newVal); //
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(error) < Constants.Shooter.motorSpeedToleranceRPM) {
        shooter.setAtTargetSpeed(true);
        // if interrupted then return true --> loadBall 
    }

    return false;
  }
}
