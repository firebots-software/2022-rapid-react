// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  // keep spinning after aimed and during load ball --> toggleFlywheel
  private final Shooter shooter;
  private final double P = 0.5; //CONSTANT - magic number, figure out thru testing; should be pretty small


  /** Creates a new SpinUpShooter. */
  public SpinUpShooter(double speed) { //speed in RPM
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
    shooter.setTargetSpeed(speed); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topError = shooter.getTargetSpeed() - shooter.getTopShooterRPM();; // desired = desired aimed position we want
    double bottomError = shooter.getTargetSpeed() - shooter.getBottomShooterRPM();; // desired = desired aimed position we want

    double newValTop = P * topError; //magic number P (proportionality constant)
    double newValBottom = P * bottomError; 
    shooter.setTopMotorSpeed(newValTop); //
    shooter.setBottomMotorSpeed(newValBottom);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isAtTargetSpeed(Constants.Shooter.motorSpeedToleranceRPM);
        // possible return false and change it in the command group
        //  TODO: command group - if interrupted then return true --> loadBall
  }
}
