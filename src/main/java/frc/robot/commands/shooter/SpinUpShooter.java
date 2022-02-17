// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  // keep spinning after aimed and during load ball --> toggleFlywheel
  private final Shooter shooter;
  private final PIDController pidTop, pidBottom; 
  private final double kp, ki, kd; 

  /** Creates a new SpinUpShooter. */
  public SpinUpShooter(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
    shooter.setTargetSpeed(speed);
    kp = 0.0008; 
    ki = 0; 
    kd = 0.000014; 
    pidTop = new PIDController(kp, ki, kd); 
    pidBottom = new PIDController(kp, ki, kd); 
    pidTop.setSetpoint(shooter.getTargetSpeed());
    pidBottom.setSetpoint(shooter.getTargetSpeed());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outputTop = pidTop.calculate(shooter.getTopShooterRPM()); 
    SmartDashboard.putNumber("shooter top motor output", outputTop); 
    shooter.setTopMotorSpeed(outputTop); 

    double outputBottom = pidBottom.calculate(shooter.getBottomShooterRPM()); 
    SmartDashboard.putNumber("shooter bottom motor output", outputBottom); 
    // shooter.setBottomMotorSpeed(outputBottom);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopBothMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return shooter.isAtTargetSpeed(Constants.Shooter.motorSpeedToleranceRPM);
        // possible return false and change it in the command group
        //  TODO: command group - if interrupted then return true --> loadBall
  }
}
