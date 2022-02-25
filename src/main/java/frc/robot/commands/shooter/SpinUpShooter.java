// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  // keep spinning after aimed and during load ball --> toggleFlywheel
  private final Shooter shooter;
  private Limelight limelight;
  private final PIDController pidTop, pidBottom; 
  private final double kp, ki, kd; 
  private double currentVoltageTop, currentVoltageBottom;

  /** Creates a new SpinUpShooter. */
  public SpinUpShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
    this.limelight = Limelight.getInstance();

    if (!shooter.isRPMAdjusting()) {
      shooter.setTopTargetRPM(Constants.Shooter.FIXED_RPM);
      shooter.setBottomTargetRPM(Constants.Shooter.FIXED_RPM);
    } else {
      shooter.setTopTargetRPM(shooter.getRPMForDistanceInches(limelight.getDistanceToTarget()));
      shooter.setBottomTargetRPM(shooter.getRPMForDistanceInches(limelight.getDistanceToTarget()));
    }
    kp = 0.000005; 
    ki = 0; 
    kd = 0.00; 
    pidTop = new PIDController(kp, ki, kd); 
    pidBottom = new PIDController(kp, ki, kd); 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidTop.setSetpoint(shooter.getTopTargetRPM());
    pidBottom.setSetpoint(shooter.getBottomTargetRPM());

    currentVoltageTop = 0;
    currentVoltageBottom = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentVoltageTop += pidTop.calculate(shooter.getTopShooterRPM()); 
    SmartDashboard.putNumber("shooter top motor output", currentVoltageTop); 
    shooter.setTopMotorSpeed(currentVoltageTop); 

    currentVoltageBottom += pidBottom.calculate(shooter.getBottomShooterRPM()); 
    SmartDashboard.putNumber("shooter bottom motor output",  currentVoltageBottom); 
    shooter.setBottomMotorSpeed(currentVoltageBottom);
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
