// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
  // keep spinning after aimed and during load ball --> toggleFlywheel
  private final Shooter shooter;
  private Limelight limelight;
  protected final PIDController pidTop;
  protected final PIDController pidBottom; 
  private final double kp, ki, kd; 
  private double currentVoltageTop, currentVoltageBottom;
  protected Timer timer;
  private final double RPM_TOLERANCE = 50;

  /** Creates a new SpinUpShooter. */
  public SpinUpShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter.getInstance();
    this.limelight = Limelight.getInstance();

    kp = 0.000005; 
    ki = 0; 
    kd = 0.00; 
    pidTop = new PIDController(kp, ki, kd); 
    pidBottom = new PIDController(kp, ki, kd); 

    pidTop.setTolerance(RPM_TOLERANCE);
    pidBottom.setTolerance(RPM_TOLERANCE);

    timer = new Timer();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLedStatus(true);
    setTargetRPM();
   

    currentVoltageTop = 0;
    currentVoltageBottom = 0;

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setTargetRPM();
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
    boolean done = pidTop.atSetpoint() & pidBottom.atSetpoint();
    if (done) {
      timer.stop();
    }
    SmartDashboard.putBoolean("SHOOTER AT RPM", done);
    return false; // RETURN FALSE -- KEEP THE WHEELS SPINNING ONCE THEY'RE UP TO SPEED
  }

  private void setTargetRPM() {
    pidTop.setSetpoint(shooter.getTopTargetRPM());
    pidBottom.setSetpoint(shooter.getBottomTargetRPM());

    if (shooter.getTopTargetRPM() > 3500) {
      shooter.setRampingConstant(0.12);
    } else {
      shooter.setRampingConstant(0.25);
    }
  }
}
