// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AlignToTargetFeedForward extends CommandBase {
  private Limelight limelight;
  private Turret turret;
  private Drivetrain drivetrain;
  private PIDController pid;
  private SimpleMotorFeedforward feedforward;

  /** Creates a new AlignToTarget. */
  public AlignToTargetFeedForward() {
    // initializing subsystems
    limelight = Limelight.getInstance();
    turret = Turret.getInstance();
    drivetrain = Drivetrain.getInstance();

    // PID and feed forward controllers
    pid = new PIDController(Constants.Limelight.alignP, Constants.Limelight.alignI, Constants.Limelight.alignD);
    pid.setTolerance(Constants.Turret.pidPositionToleranceDegrees);
    feedforward = new SimpleMotorFeedforward(Constants.Turret.ksTurret, Constants.Turret.kvTurret,
        Constants.Turret.kaTurret);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // System.out.println("starting command");
    limelight.setLedStatus(true);
    limelight.refreshValues();
    pid.setSetpoint(turret.getEncoderValDegrees() + limelight.getTx());
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Counter refreshes every 200 ms
  @Override
  public void execute() {
    // if target not seen, follow the direction drivetrain is moving
    if (!limelight.getTv()) {
      double angVel = drivetrain.getAngularVelocity(); 
      if (angVel > Constants.Turret.velThreshold) {
        turret.setMotorSpeed(Constants.Turret.constantTurretTurnSpeed);
        System.out.println("flipping right");
      } else if (angVel < -Constants.Turret.velThreshold) {
        turret.setMotorSpeed(-Constants.Turret.constantTurretTurnSpeed);
        System.out.println("flipping left");
      } 
    } else {  // if target is seen, use PID and feedforward to rotate the turret to align with the target
      pid.setSetpoint(turret.getEncoderValDegrees() + limelight.getTx());

      double angularVel = -drivetrain.getAngularVelocity();
      double pidOutput = pid.calculate(turret.getEncoderValDegrees());
      
      if (pid.atSetpoint()) {
        pidOutput = 0;
      }

      double feedForwardCalculationOnlyAngular = feedforward.calculate(angularVel);
      SmartDashboard.putNumber("turret feedforward output", feedForwardCalculationOnlyAngular);


      if (!limelight.hasSeenTarget()) { // if limelight has never seen target don't move
        turret.stopMotor();
      } else {
        turret.setMotorSpeed(pidOutput + feedForwardCalculationOnlyAngular / 12 * Constants.Turret.feedForwardConstant); // divide by 12 because feed forward output in volts 
      }
     
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLastAlignedGyro(drivetrain.getHeading()); // save the last aligned gyro valud
    turret.stopMotor();
    System.out.println("done with command");
  }

  public boolean isFinished() {
    // return pid.atSetpoint();
    return false;
  }

  // Returns true when the command should end.
  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(limelight, turret);
  }
}

// CLEANED