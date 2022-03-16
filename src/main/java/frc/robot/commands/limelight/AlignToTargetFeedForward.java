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
    limelight = Limelight.getInstance();
    turret = Turret.getInstance();
    drivetrain = Drivetrain.getInstance();
    pid = new PIDController(Constants.Limelight.alignP, Constants.Limelight.alignI, Constants.Limelight.alignD);
    pid.setTolerance(Constants.Turret.pidPositionToleranceDegrees, Constants.Turret.pidVelToleranceDegPerSecond);

    feedforward = new SimpleMotorFeedforward(Constants.Limelight.FEED_FORWARD_KS, Constants.Limelight.FEED_FORWARD_KV,
        Constants.Limelight.FEED_FORWARD_KA);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("starting command");
    limelight.setLedStatus(true);
    limelight.refreshValues();
    pid.setSetpoint(drivetrain.getHeading() - drivetrain.getLastAlignedGyro());
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Counter refreshes every 200 ms
  @Override
  public void execute() {
    if (!limelight.getTv()) {
      if (limelight.getLastKnownTx() > 0) {
        turret.setMotorSpeed(Constants.Turret.constantTurretTurnSpeed);
      } else {
        turret.setMotorSpeed(-Constants.Turret.constantTurretTurnSpeed);
      }
    } else {
      // SmartDashboard.setDefaultNumber("ATT: Output Ticks", getOutputTicks());
      double tangentialVel = -drivetrain.getCurrentSpeed() * Math.sin(drivetrain.getHeading()) / (limelight.getDistanceToTarget() * 0.0254); // convert limelight distance to meters
      double angularVel = drivetrain.getAngularVelocity();
      double pidOutput = pid.calculate(turret.getEncoderValDegrees());
      SmartDashboard.putNumber("turret pid output", pidOutput);
      turret.setMotorSpeed(pidOutput + feedforward.calculate(tangentialVel + angularVel));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLastAlignedGyro(drivetrain.getHeading());
    turret.stopMotor();
    // System.out.println("done with command");
  }

  public boolean isFinished() {
    return pid.atSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(limelight, turret);
  }
}
