// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class LimelightAimPosControl extends CommandBase {
  private Limelight limelight;
  private Turret turret;
  private double feedbackDelayCounter;
  private final int LIMELIGHT_REFRESH_INTERVAL = 5; // number of loops before refreshing Limelight
  private final double TX_THRESHOLD = 1;


  /** Creates a new AlignToTargetPosControl. */
  public LimelightAimPosControl() {
    limelight = Limelight.getInstance();
    turret = Turret.getInstance();

    addRequirements(turret, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLedStatus(true);
    limelight.refreshValues();

    feedbackDelayCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelight.getTv()) {
      if (limelight.getLastKnownTx() > 0) {
        turret.setMotorSpeed(Constants.Turret.constantTurretTurnSpeed);
      } else if (limelight.getLastKnownTx() < 0) {
        turret.setMotorSpeed(-Constants.Turret.constantTurretTurnSpeed);
      }
    } else {
      if (feedbackDelayCounter % LIMELIGHT_REFRESH_INTERVAL == 0) {
        limelight.refreshValues();
        turret.setTurretPosition(limelight.getTx());;
      }
      feedbackDelayCounter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getTv() && (Math.abs(limelight.getTx()) < TX_THRESHOLD);
  }
}
