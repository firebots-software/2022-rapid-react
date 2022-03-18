// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SpinFlywheelFeedForward extends CommandBase {
  private Shooter shooter;
  private Limelight limelight;
  private PIDController pidTop, pidBottom;
  private SimpleMotorFeedforward feedforwardTop, feedforwardBottom;
  private double RPM_TOLERANCE = 50;

  private double sum;
  private double counter = 10;
  private double currentIndex;

  /** Creates a new SpinFlywheelFeedForward. */
  public SpinFlywheelFeedForward() {
    shooter = Shooter.getInstance();

    // top
    pidTop = new PIDController(Constants.Shooter.kpFlywheel, 0, 0);
    pidTop.setTolerance(RPM_TOLERANCE);
    feedforwardTop = new SimpleMotorFeedforward(Constants.Shooter.ksTopFlywheel, Constants.Shooter.kvTopFlywheel,
        Constants.Shooter.kaTopFlywheel);

    pidBottom = new PIDController(Constants.Shooter.kpFlywheel, 0, 0);
    pidTop.setTolerance(RPM_TOLERANCE);
    feedforwardTop = new SimpleMotorFeedforward(Constants.Shooter.ksBottomFlywheel, Constants.Shooter.kvBottomFlywheel,
        Constants.Shooter.kaBottomFlywheel);

    sum = 0; 
    currentIndex = 0;

    // bottom
    // pidBottom = new PIDController(Constants.Shooter.kpFlywheel,
    // Constants.Shooter.kiFlywheel, Constants.Shooter.kdFlywheel);
    // pidBottom.setTolerance(RPM_TOLERANCE);
    // feedforwardBottom = new
    // SimpleMotorFeedforward(Constants.Shooter.ksBottomFlywheel,
    // Constants.Shooter.kvBottomFlywheel, Constants.Shooter.kaBottomFlywheel);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidTop.setSetpoint(shooter.getTopTargetRPM());
    pidBottom.setSetpoint(shooter.getBottomTargetRPM());
    System.out.println("starting feed forward");
    // pidBottom.setSetpoint(shooter.getBottomTargetRPM());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentIndex != counter) {
      if (limelight.getDistanceToTarget() != Constants.Limelight.LIMELIGHT_DEFAULT_VALUE) {
        sum += limelight.getDistanceToTarget();
        currentIndex++;
      }
    } else {
      sum += limelight.getDistanceToTarget(); 
      currentIndex++;
      double averageDist = sum/currentIndex; 
      pidTop.setSetpoint(shooter.getTopTargetRPM());
      pidBottom.setSetpoint(shooter.getBottomTargetRPM());
      double ffOutTop = feedforwardTop.calculate((averageDist) / 60.0) /
          (Constants.Shooter.MAX_VOLTAGE * Constants.Shooter.MAX_RPM);
      double topOutput = ffOutTop + pidTop.calculate(shooter.getTopShooterRPM());
      double ffOutBottom = feedforwardTop.calculate((averageDist) / 60.0) /
          (Constants.Shooter.MAX_VOLTAGE * Constants.Shooter.MAX_RPM);
      double bottomOutput = ffOutBottom + pidBottom.calculate(shooter.getBottomShooterRPM()); 

      shooter.setTopMotorSpeed(topOutput);
      shooter.setBottomMotorSpeed(bottomOutput);

      SmartDashboard.putNumber("feed forward output", topOutput);
    }
    // shooter.setTopMotorSpeed

    // double bottomOutput =
    // feedforwardBottom.calculate(shooter.getBottomTargetRPM()) +
    // pidTop.calculate(shooter.getBottomShooterRPM());
    // shooter.setBottomMotorSpeed(bottomOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopBothMotors();
    System.out.println("ending feed forward");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = pidTop.atSetpoint();// && pidBottom.atSetpoint();
    SmartDashboard.putBoolean("flywheel feed forward done", done);
    return false;
  }
}
