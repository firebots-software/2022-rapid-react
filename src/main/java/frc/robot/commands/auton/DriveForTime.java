// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveForTime extends CommandBase {
  private final Drivetrain drivetrain;
  private double speed;
  private double targetTime;
  private Timer timer;

  /**
   * Drive the robot for a set time
   * 
   * @param speed = drivetrain front-back percent output, [-1, 1] where + is forward
   * @param targetTime = time (s) to drive
   */
  public DriveForTime(double speed,double targetTime) {
    // System.out.println("INIT TIME DRIVE");
    this.drivetrain = Drivetrain.getInstance();
    timer = new Timer();
    this.speed = speed;
    this.targetTime = targetTime;
  }

  
  @Override
  public void initialize() {
    timer.start();
  }

  
  /**
   * Sets speed and angle of robot while driving
   */
  @Override
  public void execute() {
    drivetrain.PIDarcadeDrive(speed, 0);
  }

  /**
   * Stop motors when timer has elapsed
   */
  @Override
  public void end(boolean interrupted) {
    // System.out.println("TIME DRIVE DONE");
    timer.stop();
    timer.reset();
    drivetrain.stop();
  }

  /**
   * If time equals timer, then stop and reset robot and return true.
   */
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(targetTime);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
