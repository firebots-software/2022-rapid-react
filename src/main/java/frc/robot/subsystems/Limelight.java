// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  /*
   * Get the default instance of NetworkTables that was created automatically
   * when your program starts
   */
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();

  /*
   * Get the table within that instance that contains the data. There can
   * be as many tables as you like and exist to make it easier to organize
   * your data. In this case, it's a table called datatable.
   */
  private NetworkTable table = instance.getTable("limelight");

  // Limelight tx value - x degree offset of target center from viewport center
  private double tx;

  // Limelight ty value - y degree offset of target center from viewport center
  private double ty;

  // Limelight tv value - whether or not there is a valid target - 1 for yes, 0
  // for no
  private double tv;

  // The value of tx to be returned if no value is found
  private final double DEFAULT_VALUE_TX = 0.0;

  // The value of ty to be returned if no value is found
  private final double DEFAULT_VALUE_TY = 0.0;

  // The value of tv to be returned if no target is found
  private final double DEFAULT_VALUE_TV = 0;

  /** Creates a new Limelight. */
  private Limelight() {
      setLedStatus(false);
  }

  /**
   * The Singleton instance of this Limelight. External classes should use the
   * {@link #getInstance()} method to get the instance.
   */
  private static Limelight INSTANCE;

  /**
   * Returns the Singleton instance of this Limelight. This static method should
   * be
   * used -- {@code Limelight.getInstance();} -- by external classes, rather than
   * the
   * constructor to get the instance of this class.
   */
  public static Limelight getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Limelight();
    }
    return INSTANCE;
  }

  /**
   * Refreshes (updates) Limelight tx and ty values
   */
  public void refreshValues() {
    tx = table.getEntry("tx").getDouble(DEFAULT_VALUE_TX);
    ty = table.getEntry("ty").getDouble(DEFAULT_VALUE_TY);
    tv = table.getEntry("tv").getDouble(DEFAULT_VALUE_TV);
  }

  /**
   * Returns the horizontal offset from the crosshair to the target (-27 degrees
   * to 27 degrees).
   * 
   * @return Horizontal offset in degrees.
   */
  public double getTx() {
    return tx;
  }

  /**
   * Returns the vertical offset from the crosshair to the target (-20.5 degrees
   * to 20.5 degrees).
   * 
   * @return Vertical offset in degrees.
   */
  public double getTy() {
    return ty;
  }

  /**
   * Returns whether the Limelight has any valid targets (0 or 1). This means that
   * the Limelight has a valid target, or is not.
   * 
   * @return Boolean value for if limelight has a target.
   */
  public boolean getTv() {
    return tv == 1;
  }

  public double getRatio() {
    double ratio = Math.tan(((Math.toRadians(this.getTy() + Constants.Limelight.limelightAngleOffset))));
    return ratio;
  }

  public double getDistanceToTarget() {
    double ratio = this.getRatio();
    return (Constants.Limelight.heightOfTarget - 28) / ratio + 8;
  }

  public boolean isWithinShootingRange(){
    // if (shooter.isRPMAdjusting()){
      return 100 < getDistanceToTarget() && getDistanceToTarget() < 135;
    // } else {
    //   return 85 < getDistanceToTarget() && getDistanceToTarget() < 95;
    // }
  }

  public void setLedStatus(boolean on) {
    if (on) {
      table.getEntry("ledMode").setNumber(3);
    } else {
      table.getEntry("ledMode").setNumber(1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}