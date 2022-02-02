// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  /*Get the default instance of NetworkTables that was created automatically
  when your program starts */
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  
  /*Get the table within that instance that contains the data. There can
  be as many tables as you like and exist to make it easier to organize
  your data. In this case, it's a table called datatable. */
  private NetworkTable table = instance.getTable("limelight");

  // A network table entry that stores the tx value
  private NetworkTableEntry tx;

  // A network table entry that stores the ty value
  private NetworkTableEntry ty;

  // A network table entry that stores the tv value
  private NetworkTableEntry tv;

  // A network table entry that stores the ta value
  private NetworkTableEntry ta;

  // The value of tx to be returned if no value is found
  private final double DEFAULT_VALUE_TX = 0.0;

  // The value of ty to be returned if no value is found
  private final double DEFAULT_VALUE_TY = 0.0;

  // The value of tv to be returned if no target is found
  private final double DEFAULT_VALUE_TV = 0;

  // The value of ta to be returned if no target is found
  private final double DEFAULT_VALUE_TA = 0;

  /** Creates a new Limelight. */
  public Limelight() {
    tx.setDouble(DEFAULT_VALUE_TX);
    ty.setDouble(DEFAULT_VALUE_TY);
    tv.setDouble(DEFAULT_VALUE_TV);
    ta.setDouble(DEFAULT_VALUE_TA);
  }

    /**
   * The Singleton instance of this Limelight. External classes should use the
   * {@link #getInstance()} method to get the instance.
   */
  private static Limelight INSTANCE;

  /**
   * Returns the Singleton instance of this Limelight. This static method should be
   * used -- {@code Limelight.getInstance();} -- by external classes, rather than the
   * constructor to get the instance of this class.
   */
  public static Limelight getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Limelight();
    }
    return INSTANCE;
  }

  /**
   * Refreshes (updates) the tx and ty values
   */
  public void refreshValues(){
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
  }

   /**
     * Returns the horizontal offset from the crosshair to the target (-27 degrees to 27 degrees).
     * @return Horizontal offset in degrees.
     */
  public double getTx() {
    return tx.getDouble(DEFAULT_VALUE_TX);
  }

   /**
     * Returns the vertical offset from the crosshair to the target (-20.5 degrees to 20.5 degrees). 
     * @return Vertical offset in degrees.
     */
  public double getTy() {
    return ty.getDouble(DEFAULT_VALUE_TY);
  }


   /**
     * Returns the target area. (From 0% of the image to 100% of the image) 
     * @return Target area as a percentage.
     */
    public double getTa() {
      return ta.getDouble(DEFAULT_VALUE_TA);
    }


   /**
     * Returns whether the Limelight has any valid targets (0 or 1). This means that the Limelight has a valid target, or is not. 
     * @return Boolean value for if limelight has a target.
     */
    public boolean getTv() {
      return tv.getDouble(DEFAULT_VALUE_TV) == 1;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}