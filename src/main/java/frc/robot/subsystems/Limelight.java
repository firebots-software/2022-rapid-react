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

  // The value of tx to be returned if no value is found
  private final double DEFAULT_VALUE_TX = 0.0;

  // The value of ty to be returned if no value is found
  private final double DEFAULT_VALUE_TY = 0.0;

  /** Creates a new Limelight. */
  public Limelight() {
    tx.setDouble(DEFAULT_VALUE_TX);
    ty.setDouble(DEFAULT_VALUE_TY);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
