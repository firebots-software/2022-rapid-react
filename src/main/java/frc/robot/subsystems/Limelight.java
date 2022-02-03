// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
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
  
  // Limelight tx value - x degree offset
  private double tx; 

  // Limelight ty value - y degree offset
  private double ty;

  // Limelight tv value - whether or not there is a valid target - 1 for yes, 0 for no
  private double tv;  

  
  /** Creates a new Limelight. */
  public Limelight() {
    
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
   * Refreshes (updates) Limelight tx and ty values
   */

  public void refreshValues(){
    tx = table.getEntry("tx").getDouble(0); 
    ty = table.getEntry("ty").getDouble(0); 
    tv = table.getEntry("tv").getDouble(0); 
  }

  public double getTX(){
    return tx;  
  }

  public double getTY(){
    return ty;  
  }

  public double getTV(){
    return tv; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
