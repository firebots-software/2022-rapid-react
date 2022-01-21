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
  private NetworkTable table = instance.getTable("GRIP/myContoursReport");

  private NetworkTableEntry contourReport;
  
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
   * Refreshes (updates) the myContoursReport Network Table
   */
  public void refreshValues(){
    table = instance.getTable("GRIP/myContoursReport");
  }


  public double[][] getContours(){
    double[] areaArr = table.getEntry("area").getDoubleArray(new double[]{}); 
    double[] centerXArr = table.getEntry("centerX").getDoubleArray(new double[]{}); 
    double[] centerYArr = table.getEntry("centerY").getDoubleArray(new double[]{}); 
    double[] widthArr = table.getEntry("width").getDoubleArray(new double[]{}); 
    double[] heightArr = table.getEntry("height").getDoubleArray(new double[]{}); 
    double[] solidityArr = table.getEntry("solidity").getDoubleArray(new double[]{}); 

    return new double[][]{areaArr, centerXArr, centerYArr, widthArr, heightArr, solidityArr};
  }


    // Subsystem Calculations
    // public double getXOffset() {
    //   //TODO: get the centerX from Network Table and return
    // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
