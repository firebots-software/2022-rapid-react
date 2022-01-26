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
    double[] degreeOffsetArr = new double[centerXArr.length];

    // Horizontal FOV is 59.6 degrees
    // Hardware Zoom is 3x
    // Pixels is 320 x 240 fps, center is 159.5th column
    // Detect centerX at certain point
    
    for (int i = 0; i < degreeOffsetArr.length; i++) {
      degreeOffsetArr[i] = (centerXArr[i] - 159.5) * 0.18625; 
    } 

    return new double[][]{areaArr, centerXArr, centerYArr, widthArr, heightArr, solidityArr, degreeOffsetArr};
  }

  // Get endpoints of two furthest contours
  public double[] getEndpoints(){

      double[][] contours = getContours();
      double[] centerXArr = contours[1];
      double[] widthArr = contours[3];

      if(centerXArr.length == 1){
        return new double[]{
          centerXArr[0] - (0.5 * widthArr[0]), centerXArr[0] + (0.5 * widthArr[0])
        };
      }

      double xMax = centerXArr[0], xMin = centerXArr[0];
      double minWidth = 0, maxWidth = 0;

      for (int i = 1; i < centerXArr.length; i++) {
        if(xMax < centerXArr[i]) {
          xMax = centerXArr[i];
          maxWidth = widthArr[i];
        }
        if(xMin > centerXArr[i]) {
          xMin = centerXArr[i];
          minWidth = widthArr[i];
        }
      }
      
      return new double[]{
        xMin - (0.5 * minWidth), xMax + (0.5 * maxWidth)
      };

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
