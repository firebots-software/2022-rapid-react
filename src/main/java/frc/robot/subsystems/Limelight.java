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


  private NetworkTableEntry tx;
  private NetworkTableEntry ty;

  /** Creates a new Limelight. */
  public Limelight() {
    tx.setDouble(0.0);
    ty.setDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}