// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  //TODO: Java doc
    private TalonFX motor;
    private static Turret instance;
    private final double RAMPING_CONSTANT = 0.25;
    private final double maxRange = 0; 
    private final double minRange = -90;

  /** Creates a new Turret. */
  private Turret() {
    this.motor = new TalonFX(Constants.Turret.motorPortNumber);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(RAMPING_CONSTANT); 

    zeroEncoder();
  }

  public static Turret getInstance(){
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    if (this.getEncoderValDegrees() <= minRange && speed < 0) {
      speed = 0; // left hardstop
    } else if (this.getEncoderValDegrees() >= maxRange && speed > 0) {
      speed = 0; // right hardstop
    }
    
    motor.set(ControlMode.PercentOutput, speed);
    
  }

  public void stopMotor() {
    setMotorSpeed(0);
  }

  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }

  public double getEncoderValTicks() {
    return motor.getSelectedSensorPosition();
  }

  public double getEncoderValDegrees() {
    return motor.getSelectedSensorPosition() / Constants.Turret.encoderTicksPerDegree;
  }

}
