// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  //TODO: Java doc
    private TalonFX motor;
    private static Turret instance;
    private final double RAMPING_CONSTANT = 0.25;

  /** Creates a new Turret. */
  private Turret() {
    this.motor = new TalonFX(Constants.Turret.motorPortNumber);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(RAMPING_CONSTANT); 
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
