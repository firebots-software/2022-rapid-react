// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  //TODO: Java doc
    private TalonFX motor;
    private DigitalInput hallEffect;

    private static Turret instance;
    private final double RAMPING_CONSTANT = 0.25;
    private final double maxRange = 80; 
    private final double minRange = -80;
    private SimpleMotorFeedforward turretFF;

  /** Creates a new Turret. */
  private Turret() {
    this.motor = new TalonFX(Constants.Turret.motorPortNumber);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(RAMPING_CONSTANT); 
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configMotionAcceleration(20 * Constants.Turret.encoderTicksPerDegree * 0.1);
    motor.configMotionCruiseVelocity(150 * Constants.Turret.encoderTicksPerDegree * 0.1);
    motor.configMotionSCurveStrength(8);
    turretFF = new SimpleMotorFeedforward(Constants.Turret.ksTurret, Constants.Turret.kvTurret, Constants.Turret.kaTurret);

    hallEffect = new DigitalInput(Constants.Turret.hallEffectPort);
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
    if (isHallEffectEnabled()) zeroEncoder();

    if (getEncoderValDegrees() >= maxRange && motor.getMotorOutputPercent() > 0) {
      stopMotor();
    } 

    if (getEncoderValDegrees() >= minRange && motor.getMotorOutputPercent() < 0) {
      stopMotor();
    }

  }

  public void setMotorSpeed(double speed) {
    // if (this.getEncoderValDegrees() <= minRange + 10) {
    //   speed *= 0.5; 
    // } else if (this.getEncoderValDegrees() >= maxRange - 10){
    //   speed *= 0.5; 
    // }
    
    if (this.getEncoderValDegrees() <= minRange && speed < 0) {
      speed = 0; // left hardstop
    } else if (this.getEncoderValDegrees() >= maxRange && speed > 0) {
      speed = 0; // right hardstop
    }   
    
    motor.set(ControlMode.PercentOutput, speed);
    
  }

  public void setTurretClosedLoopVelocity(double degPerS) {
    motor.set(
        ControlMode.Velocity,
        degPerS * Constants.Turret.encoderTicksPerDegree * 0.1, // encoder ticks per 100ms
        DemandType.ArbitraryFeedForward,
        (turretFF.calculate(degPerS) / 12.0) - 10 // MAGIC NUMBER
    );

  }

  public void setTurretPosition(double degrees) {
    if (getEncoderValDegrees() + degrees > maxRange) { // right soft stop -- only go up to maxrange
      degrees = maxRange - getEncoderValDegrees();
    } 

    if (getEncoderValDegrees() + degrees < minRange) { // left soft stop
      degrees = minRange - getEncoderValDegrees();
    }

    motor.set(
      ControlMode.Position,
      degrees * Constants.Turret.encoderTicksPerDegree,
      DemandType.ArbitraryFeedForward,
      (turretFF.calculate(degrees) / 12.0)
    );

  }



  public double getMotionMagicPosition() {
    return motor.getActiveTrajectoryPosition() / Constants.Turret.encoderTicksPerDegree;
  }


  public void stopMotor() {
    setMotorSpeed(0);
  }

  public boolean isHallEffectEnabled() {
    return !hallEffect.get();
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

  public double getDegreesPerSec() {
    return (motor.getSelectedSensorVelocity() / Constants.Turret.encoderTicksPerDegree) * 10;
  }

}
