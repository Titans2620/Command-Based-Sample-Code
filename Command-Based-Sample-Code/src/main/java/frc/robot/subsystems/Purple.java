// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Purple extends SubsystemBase {
  private CANSparkMax purple = new CANSparkMax(Constants.MOTOR_PID_CONTROL_ID, MotorType.kBrushless);
  private SparkPIDController p_purplePIDController;
  private RelativeEncoder p_Encoder;
  private double purpleSetPoint;
  private double kp, ki, kd, kiz, kff, kMaxOutput, kMinOutput, maxRpm, maxVel, minVel, maxAcc, allowedErr;
  
  public Purple() {
    purple.restoreFactoryDefaults();
    purpleSetPoint = 0.0;
    p_purplePIDController = purple.getPIDController();
    p_Encoder = purple.getEncoder();

    kp = 5e-2; 
    ki = 1e-6;
    kd = 0; 
    kiz = 0; 
    kff = 0.000156; 
    kMaxOutput = 0.25; 
    kMinOutput = -1;
    maxRpm = 5700;

    p_purplePIDController.setP(kp);
   // m_pidController.setI(ki);
    p_purplePIDController.setD(kd);
  //  m_pidController.setIZone(kiz);
    p_purplePIDController.setFF(kff);
    p_purplePIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setTargetPosition(double purpleSetPoint) {
    this.purpleSetPoint = purpleSetPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    p_purplePIDController.setReference(purpleSetPoint, CANSparkMax.ControlType.kPosition);
  }
}
