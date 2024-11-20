// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorPIDControlSubsystem extends SubsystemBase {
  /** Creates a new MotorPIDControlSubsystem. */
  private CANSparkMax motorPIDControl = new CANSparkMax(Constants.MOTOR_PID_CONTROL_ID, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double setPoint;
  private double kp, ki, kd, kiz, kff, kMaxOutput, kMinOutput, maxRpm, maxVel, minVel, maxAcc, allowedErr;

  //Logging Variables
  DoubleLogEntry setPointLog, positionLog;

  public MotorPIDControlSubsystem() {
    //Logging data
    DataLogManager.start();

    DataLog log = DataLogManager.getLog();
    setPointLog = new DoubleLogEntry(log, "/motorPIDControlSubsystem/setPoint");
    positionLog = new DoubleLogEntry(log, "/motorPIDControlSubsystem/positionLog");

    motorPIDControl.restoreFactoryDefaults();
    setPoint = 0.0;
    m_pidController = motorPIDControl.getPIDController();
    m_encoder = motorPIDControl.getEncoder();

    //PID Coefficients
    kp = 5e-5; 
    ki = 1e-6;
    kd = 0; 
    kiz = 0; 
    kff = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRpm = 5700;

    m_pidController.setP(kp);
    m_pidController.setI(ki);
    m_pidController.setD(kd);
    m_pidController.setIZone(kiz);
    m_pidController.setFF(kff);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setTargetPosition(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    setPointLog.append(setPoint);
    positionLog.append(m_encoder.getPosition());

  }
}
