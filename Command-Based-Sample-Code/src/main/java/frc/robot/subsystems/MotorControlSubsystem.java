// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorControlSubsystem extends SubsystemBase {
  /** Creates a new MotorControlSubsystem. */
  private CANSparkMax motorControl = new CANSparkMax(Constants.MOTOR_CONTROL_ID, MotorType.kBrushless);
  private double voltage;

  DoubleLogEntry voltageLog, motorSpeedLog;

  public MotorControlSubsystem() {
        DataLogManager.start();

    DataLog log = DataLogManager.getLog();
    voltageLog = new DoubleLogEntry(log, "/motorPIDControlSubsystem/voltageLog");
    motorSpeedLog = new DoubleLogEntry(log, "/motorPIDControlSubsystem/motorSpeedLog");
    
    voltage  =0.0;
  }

  public void setVoltage(double voltage){
    this.voltage = voltage;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorControl.setVoltage(voltage);
    voltageLog.append(voltage);
    motorSpeedLog.append(motorControl.getAppliedOutput());
  }
}
