// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorControlSubsystem;

public class SetMotorControl extends Command {
  /** Creates a new SetMotorControl. */
  MotorControlSubsystem m_MotorControlSubsystem;
  double voltage;

  public SetMotorControl(MotorControlSubsystem m_MotorControlSubsystem, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_MotorControlSubsystem = m_MotorControlSubsystem;
    this.voltage = voltage;
    addRequirements(this.m_MotorControlSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_MotorControlSubsystem.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
