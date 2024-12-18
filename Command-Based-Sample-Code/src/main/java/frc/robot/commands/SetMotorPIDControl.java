// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorPIDControlSubsystem;

public class SetMotorPIDControl extends Command {
  /** Creates a new SetMotorPIDControl. */
  MotorPIDControlSubsystem m_MotorControlPIDSubsystem;
  double setPoint;

  public SetMotorPIDControl(MotorPIDControlSubsystem m_MotorPIDControlSubsystem, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_MotorControlPIDSubsystem = m_MotorPIDControlSubsystem;
    this.setPoint = setPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_MotorControlPIDSubsystem.setTargetPosition(setPoint);
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
