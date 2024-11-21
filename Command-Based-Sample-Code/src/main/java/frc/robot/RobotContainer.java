// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetMotorControl;
import frc.robot.subsystems.MotorControlSubsystem;
import frc.robot.subsystems.MotorPIDControlSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static MotorControlSubsystem m_MotorControlSubsystem = new MotorControlSubsystem();
  private static MotorPIDControlSubsystem m_MotorPIDControlSubsystem = new MotorPIDControlSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);

  //buttons
  private final JoystickButton runMotorForward = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton runMotorReverse = new JoystickButton(m_driverController, XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    CameraServer.startAutomaticCapture();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    DataLog log = DataLogManager.getLog();

    configureBindings();
  }

  private void configureBindings() {
      //Set Default Commands
      m_MotorControlSubsystem.setDefaultCommand(new SetMotorControl(m_MotorControlSubsystem, 0.0));
      runMotorForward.whileTrue(new SetMotorControl(m_MotorControlSubsystem, 12.0));
      runMotorReverse.whileTrue(new SetMotorControl(m_MotorControlSubsystem, -12.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
