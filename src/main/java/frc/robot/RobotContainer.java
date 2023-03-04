// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick driveJoystick = new Joystick(Constants.USB_DRIVE_JOYSTICK);
  private final Joystick utilityJoystick = new Joystick(Constants.USB_UTILITY_JOYSTICK);

  private final Drive drive = new Drive();
  private final IMU IMU = new IMU();
  private final Arm arm = new Arm();
  private final PneumaticClaw claw = new PneumaticClaw();
  private final Limelight limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new JoystickDrive(drive, () -> driveJoystick.getRawAxis(1), () -> driveJoystick.getRawAxis(2)));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveJoystick, 5).onTrue(claw.close());
    new JoystickButton(driveJoystick, 3).onTrue(claw.open());

    new POVButton(driveJoystick, 0).or(new POVButton(driveJoystick, 315)).or(new POVButton(driveJoystick, 45)).whileTrue(new SetArmExtensionPower(arm, Constants.ARM_EXTEND_MAX_POWER));
    new POVButton(driveJoystick, 180).or(new POVButton(driveJoystick, 225)).or(new POVButton(driveJoystick, 135)).whileTrue(new SetArmExtensionPower(arm, -Constants.ARM_EXTEND_MAX_POWER));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
