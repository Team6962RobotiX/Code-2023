// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final IMU IMU = new IMU();
  private final Drive drive = new Drive(IMU);
  private final Arm arm = new Arm(() -> driveJoystick);
  private final PneumaticClaw claw = new PneumaticClaw();
  private final Limelight topLimelight = new Limelight(Constants.TOP_LIMELIGHT_NAME);
  private final Limelight bottomLimelight = new Limelight(Constants.BOTTOM_LIMELIGHT_NAME);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new JoystickDrive(drive, () -> driveJoystick));
    // topLimelight.setDefaultCommand(new AutoOrient(topLimelight, drive, arm));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveJoystick, 5).onTrue(claw.toggle());
    // new JoystickButton(driveJoystick, 3).onTrue(claw.open());

    new JoystickButton(driveJoystick, 7).onTrue(new DriveStraight(drive, IMU, -1, 0.3));
    // new JoystickButton(driveJoystick, 9).onTrue(arm.liftToAngle(0));
    // new JoystickButton(driveJoystick, 9).onTrue(arm.extendToLength(0));

    new JoystickButton(driveJoystick, 6).onTrue(arm.extendToLength(1.8 - Constants.ARM_STARTING_LENGTH));
    new JoystickButton(driveJoystick, 4).onTrue(arm.extendToLength(1 - Constants.ARM_STARTING_LENGTH));

    //Orient with an object
    new JoystickButton(driveJoystick, 8).whileTrue(new AutoOrient(bottomLimelight, drive, arm));

    //Orient with vision targets
    new JoystickButton(utilityJoystick, 9).whileTrue(new AutoOrient(topLimelight, drive, arm));

    // new JoystickButton(driveJoystick, 12).onTrue(arm.setLiftPowerCmd(-0.4));
    // new JoystickButton(driveJoystick, 10).onTrue(arm.setLiftPowerCmd(0.4));
    // new JoystickButton(driveJoystick, 12).or(new JoystickButton(driveJoystick, 10)).onFalse(arm.setLiftPowerCmd(0));

    new JoystickButton(driveJoystick, 2).whileTrue(new AutoBalance(IMU, drive));
    // new JoystickButton(driveJoystick, 2).onTrue(arm.coast());
    // new JoystickButton(driveJoystick, 2).onFalse(arm.brake());

  }

  public Command getAutonomousCommand() {

    // testing
    return new SequentialCommandGroup(
      new DriveStraight(drive, IMU, .3, 0.4, true),
      new AutoBalance(IMU, drive)
    );
  }

  public void disabledPeriodic() {
    arm.resetPID();
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
