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
  private final Arm arm = new Arm();
  private final PneumaticClaw claw = new PneumaticClaw();
  private final Limelight limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new JoystickDrive(drive, () -> driveJoystick.getRawAxis(1), () -> driveJoystick.getRawAxis(2)));

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveJoystick, 5).onTrue(claw.close());
    new JoystickButton(driveJoystick, 3).onTrue(claw.open());

    new POVButton(driveJoystick, 0).or(new POVButton(driveJoystick, 315)).or(new POVButton(driveJoystick, 45)).whileTrue(new SetArmExtensionPower(arm, 0.4));
    new POVButton(driveJoystick, 180).or(new POVButton(driveJoystick, 225)).or(new POVButton(driveJoystick, 135)).whileTrue(new SetArmExtensionPower(arm, -0.4));

    new JoystickButton(driveJoystick, 12).onTrue(arm.setLiftPowerCmd(-0.4));
    new JoystickButton(driveJoystick, 10).onTrue(arm.setLiftPowerCmd(0.4));
    new JoystickButton(driveJoystick, 12).or(new JoystickButton(driveJoystick, 10)).onFalse(arm.setLiftPowerCmd(0));

    new JoystickButton(driveJoystick, 11).onTrue(new AutoBalance(IMU, drive));
    new JoystickButton(driveJoystick, 11).onFalse(new JoystickDrive(drive, () -> driveJoystick.getRawAxis(1), () -> driveJoystick.getRawAxis(2)));
    new JoystickButton(driveJoystick, 2).onTrue(arm.coast());
    new JoystickButton(driveJoystick, 2).onFalse(arm.brake());

  }

  public Command getAutonomousCommand() {
    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA), Constants.DRIVE_KINEMATICS, 10);

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(Constants.AUTONOMOUS_MAX_SPEED, Constants.AUTONOMOUS_MAX_ACCELERATION)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(Constants.DRIVE_KINEMATICS)
    //     // Apply the voltage constraint
    //     .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(

    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),

    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),

    //     // Pass config
    //     config);

    // RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drive::getPose, new RamseteController(Constants.AUTONOMOUS_RAMSETE_B, Constants.AUTONOMOUS_RAMSETE_ZETA), new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA), Constants.DRIVE_KINEMATICS, drive::getWheelSpeeds, new PIDController(Constants.DRIVE_KP, 0, 0), new PIDController(Constants.DRIVE_KP, 0, 0),

    //     // RamseteCommand passes volts to the callback
    //     drive::tankDriveVolts, drive);

    // // Reset odometry to the starting pose of the trajectory.
    // drive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    return null;
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
