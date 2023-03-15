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
  private final Arm arm = new Arm(() -> driveJoystick);
  private final PneumaticClaw claw = new PneumaticClaw();
  private final Limelight topLimelight = new Limelight(Constants.TOP_LIMELIGHT_NAME);
  private final Limelight bottomLimelight = new Limelight(Constants.BOTTOM_LIMELIGHT_NAME);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new JoystickDrive(drive, () -> driveJoystick));
    arm.setDefaultCommand(new JoystickArm(arm, () -> utilityJoystick));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // MAIN DRIVER

    new JoystickButton(driveJoystick, 12).whileTrue(new AutoBalance(IMU, drive));
    new JoystickButton(driveJoystick, 5).whileTrue(new AutoOrient(bottomLimelight, drive, arm));
    new JoystickButton(driveJoystick, 6).whileTrue(new AutoOrient(topLimelight, drive, arm));

    // UTILITY DRIVER
    new JoystickButton(utilityJoystick, 1).onTrue(claw.toggle());

    new JoystickButton(utilityJoystick, 12).onTrue(arm.toPosition(1.15, 0.0)); // BOTTOM
    new JoystickButton(utilityJoystick, 10).onTrue(arm.toPosition(1.22, 1.3)); // MIDDLE
    new JoystickButton(utilityJoystick, 8).onTrue(arm.toPosition(1.6, 1.7)); // TOP
    new JoystickButton(utilityJoystick, 9).onTrue(arm.toPosition(Constants.ARM_STARTING_LENGTH, 1.2)); // DOUBLE SUBSTATION
    new JoystickButton(utilityJoystick, 11).onTrue(arm.liftToAngle(0));
    new JoystickButton(utilityJoystick, 11).onTrue(arm.extendToLength(0));
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

  public void disabledPeriodic() {
    arm.resetPID();
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
