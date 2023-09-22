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

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



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
  private final Intake intake = new Intake();
  // private final ExampleSubsystem tester = new ExampleSubsystem();

  private final Command simpleauto1 = new SequentialCommandGroup(
    new DriveStraight(drive, IMU, -.2, 0.6), 
    new DriveStraight(drive, IMU, 1.7, 0.4),
    new AutoBalance(IMU, drive)
  );

  private final Command complexauto1 = new SequentialCommandGroup(
    new DriveStraight(drive, IMU, -.2, 0.6), 
    new DriveStraight(drive, IMU, 2.2, 0.5),
    new DriveStraight(drive, IMU, 1.8, 0.4),
    new RotateDrive(drive, IMU, 175),
    new DriveStraight(drive, IMU, 2.0, 0.5),
    new AutoBalance(IMU, drive)
  );

  private final Command auto2 = new SequentialCommandGroup(
    new DriveStraight(drive, IMU, -.2, 0.6),
    new DriveStraight(drive, IMU, 4.5, 0.6)
  );

  private final Command auto3 = new SequentialCommandGroup(
    new RotateDrive(drive, IMU, 180),
    arm.toPosition(1.6, 0.7),
    new WaitCommand(3)
    // claw.toggle()
    //new DriveStraight(drive, IMU, -0.5, 0.4),
    //new RotateDrive(drive, IMU, 180)
  );

  private final Command noauto = new SequentialCommandGroup();
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chooser.setDefaultOption("Complex Auto 1", complexauto1);
    chooser.addOption("Simple Auto 1", simpleauto1);
    chooser.addOption("Auto 2", auto2);
    chooser.addOption("Auto 3", auto3);
    chooser.addOption("No Auto", noauto);

    SmartDashboard.putData(chooser);


    drive.setDefaultCommand(new JoystickDrive(drive, () -> driveJoystick));
    if (Constants.ENABLE_ARM) {
      arm.setDefaultCommand(new JoystickArm(arm, () -> utilityJoystick));
    }

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // MAIN DRIVER

    new JoystickButton(driveJoystick, 12).whileTrue(new AutoBalance(IMU, drive));
    // new JoystickButton(driveJoystick, 3).whileTrue(new AutoOrient(Constants.BOTTOM_LIMELIGHT_NAME, drive));
    // new JoystickButton(driveJoystick, 5).whileTrue(new AutoOrient(Constants.TOP_LIMELIGHT_NAME, drive));

    // UTILITY DRIVER
    new JoystickButton(utilityJoystick, 1).whileTrue(intake.output());
    new JoystickButton(utilityJoystick, 2).whileTrue(intake.intake());

    new JoystickButton(utilityJoystick, 11).onTrue(arm.liftToAngle(90));
    new JoystickButton(utilityJoystick, 11).onTrue(arm.extendToLength(0));

    // new JoystickButton(utilityJoystick, 12).onTrue(arm.toPosition(1.15, 0.3)); // BOTTOM CONE
    // new JoystickButton(utilityJoystick, 11).onTrue(arm.toPosition(1.15, 0.15)); // BOTTOM CUBE
    // new JoystickButton(utilityJoystick, 10).onTrue(arm.toPosition(1.22, 1.3)); // MIDDLE
    // new JoystickButton(utilityJoystick, 8).onTrue(arm.toPosition(1.6, 1.7)); // TOP
    // new JoystickButton(utilityJoystick, 9).onTrue(arm.toPosition(Constants.ARM_STARTING_LENGTH, 1.25)); // DOUBLE SUBSTATION
    new JoystickButton(utilityJoystick, 7).onTrue(arm.toPosition(Constants.ARM_MAX_LENGTH, Constants.ARM_HEIGHT));
    // new JoystickButton(utilityJoystick, 7).onTrue(arm.extendToLength(0));
    // new JoystickButton(utilityJoystick, 2).onTrue(arm.extendToLength(0));
    // new JoystickButton(utilityJoystick, 5).onTrue(arm.extendToLength(0)); // STOW
    // new JoystickButton(utilityJoystick, 5).onTrue(arm.liftToAngle(55)); // STOW

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
    // return null;
  }

  public void disabledPeriodic() {
    arm.resetPID();
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
