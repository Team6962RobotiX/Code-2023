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
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import java.util.List;

import javax.swing.RepaintManager;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final Logger logger = new Logger(drive, arm, intake, IMU);
  // private final ExampleSubsystem tester = new ExampleSubsystem();

  // private final Command simpleauto1 = new SequentialCommandGroup(
  //   new DriveStraight(drive, IMU, -.2, 0.6), 
  //   new DriveStraight(drive, IMU, 1.7, 0.4),
  //   new AutoBalance(IMU, drive)
  // );

  private final Command balanceAuto = new SequentialCommandGroup(
    IMU.resetVariables(),
    // new DriveUntil(drive, IMU::isCenteredOnStation, Constants.AUTONOMOUS_POWER, 0.0),
    new AutoBalance(IMU, drive)
  );

  private final Command communityAuto = new SequentialCommandGroup(
    IMU.resetVariables(),
    new DriveUntil(drive, (() -> true), -0.6, 0.1),
    new DriveUntil(drive, IMU::isOnStation, 0.6, 0.0),
    new DriveUntil(drive, IMU::isOffStation,  0.4, 0.1),
    new WaitCommand(0.2),
    new RotateDrive(drive, IMU, 180.0, 0.8),
    new WaitCommand(0.2),
    new DriveUntil(drive, IMU::isOnStation, 0.6, 0.0),
    new DriveUntil(drive, IMU::isCenteredOnStation, 0.37, 0.0),
    new AutoBalance(IMU, drive)
  );

  private final Command placeAuto = new SequentialCommandGroup(
    IMU.resetVariables(),
    arm.toPosition(1.6, 1.55),
    new WaitUntilCommand(arm::doneMoving),
    new WaitCommand(0.2),
    intake.totalOutput(),
    new WaitCommand(0.4),
    arm.extendToLength(0),
    arm.liftToAngle(Constants.ARM_LIFT_MIN_ANGLE),
    new WaitUntilCommand(arm::doneMoving)
  );

  private final Command placeAndBalanceAuto = new SequentialCommandGroup(
    IMU.resetVariables(),
    arm.toPosition(1.6, 1.55),
    new WaitUntilCommand(arm::doneMoving),
    new WaitCommand(0.2),
    intake.totalOutput(),
    new WaitCommand(0.4),
    arm.extendToLength(0),
    arm.liftToAngle(Constants.ARM_LIFT_MIN_ANGLE),
    new WaitUntilCommand(arm::doneMoving),
    new RotateDrive(drive, IMU, 180.0, 0.8),
    new DriveUntil(drive, IMU::isOnStation, 0.6, 0.0),
    new DriveUntil(drive, IMU::isCenteredOnStation, 0.37, 0.0),
    new AutoBalance(IMU, drive)
  );

  private final Command placeAndCommunity = new SequentialCommandGroup(
    IMU.resetVariables(),
    arm.toPosition(1.6, 1.55),
    new WaitUntilCommand(arm::doneMoving),
    new WaitCommand(0.2),
    intake.totalOutput(),
    new WaitCommand(0.4),
    arm.extendToLength(0),
    arm.liftToAngle(Constants.ARM_LIFT_MIN_ANGLE),
    new WaitUntilCommand(arm::doneMoving),
    new DriveStraight(drive, IMU, -3.0),
    new RotateDrive(drive, IMU, 180, 0.8)
  );

  // private final Command auto2 = new SequentialCommandGroup(
  //   new DriveStraight(drive, IMU, -.2, 0.6),
  //   new DriveStraight(drive, IMU, 4.5, 0.6)
  // );

  private final Command auto3 = new SequentialCommandGroup(
    IMU.resetVariables()
    // new RotateDrive(drive, IMU, 180),
    // arm.toPosition(1.6, 0.7),
    // new WaitCommand(3)
    // claw.toggle()
    //new DriveStraight(drive, IMU, -0.5, 0.4),
    //new RotateDrive(drive, IMU, 180)
  );

  private final Command noauto = new SequentialCommandGroup();
  SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonChooser.setDefaultOption("Community and balance", communityAuto);
    autonChooser.addOption("Just place", placeAuto);
    autonChooser.addOption("Place and community", placeAndCommunity);
    autonChooser.addOption("Place and balance", placeAndBalanceAuto);
    autonChooser.addOption("Testing", new RotateDrive(drive, IMU, 180, 0.8));
    autonChooser.addOption("No auto", noauto);

    SmartDashboard.putData(autonChooser);


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

    new JoystickButton(utilityJoystick, 11).onTrue(arm.toPosition(1.15, 0.5)); // FLOOR
    
    new JoystickButton(utilityJoystick, 12).onTrue(arm.toPosition(1.22, 1.15)); // MIDDLE
    
    new JoystickButton(utilityJoystick, 10).onTrue(arm.toPosition(1.6, 1.53)); // TOP
    
    new JoystickButton(utilityJoystick, 8).onTrue(arm.toPosition(Constants.ARM_STARTING_LENGTH, 1.38)); // DOUBLE SUBSTATION CONE
    new JoystickButton(utilityJoystick, 3).onTrue(arm.toPosition(Constants.ARM_STARTING_LENGTH, 1.23)); // DOUBLE SUBSTATION CUBE
     
    new JoystickButton(utilityJoystick, 7).onTrue(arm.extendToLength(0)); // INSIDE
    new JoystickButton(utilityJoystick, 7).onTrue(arm.liftToAngle(Constants.ARM_LIFT_MIN_ANGLE));
    
    new JoystickButton(utilityJoystick, 9).onTrue(arm.extendToLength(0)); // STOW
    new JoystickButton(utilityJoystick, 9).onTrue(arm.liftToAngle(55)); // STOW

    new JoystickButton(driveJoystick, 8).onTrue(new AprilTagRotate(drive, arm, IMU)); // auto orient

  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
    // return null;
  }

  public void disabledPeriodic() {
    arm.resetPID();
  }

  public Logger getLogger() {
    return logger;
  }

  // ExtendArm
  // LiftArm
  // AutoBalance
  // ClawGrab
}
