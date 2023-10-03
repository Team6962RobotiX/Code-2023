// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  public CANSparkMax leftBank1 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax leftBank2 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_2, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax rightBank1 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax rightBank2 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_2, CANSparkMax.MotorType.kBrushless);

  private MotorControllerGroup rightBank = new MotorControllerGroup(rightBank1, rightBank2);
  private MotorControllerGroup leftBank = new MotorControllerGroup(leftBank1, leftBank2);

  private DifferentialDrive drive = new DifferentialDrive(leftBank, rightBank);

  private RelativeEncoder leftBankEncoder = leftBank2.getEncoder();
  private RelativeEncoder rightBankEncoder = rightBank1.getEncoder();

  private DifferentialDriveOdometry odometry;
  private Field2d field = new Field2d();

  private IMU IMU;  
  double speedCap = 1.0;

  public Drive(IMU IMU) {
    this.IMU = IMU;

    leftBank1.restoreFactoryDefaults();
    leftBank2.restoreFactoryDefaults();
    rightBank1.restoreFactoryDefaults();
    rightBank2.restoreFactoryDefaults();

    setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightBank.setInverted(true);

    leftBankEncoder.setPositionConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION);
    leftBankEncoder.setVelocityConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION / 60.0);
    rightBankEncoder.setPositionConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION);
    rightBankEncoder.setVelocityConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION / 60.0);


    resetEncoders();
    odometry = new DifferentialDriveOdometry(IMU.getRotation2d(), leftBankEncoder.getPosition(), -rightBankEncoder.getPosition());

    // SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    odometry.update(IMU.getRotation2d(), leftBankEncoder.getPosition(), -rightBankEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());
    if (!Constants.ENABLE_DRIVE) {
      drive.tankDrive(0, 0);
      return;
    }
    SmartDashboard.putData("Differential Drive", drive);    
    // This method will be called once per scheduler run
  }

  public DifferentialDriveKinematics getKinematics() {
    return new DifferentialDriveKinematics(Constants.TRACKWIDTH);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftBankEncoder.getVelocity(), -rightBankEncoder.getVelocity());
  }

  public double getHeading() {
    return IMU.getRotation2d().getDegrees();
  }

  public void driveMetersPerSecond(double left, double right) {
    double leftPower = Constants.driveSpeedToPower(left);
    double rightPower = Constants.driveSpeedToPower(right);
    if (Math.abs(leftPower) < 0.12) {
      leftPower = 0.125 * Math.signum(leftPower);
    }
    if (Math.abs(rightPower) < 0.12) {
      rightPower = 0.125 * Math.signum(rightPower);
    }
    tankDrive(leftPower, rightPower);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftBank.setVoltage(leftVolts);
    rightBank.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(IMU.getRotation2d(), leftBankEncoder.getPosition(), -rightBankEncoder.getPosition(), pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    leftBank1.setIdleMode(idleMode);
    leftBank2.setIdleMode(idleMode);
    rightBank1.setIdleMode(idleMode);
    rightBank2.setIdleMode(idleMode);
  }

  public void arcadeDrive(double straightPower, double turningPower) {
    if (!Constants.ENABLE_DRIVE) {
      return;
    }

    drive.arcadeDrive(straightPower, turningPower);
  }

  public void tankDrive(double leftBankSpeed, double rightBankSpeed) {
    if (!Constants.ENABLE_DRIVE) {
      return;
    }

    leftBankSpeed = Math.max(Math.min(leftBankSpeed, speedCap), -speedCap);
    rightBankSpeed = Math.max(Math.min(rightBankSpeed, speedCap), -speedCap);
    drive.tankDrive(leftBankSpeed, rightBankSpeed);
  }

  public void resetEncoders() {
    leftBankEncoder.setPosition(0);
    rightBankEncoder.setPosition(0);
  }

  public double getAvgEncoderDistance() {
    return (leftBankEncoder.getPosition() - rightBankEncoder.getPosition()) / 2.0;
  }

  public void setSpeedCap(double cap) {
    speedCap = cap;
  }
}
