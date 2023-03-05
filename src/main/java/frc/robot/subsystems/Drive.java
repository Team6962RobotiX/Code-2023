// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  CANSparkMax leftBank1 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  CANSparkMax leftBank2 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_2, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightBank1 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightBank2 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_2, CANSparkMax.MotorType.kBrushless);

  MotorControllerGroup rightBank = new MotorControllerGroup(rightBank1, rightBank2);
  MotorControllerGroup leftBank = new MotorControllerGroup(leftBank1, leftBank2);

  DifferentialDrive drive = new DifferentialDrive(leftBank, rightBank);

  RelativeEncoder leftBankEncoder = leftBank1.getEncoder();
  RelativeEncoder rightBankEncoder = rightBank1.getEncoder();

  private DifferentialDriveOdometry odometry;
  private Field2d field = new Field2d();

  private IMU IMU;

  public Drive() {
    if (!Constants.ENABLE_DRIVE) {
      System.out.println("Drive Disabled");
      return;
    }

    leftBank1.restoreFactoryDefaults();
    leftBank2.restoreFactoryDefaults();
    rightBank1.restoreFactoryDefaults();
    rightBank2.restoreFactoryDefaults();

    setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftBank.setInverted(true);

    leftBankEncoder.setPositionConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION);
    rightBankEncoder.setPositionConversionFactor(Constants.DRIVE_METERS_PER_REVOLUTION);

    resetEncoders();

    IMU = new IMU();

    odometry = new DifferentialDriveOdometry(IMU.getRotation2d(), leftBankEncoder.getPosition(), rightBankEncoder.getPosition());

    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Differential Drive", drive);
  }

  @Override
  public void periodic() {
    odometry.update(IMU.getRotation2d(), leftBankEncoder.getPosition(), rightBankEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftBankEncoder.getVelocity(), rightBankEncoder.getVelocity());
  }

  public double getHeading() {
    return IMU.getRotation2d().getDegrees();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftBank.setVoltage(leftVolts);
    rightBank.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(IMU.getRotation2d(), leftBankEncoder.getPosition(), rightBankEncoder.getPosition(), pose);
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

    drive.tankDrive(leftBankSpeed, rightBankSpeed);
  }

  public void resetEncoders() {
    leftBankEncoder.setPosition(0);
    rightBankEncoder.setPosition(0);
  }
}
