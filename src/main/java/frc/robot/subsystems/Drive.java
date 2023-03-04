// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  CANSparkMax leftBank1, leftBank2, rightBank1, rightBank2;
  RelativeEncoder rightBankEncoder, leftBankEncoder;
  MotorControllerGroup rightBank, leftBank;
  DifferentialDrive drive;

  public Drive() {
    if (!Constants.ENABLE_DRIVE) {
      System.out.println("Drive Disabled");
      return;
    }

    leftBank1 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
    leftBank2 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_2, CANSparkMax.MotorType.kBrushless);
    rightBank1 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
    rightBank2 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_1, CANSparkMax.MotorType.kBrushless);

    leftBank1.restoreFactoryDefaults();
    leftBank2.restoreFactoryDefaults();
    rightBank1.restoreFactoryDefaults();
    rightBank2.restoreFactoryDefaults();

    leftBank1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftBank2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightBank1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightBank2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightBank = new MotorControllerGroup(rightBank1, rightBank2);
    leftBank = new MotorControllerGroup(leftBank1, leftBank2);
    leftBank.setInverted(true);

    drive = new DifferentialDrive(leftBank, rightBank);

    leftBankEncoder = leftBank1.getEncoder();
    rightBankEncoder = rightBank1.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    drive.arcadeDrive(straightPower, turningPower);
  }

  public void tankDrive(double leftBankSpeed, double rightBankSpeed) {
    drive.tankDrive(leftBankSpeed, rightBankSpeed);
  }
}
