// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!Constants.ENABLE_DRIVE) {
      return;
    }

    double straightAxis = driveJoystick.getRawAxis(1);
    double twistAxis = driveJoystick.getRawAxis(2);

    double straightPower = mapPower(straightAxis, Constants.DRIVE_BASE_POWER, Constants.DRIVE_POWER_LIMIT, Constants.STRAIGHT_DEADZONE);
    double turningPower = mapPower(twistAxis, 0, Constants.DRIVE_POWER_LIMIT, Constants.TWIST_DEADZONE);

    drive.arcadeDrive(straightPower, turningPower);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
