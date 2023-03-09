// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase {
  private final Drive drive;
  private final Supplier<Double> joystickStraight, joystickTurn;

  public JoystickDrive(Drive drive, Supplier<Double> joystickStraight, Supplier<Double> joystickTurn) {
    this.drive = drive;
    this.joystickStraight = joystickStraight;
    this.joystickTurn = joystickTurn;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double straightAxis = -joystickStraight.get();
    double twistAxis = -joystickTurn.get();

    double straightPower = Constants.mapPower(straightAxis, Constants.DRIVE_BASE_POWER, Constants.DRIVE_POWER_LIMIT, Constants.STRAIGHT_DEADZONE);
    double turningPower = Constants.mapPower(twistAxis, 0, Constants.DRIVE_TURN_POWER_LIMIT, Constants.TWIST_DEADZONE);

    drive.arcadeDrive(straightPower, turningPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
