// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoBalance extends CommandBase {
  private final IMU IMU;
  private final Drive drive;

  public AutoBalance(IMU IMU, Drive drive) {
    this.IMU = IMU;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IMU, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = IMU.getRoll();
    if (Math.abs(roll) > Constants.BALANCE_LEVEL_ANGLE_RANGE) {
      double power = ((roll / 360 * Constants.BALANCE_ANGLE_POWER_MULTIPLE) + (Constants.BALANCE_BASE_POWER * Math.signum(roll)));
      drive.tankDrive(power, power);
    }
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
