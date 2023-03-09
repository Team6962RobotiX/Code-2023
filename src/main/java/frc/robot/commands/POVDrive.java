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

public class POVDrive extends CommandBase {
  private final Drive drive;
  private final double POV;

  public POVDrive(Drive drive, double POV) {
    this.drive = drive;
    this.POV = POV;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(0, 0);
    
    if (POV == 0  ) {
      drive.arcadeDrive(Constants.DRIVE_FINE_CONTROL_POWER, 0);
    }
    if (POV == 45 ) {
      drive.arcadeDrive(Constants.DRIVE_FINE_CONTROL_POWER, -Constants.DRIVE_FINE_CONTROL_POWER);
    }
    if (POV == 90 ) {
      drive.arcadeDrive(0, -Constants.DRIVE_FINE_CONTROL_POWER);
    }
    if (POV == 135) {
      drive.arcadeDrive(-Constants.DRIVE_FINE_CONTROL_POWER, -Constants.DRIVE_FINE_CONTROL_POWER);
    }
    if (POV == 180) {
      drive.arcadeDrive(-Constants.DRIVE_FINE_CONTROL_POWER, 0);
    }
    if (POV == 225) {
      drive.arcadeDrive(-Constants.DRIVE_FINE_CONTROL_POWER, Constants.DRIVE_FINE_CONTROL_POWER);
    }
    if (POV == 270) {
      drive.arcadeDrive(0, Constants.DRIVE_FINE_CONTROL_POWER);
    }
    if (POV == 315) {
      drive.arcadeDrive(Constants.DRIVE_FINE_CONTROL_POWER, Constants.DRIVE_FINE_CONTROL_POWER);
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
