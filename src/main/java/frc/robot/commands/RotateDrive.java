// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateDrive extends CommandBase {
  private final Drive drive;
  private final IMU imu;

  private boolean isFinished = false;
  private double endDegrees;
  private double startDegrees;
  private double power;

  public RotateDrive(Drive drive, IMU imu, double degrees, double power) {
    this.drive = drive;
    this.power = power;
    this.imu = imu;
    this.endDegrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, imu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startDegrees = imu.getIMU().getAngle();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degrees = imu.getIMU().getAngle() - startDegrees;
    double angularVelocity = imu.getIMU().getRawGyroZ();
    double tolerance = angularVelocity * Constants.GYRO_DELAY;
    
    if (endDegrees - degrees < Math.abs(tolerance)) {
      isFinished = true;
      drive.tankDrive(0, 0);
    }

    drive.arcadeDrive(0, -power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}