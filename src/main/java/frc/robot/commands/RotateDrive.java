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
  private double endRadians;
  private double startRadians;
  private double currentAngularVelocity = 0.0;
  private double tolerance = 0.01;
  private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.AUTONOMOUS_ANGULAR_ACCELERATION);

  public RotateDrive(Drive drive, IMU imu, double radians) {
    this.drive = drive;
    this.imu = imu;
    this.endRadians = radians;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, imu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startRadians = imu.getIMU().getAngle() / 180.0 * Math.PI;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double radians = imu.getIMU().getAngle() / 180.0 * Math.PI - startRadians;

    System.out.println(radians);
    
    // Account for acceleration
    double timeToStop = currentAngularVelocity / (Constants.AUTONOMOUS_ANGULAR_ACCELERATION * -Math.signum(currentAngularVelocity));
    double radiansToStop = (currentAngularVelocity * timeToStop) + (0.5 * Constants.AUTONOMOUS_ANGULAR_ACCELERATION * -Math.signum(currentAngularVelocity) * Math.pow(timeToStop, 2));
    
    if (currentAngularVelocity == 0) {
      radiansToStop = 0.0;
    }

    double angularVelocity = 0.0;
    if (Math.abs(endRadians - radians) > Math.abs(radiansToStop)) {
      angularVelocity = Constants.AUTONOMOUS_ANGULAR_SPEED * Math.signum(endRadians - radians);
    }

    if (Math.abs(endRadians - radians) < tolerance) {
      isFinished = true;
    }

    angularVelocity = accelerationLimiter.calculate(angularVelocity);
    double driveVelocity = Constants.rotationalSpeedToDriveSpeed(angularVelocity);

    drive.driveMetersPerSecond(driveVelocity, -driveVelocity);
    currentAngularVelocity = angularVelocity;
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