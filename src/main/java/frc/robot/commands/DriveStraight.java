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
public class DriveStraight extends CommandBase {
  private final Drive drive;
  private final IMU imu;

  private boolean isFinished = false;
  private double endMeters;
  private double startMeters;
  private double currentVelocity = 0.0;
  private double tolerance = 0.01;
  private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.AUTONOMOUS_ACCELERATION);
  double speed = 0.0;

  public DriveStraight(Drive drive, IMU imu, double meters) {
    this.drive = drive;
    this.imu = imu;
    this.endMeters = meters;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, imu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startMeters = drive.getAvgEncoderDistance();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double meters = drive.getAvgEncoderDistance() - startMeters;
    
    // Account for acceleration
    double timeToStop = currentVelocity / (Constants.AUTONOMOUS_ACCELERATION * -Math.signum(currentVelocity));
    double metersToStop = (currentVelocity * timeToStop) + (0.5 * Constants.AUTONOMOUS_ACCELERATION * -Math.signum(currentVelocity) * Math.pow(timeToStop, 2));
    
    if (currentVelocity == 0) {
      metersToStop = 0.0;
    }

    double velocity = 0.0;
    if (Math.abs(endMeters - meters) > Math.abs(metersToStop)) {
      velocity = Constants.AUTONOMOUS_SPEED * Math.signum(endMeters - meters);
    }

    if (Math.abs(endMeters - meters) < tolerance) {
      isFinished = true;
    }

    velocity = accelerationLimiter.calculate(velocity);

    drive.driveMetersPerSecond(velocity, velocity);
    currentVelocity = velocity;
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