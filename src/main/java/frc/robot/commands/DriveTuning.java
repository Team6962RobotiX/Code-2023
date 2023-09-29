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
public class DriveTuning extends CommandBase {
  private final Drive drive;
  private final IMU imu;

  private boolean isFinished = false;
  private double maxPower = 0.7;
  private double powerStep = 0.1;

  private double startTime = 0.0;
  private int currentStep = 0;

  private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.2);

  public DriveTuning(Drive drive, IMU imu) {
    this.drive = drive;
    this.imu = imu;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = System.currentTimeMillis() - startTime;
    
    if ((int) Math.floor(time / 1000.0) != currentStep) {
        System.out.println("Power: " + powerStep * currentStep);
        System.out.println("Speed: " + drive.getWheelSpeeds().leftMetersPerSecond);
    }

    currentStep = (int) Math.floor(time / 1000.0);
    double power = powerStep * currentStep;
    
    if (power > maxPower) {
        power = 0.0;
        if (drive.getWheelSpeeds().leftMetersPerSecond == 0) {
            isFinished = true;
            return;
        }
    }

    power = accelerationLimiter.calculate(power);

    drive.tankDrive(power, power);

    // drive.driveMetersPerSecond(velocity, velocity);
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