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
public class RotateDrive extends CommandBase {
  private final Drive drive;
  private final IMU IMU;

  private float angle;
  private double threshold = 0.05;

  private float initialYaw;
  private float desiredYaw;

  private double drivingPower = 0.3;


  public RotateDrive(Drive drive, IMU IMU, float angle) {
    this.drive = drive;
    this.IMU = IMU;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, IMU);  
  }

  public RotateDrive(Drive drive, IMU IMU, float angle, double power) {
    this.drive = drive;
    this.IMU = IMU;
    this.angle = angle;
    this.drivingPower = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, IMU);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialYaw = IMU.getIMU().getYaw();
    desiredYaw = angle+initialYaw;
  }

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    float yaw = IMU.getIMU().getYaw();
    if (yaw < desiredYaw-threshold) {
      drive.arcadeDrive(0, drivingPower);
    } else if (yaw > desiredYaw+threshold) {
      drive.arcadeDrive(0, -drivingPower);
    }

    System.out.println(IMU.getIMU().getYaw());
    drive.arcadeDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getLeftBankEncoder() > 1;
  }
}

