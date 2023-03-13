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
public class DriveStraight extends CommandBase {
  private final Drive drive;
  private final IMU imu;

  private double startDistance;
  private double distanceTolerance = 0.05;
  private double desiredDistance = 0;
  private double drivePower;
  private boolean isFinished = false;

  private boolean rollCheck = false;
  private double rollCuttoff = 12.0;

  public DriveStraight(Drive drive, IMU imu, double desiredDistance, double drivePower) {
    this.drive = drive;
    this.imu = imu;
    this.drivePower = drivePower;
    this.desiredDistance = desiredDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, imu);
  }

  public DriveStraight(Drive drive, IMU imu, double desiredDistance, double drivePower, boolean rollCheck) {
    this.drive = drive;
    this.imu = imu;
    this.drivePower = drivePower;
    this.desiredDistance = desiredDistance;
    this.rollCheck = rollCheck;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, imu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drive.resetEncoders();
    startDistance = drive.getLeftBankEncoder();
    System.out.println("RUNNING");
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {      

    if (rollCheck) {
      drive.arcadeDrive(drivePower, 0);
      if (imu.getIMU().getRoll() > rollCuttoff || imu.getIMU().getRoll() < -rollCuttoff) {
        isFinished = true;
      }
    }
    else {
      if (drive.getLeftBankEncoder() - startDistance < (desiredDistance - distanceTolerance)) {
        //System.out.println("Forward");
        drive.arcadeDrive(drivePower, 0);
      } else if (drive.getLeftBankEncoder() - startDistance > (desiredDistance + distanceTolerance)) {
        //System.out.println("Backward");
        drive.arcadeDrive(-drivePower, 0);
      } else {
        isFinished = true;
      }
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
    return isFinished;
  }
}
