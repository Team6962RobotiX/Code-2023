// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AprilTagRotate extends CommandBase {
  
  private Drive drive;
  private Arm arm;
  private IMU imu;

  private double x_cone_tag = -0.99;

  private Pose3d targetPos;
  private Pose3d camPos;

  public AprilTagRotate(Drive drive, Arm arm, IMU imu) {
    this.imu = imu;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(Constants.TOP_LIMELIGHT_NAME, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPos = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.TOP_LIMELIGHT_NAME);
    camPos = LimelightHelpers.getCameraPose3d_TargetSpace(Constants.TOP_LIMELIGHT_NAME);
    double x_lime_tag = camPos.getX();
    System.out.println("x_lime_tag: " + x_lime_tag);
    double z_lime_tag = Math.abs(camPos.getZ());
    System.out.println("z_lime_tag: " + z_lime_tag);
    double radrotate_tag_wall = Math.atan2(x_lime_tag, z_lime_tag);
    System.out.println("radrotate_tag_wall: " + radrotate_tag_wall);
    double radrotate_lime_tag = targetPos.getRotation().getY();
    System.out.println("radrotate_lime_tag:" + radrotate_lime_tag);
    double radrotate_cone = Math.atan2(x_cone_tag-x_lime_tag, z_lime_tag);
    System.out.println("radrotate_cone: " + radrotate_cone);
    double total_rotate = radrotate_cone+radrotate_lime_tag;
    RotateDriveOld rotatoer = new RotateDriveOld(drive, imu, total_rotate);
    // System.out.println(total_rotate);
    rotatoer.schedule();
    //DriveStraight forwarder = DriveStraight(drive, imu, distance, drivePower)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
