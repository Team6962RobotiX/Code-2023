package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IMU;

public class AutoDeccel extends CommandBase {
  private Drive drive;
  private Arm arm;
  private IMU imu;

  public AutoDeccel(Drive drive, IMU imu) {
    this.imu = imu;
    this.drive = drive;

    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    drive.setSpeedCap(getMaxSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setSpeedCap(1.0);
  }
  
  public double getMaxSpeed() {
    Pose3d camSpace = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.TOP_LIMELIGHT_NAME);
    Pose3d targetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(Constants.TOP_LIMELIGHT_NAME);
    double z = Math.abs(targetSpace.getZ());
    double maxSpeed = z / 30 + Constants.DRIVE_BASE_POWER;
    return maxSpeed;
  }
}