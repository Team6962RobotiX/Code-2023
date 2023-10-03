package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Limelight;

public class AutoDeccel extends CommandBase {
  private Drive drive;
  private Arm arm;
  private IMU imu;
  private Limelight limelight;

  public AutoDeccel(Drive drive, IMU imu, Limelight limelight) {
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
    double z = limelight.getLastKnownAprilTagZ();
    double maxSpeed = z / 30 + Constants.DRIVE_BASE_POWER;
    return maxSpeed;
  }
}