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
    this.limelight = limelight;

    addRequirements();
  }
  
  @Override
  public void execute() {
    // System.out.print("MaxSpeed - ");
    // System.out.println(getMaxSpeed());

    drive.setSpeedCap(1 - Math.pow(1 - getMaxSpeed(), 2));

    // System.out.print("SpeedCap - ");
    // System.out.println(drive.getSpeedCap());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setSpeedCap(1);
  }
  
  public double getMaxSpeed() {
    double z = limelight.getLastKnownAprilTagZ();
    double maxSpeed = z / 17 + Constants.DRIVE_FINE_CONTROL_POWER;
    return maxSpeed;
  }
}