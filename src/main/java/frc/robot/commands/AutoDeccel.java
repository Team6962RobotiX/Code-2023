package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    drive.setSpeedCap(1 - Math.pow(1 - getMaxSpeed(), 1));

    System.out.print("SpeedCap - ");
    SmartDashboard.putNumber("getLastKnownAprilTagZ", limelight.getLastKnownAprilTagZ());
    System.out.println(drive.getSpeedCap());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setSpeedCap(1);
  }
  
  public double getMaxSpeed() {
    double z = limelight.getLastKnownAprilTagZ();
    double initialDist = 4;
    double initialSpeed = 0.9;
    double finalDist = 1.68;
    double finalSpeed = 0.36;
    double maxSpeed = ((finalSpeed - initialSpeed)/(finalDist - initialDist)) *(z - initialDist) + initialSpeed;
    maxSpeed = maxSpeed < Constants.DRIVE_FINE_CONTROL_POWER ? Constants.DRIVE_FINE_CONTROL_POWER : maxSpeed;
    System.out.println(z);
    return maxSpeed;
  }
}