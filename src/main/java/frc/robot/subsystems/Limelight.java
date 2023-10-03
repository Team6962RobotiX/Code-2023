package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private double lastKnownAprilTagZ = 0.0;

  public Limelight() {

  }

  @Override
  public void periodic() {
    Pose3d camSpace = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.TOP_LIMELIGHT_NAME);
    Pose3d targetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(Constants.TOP_LIMELIGHT_NAME);
    double z = Math.abs(targetSpace.getZ());
    if (z != 0) {
      lastKnownAprilTagZ = z;
    }
  }

  public double getLastKnownAprilTagZ() {
    return lastKnownAprilTagZ;
  }
}
