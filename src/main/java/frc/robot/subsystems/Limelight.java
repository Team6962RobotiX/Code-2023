package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private double lastKnownAprilTagZ = 0.0;

  public Limelight() {

  }

  @Override
  public void periodic() {
    Pose3d targetSpaceTop = LimelightHelpers.getCameraPose3d_TargetSpace(Constants.TOP_LIMELIGHT_NAME);
    Pose3d targetSpaceBottom = LimelightHelpers.getCameraPose3d_TargetSpace(Constants.BOTTOM_LIMELIGHT_NAME);

    double z_top = Math.abs(targetSpaceTop.getZ());
    double z_bottom = Math.abs(targetSpaceBottom.getZ());

    if (z_top != 0 && z_bottom != 0) {
      lastKnownAprilTagZ = (z_top+z_bottom)/2;
    }else if (z_top != 0){
      lastKnownAprilTagZ = z_top;
    }else if (z_bottom != 0){
      lastKnownAprilTagZ = z_bottom;
    }
  }

  public double getLastKnownAprilTagZ() {
    return lastKnownAprilTagZ;
  }
}
