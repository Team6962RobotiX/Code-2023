// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import java.lang.Math;

public class Limelight extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private String name;

  public Limelight(String name) {
    if (!Constants.ENABLE_LIMELIGHT) {
      System.out.println("Vision Disabled");
      return;
    }
    this.name = name;
  }

  @Override
  public void periodic() {
    limelightData = LimelightHelpers.getLatestResults(name);
  }

  public LimelightHelpers.Results getTargetingResults() {
    return limelightData.targetingResults;
  }

  public String getName() {
    return name;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
