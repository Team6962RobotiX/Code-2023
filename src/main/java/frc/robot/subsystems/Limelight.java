// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  LimelightHelpers.LimelightResults topLimelightData;
  LimelightHelpers.LimelightResults bottomLimelightData;

  public Limelight() {
    if (!Constants.ENABLE_LIMELIGHT) {
      System.out.println("Vision Disabled");
      return;
    }
  }

  @Override
  public void periodic() {
    // topLimelightData = LimelightHelpers.getLatestResults(Constants.TOP_LIMELIGHT_NAME);
    // bottomLimelightData = LimelightHelpers.getLatestResults(Constants.BOTTOM_LIMELIGHT_NAME);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
