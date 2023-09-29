// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public final class TankAuton {
  public static Command fullAuto(String pathName, HashMap<String, Command> eventMap, Drive tankDrive) {
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(Constants.AUTONOMOUS_SPEED, Constants.AUTONOMOUS_ACCELERATION));
    
    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
      tankDrive::getPose,
      tankDrive::resetPose,
      Constants.AUTONOMOUS_PID,
      tankDrive.getKinematics(),
      tankDrive::driveMetersPerSecond,
      eventMap,
      tankDrive
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, Drive tankDrive) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if(isFirstPath){
              tankDrive.resetPose(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            tankDrive::getPose,
            Constants.AUTONOMOUS_PID,
            tankDrive.getKinematics(),
            tankDrive::driveMetersPerSecond,
            tankDrive
        )
    );
  }

  public Command toPosition(Translation2d pose, Rotation2d rotation, Drive tankDrive) {
    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)), // position, heading
        new PathPoint(pose, rotation) // position, heading
    );
    return followTrajectoryCommand(traj, true, tankDrive);
  }
}