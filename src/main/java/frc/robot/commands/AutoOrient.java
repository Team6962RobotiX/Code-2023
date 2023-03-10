package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.lang.Math;

public class AutoOrient extends CommandBase {

    private Limelight camera;
    private Drive drive;
    private Arm arm;
    private LimelightHelpers.LimelightTarget_Retro centerNode;
    private PIDController orientPID = new PIDController(Constants.DRIVE_ORIENT_KP, 0, 0);

    private LimelightHelpers.LimelightTarget_Retro target;
    private String level;

    /** An example command that uses an example subsystem. */
    public AutoOrient(Limelight camera, Drive drive, Arm arm, String level) {
        this.camera = camera;
        this.drive = drive;
        this.arm = arm;
        this.level = level;

        addRequirements(camera, drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME)) {
            if (level.equals("TOP")) {
                target = getTopTarget();
            } else if (level.equals("MID")) {
                target = getMidTarget();
            }
        } else if (camera.getName().equals(Constants.BOTTOM_LIMELIGHT_NAME)) {

        }

        double targetPosX = target.getTargetPose_CameraSpace().getX();
        double targetPosY = target.getTargetPose_CameraSpace().getY();
        double PIDPower = orientPID.calculate(targetPosX);
        // arm.setTargetPosition(targetPosX, targetPosY);
        drive.tankDrive(PIDPower, -PIDPower);
        arm.setTargetPosition(targetPosX, targetPosY);
    }

    public LimelightHelpers.LimelightTarget_Retro getMidTarget() {
        LimelightHelpers.LimelightTarget_Retro[] retro = camera.getTargetingResults().targets_Retro;
        LimelightHelpers.LimelightTarget_Retro midTarget = retro[0];

        for (int i = 0; i < retro.length; i++) {
            if (retro[i].getTargetPose_CameraSpace().getY() + Constants.TOP_LIMELIGHT_HEIGHT < (Constants.NODE_TAPE_HEIGHT_MID + Constants.NODE_TAPE_HEIGHT_TOP) / 2) {
                continue;
            }
            if (Math.abs(retro[i].tx) < Math.abs(midTarget.tx)) {
                midTarget = retro[i];
            }
        }

        return midTarget;
    }

    public LimelightHelpers.LimelightTarget_Retro getTopTarget() {
        LimelightHelpers.LimelightTarget_Retro[] retro = camera.getTargetingResults().targets_Retro;
        LimelightHelpers.LimelightTarget_Retro topTarget = retro[0];

        for (int i = 0; i < retro.length; i++) {
            if (retro[i].getTargetPose_CameraSpace().getY() + Constants.TOP_LIMELIGHT_HEIGHT > (Constants.NODE_TAPE_HEIGHT_MID + Constants.NODE_TAPE_HEIGHT_TOP) / 2) {
                continue;
            }
            if (Math.abs(retro[i].tx) < Math.abs(topTarget.tx)) {
                topTarget = retro[i];
            }
        }

        return topTarget;
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
