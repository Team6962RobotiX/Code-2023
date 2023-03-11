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
    private PIDController orientPID = new PIDController(Constants.DRIVE_ORIENT_KP, 0, 0);

    /** An example command that uses an example subsystem. */
    public AutoOrient(Limelight camera, Drive drive, Arm arm) {
        this.camera = camera;
        this.drive = drive;
        this.arm = arm;
        orientPID.setTolerance(1);
        orientPID.setSetpoint(0);
        addRequirements(camera, drive, arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double targetPosZ = 0;
        double targetPosY = 0;
        double tx = 0;

        if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME)) {
            if (camera.getTargetingResults().targets_Retro.length == 0) {
                return;
            }

            LimelightHelpers.LimelightTarget_Retro target = camera.getTargetingResults().targets_Retro[0];
            // targetPosZ = target.getTargetPose_CameraSpace().getZ();
            // targetPosY = target.getTargetPose_CameraSpace().getY();

        } else if (camera.getName().equals(Constants.BOTTOM_LIMELIGHT_NAME)) {
            if (camera.getTargetingResults().targets_Detector.length == 0) {
                return;
            }

            targetPosZ = 1;
            targetPosY = 0;
        }

        tx = LimelightHelpers.getTX(camera.getName());
        
        double PIDPower = orientPID.calculate(tx);
        drive.arcadeDrive(0, Constants.DRIVE_BASE_TURN_POWER * Math.signum(PIDPower) + PIDPower);

        double[] pose = LimelightHelpers.getTargetPose_CameraSpace(Constants.TOP_LIMELIGHT_NAME);

        for (Double x : pose) {
            System.out.println(x);
        }
        
        // arm.setTargetPosition(targetPosX, targetPosY);
        // System.out.println("targetPosZ");
        // System.out.println(targetPosZ);
        // System.out.println("targetPosY");
        // System.out.println(targetPosY);

        // arm.setTargetPosition(targetPosZ, targetPosY);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
