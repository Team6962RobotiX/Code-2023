package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private double cameraHeight = 0.0;
    private double cameraAngle = 0.0;
    private double cameraPixels = 0.0;
    private double focalLength = 0.0;

    private double heightMid = 0.0;
    private double heightTall = 0.0;
    private double realArea = 0.0;

    private double dist = 0.0;

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
        double targetPosZ = 0.0;
        double targetPosY = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        double ta = 0.0;

        if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME)) {
            if (camera.getTargetingResults().targets_Retro.length == 0) {
                return;
            }

            LimelightHelpers.LimelightTarget_Retro target = camera.getTargetingResults().targets_Retro[0];

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

        // double[] pose = LimelightHelpers.getCameraPose_TargetSpace(Constants.TOP_LIMELIGHT_NAME);

        // System.out.println(pose.length);

        // ---------------------------------------------------------------

        // Pose3d pose3D = new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(Units.degreesToRadians(pose[3]), Units.degreesToRadians(pose[4]), Units.degreesToRadians(pose[5])));

        // System.out.println(pose3D.getX());
        // System.out.println(pose3D.getY());
        // System.out.println(pose3D.getZ());

        // ---------------------------------------------------------------

        ty = LimelightHelpers.getTY(camera.getName());
        ta = LimelightHelpers.getTA(camera.getName());

        double forwardDistanceApprox = getForwardDistanceApprox(ta);
        System.out.println(forwardDistanceApprox);
        double targetHeight = determineTargetHeight(forwardDistanceApprox, ty);
        System.out.println(targetHeight);
        double forwardDistancePrecise = getForwardDistancePrecise(targetHeight, ty);
        System.out.println(forwardDistancePrecise);

        dist = forwardDistancePrecise;

        // double[] defaultPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // double[] pose = NetworkTableInstance.getDefault().getTable(Constants.TOP_LIMELIGHT_NAME).getEntry("camerapose_targetspace").getDoubleArray(defaultPose);
        
        // System.out.println("Distance");
        // System.out.println(Math.abs(pose[1]));
        // System.out.println("Height");
        // System.out.println(Math.abs(pose[0]) + Constants.TOP_LIMELIGHT_HEIGHT);
        // System.out.println(pose[2]);
        
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

    public double getForwardDistanceApprox(double areaPercentage) {
        double forwardDistance = focalLength * Math.sqrt(realArea / (areaPercentage * cameraPixels));
        return forwardDistance;
    }

    public double determineTargetHeight(double forwardDistanceApprox, double targetAngle) {
        double targetAngleRad = targetAngle * (Math.PI / 180.0);
        double cameraAngleRad = cameraAngle * (Math.PI / 180.0);

        // (h2 - h1) = tan(a1 + a2) * d
        double targetHeightApprox = (Math.tan(targetAngleRad + cameraAngleRad) * forwardDistanceApprox) + cameraHeight;

        if (Math.abs(targetHeightApprox - heightTall) < Math.abs(targetHeightApprox - heightMid)) {
            return heightTall;
        } else {
            return heightMid;
        }
    }

    public double getForwardDistancePrecise(double targetHeight, double targetAngle) {
        double targetAngleRad = targetAngle * (Math.PI / 180.0);
        double cameraAngleRad = cameraAngle * (Math.PI / 180.0);

        double forwardDistance = (targetHeight - cameraHeight) / Math.tan(targetAngleRad + cameraAngleRad);
        return forwardDistance;
    }

    public double getForwardDistance() {
        return dist;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
