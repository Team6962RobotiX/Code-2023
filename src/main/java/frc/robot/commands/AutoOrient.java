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

    private double cameraHeight = 1.19;
    private double cameraAngle = -0.5;
    private double cameraPixels = 640*480;
    // private double focalLength = 0.690;

    private double heightMid = 0.606; // Hopefully this is accurate to the arena but this value for now is just for testing with our PVC pipe construction of the game field (Applies to the next variable as well)
    private double heightTall = 1.115;
    private double realArea = (0.0318/2) * 0.1016; // This value is also specific to our field element prototypes and not the actual game elements
    private double nodeHeight = 0.0;

    // following variable isn't really for the execute method in this class it's more arm related
    private double dist = 0.0;

    /** An example command that uses an example subsystem. */
    // reinsert drive and arm arguments
    public AutoOrient(Limelight camera, Drive drive, Arm arm) {
        this.camera = camera;
        this.drive = drive;
        this.arm = arm;
        orientPID.setTolerance(1);
        orientPID.setSetpoint(0);
        addRequirements(camera);
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

        double targetHeight = determineTargetHeight(ty);
        // System.out.println(targetHeight);
        double forwardDistancePrecise1 = getForwardDistancePrecise(heightMid, ty);
        double forwardDistancePrecise2 = getForwardDistancePrecise(heightTall, ty);
        double forwardDistancePrecise = getForwardDistancePrecise(targetHeight, ty);
        System.out.println("Distance: " + forwardDistancePrecise + ", Node Height: " + targetHeight);

        dist = forwardDistancePrecise;

        /* Upcoming commands:
            (NOTE: NODE_TIP_HEIGHT_X is the height of the tip of the cone node, depending on the targetHeight, and distToArmJoint is the (vertical, so far) distance from the camera to the joint of the arm that needs to be moved)
         *  double rotationNeeded = Math.atan((Constants.NODE_TIP_HEIGHT_X - (cameraHeight - distToArmJoint)) / dist) + (90 - startAngle) + EPSILON; <-- epsilon being a certain amount of extra angle we'd want to use to go OVER the node before dropping the game piece DOWN into it
         *  double extensionNeeded = Math.sqrt(Math.pow(dist, 2) + Math.pow(Constants.NODE_TIP_HEIGHT_X - (cameraHeight - distToArmJoint), 2));
         *  ---------- Next line is pure pseudocode; fill in with the real extension commands ---------- 
         *  arm.extend(extensionNeeded, rotationNeeded) <-- polar form
        */

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

    // public double getForwardDistanceApprox(double areaPercentage) {
    //     double forwardDistance = focalLength * Math.sqrt(realArea / ((areaPercentage / 100) * cameraPixels));
    //     return forwardDistance;
    // }

    public double determineTargetHeight(double targetAngle) {
        // double targetAngleRad = targetAngle * (Math.PI / 180.0);
        // double cameraAngleRad = cameraAngle * (Math.PI / 180.0);

        // (h2 - h1) = tan(a1 + a2) * d
        // double targetHeightApprox = (Math.tan(targetAngleRad + cameraAngleRad) * forwardDistanceApprox) + cameraHeight;

        if (Math.abs(targetAngle) < 6.0) {
            return heightTall;
            // nodeHeight = Constants.NODE_TIP_HEIGHT_TOP;
        } else {
            return heightMid;
            // nodeHeight = Constants.NODE_TIP_HEIGHT_MID;
        }
    }

    public double getForwardDistancePrecise(double targetHeight, double targetAngle) {
        double targetAngleRad = targetAngle * (Math.PI / 180.0);
        double cameraAngleRad = cameraAngle * (Math.PI / 180.0);

        double forwardDistance = (targetHeight - cameraHeight) / Math.tan(targetAngleRad + cameraAngleRad);
        return forwardDistance;
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
