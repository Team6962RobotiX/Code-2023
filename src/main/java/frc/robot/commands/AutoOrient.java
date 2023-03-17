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

//import org.json.simple.parser.JSONParser;
//import org.json.simple.parser.ParseException;

import java.lang.Math;
import java.util.Arrays;

public class AutoOrient extends CommandBase {

    private Limelight camera;
    private Drive drive;
    private Arm arm;
    private PIDController orientPID = new PIDController(Constants.DRIVE_ORIENT_KP, 0, 0);

    private double cameraHeight;
    private double cameraAngle = 0.0;
    // private double cameraPixels = 2592*1944;
    // private double focalLength = 0.690;

    private double targetHeight;
    private double heightMid = Constants.NODE_TAPE_HEIGHT_MID; // Hopefully this is accurate to the arena but this value for now is just for testing with our PVC pipe construction of the game field (Applies to the next variable as well)
    private double heightTall = Constants.NODE_TAPE_HEIGHT_TOP;
    private double realArea = (0.0318/2) * 0.1016; // This value is also specific to our field element prototypes and not the actual game elements
    private double nodeHeight = 0.0;

    // following variable isn't really for the execute method in this class it's more arm related
    private double dist = 0.0;

    /** An example command that uses an example subsystem. */
    // reinsert drive and arm arguments
    public AutoOrient(Limelight camera, Drive drive, Arm arm) {
        this.camera = camera;
        if (camera.getName().equals(Constants.BOTTOM_LIMELIGHT_NAME)) {
            cameraHeight = Constants.BOTTOM_LIMELIGHT_HEIGHT;
        } else 
        // if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME))
        {
            cameraHeight = Constants.TOP_LIMELIGHT_HEIGHT;
        }
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
        double typ = 0.0;
        double ta = 0.0;

        // if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME)) {
        //     // // if (camera.getTargetingResults().targets_Retro.length == 0) {
        //     // //     return;
        //     // // }
        //     // double [] corners = LimelightHelpers.getCorners(Constants.TOP_LIMELIGHT_NAME);
        //     // System.out.println("TOP:" + Arrays.toString(corners));

        //    // System.out.println(LimelightHelpers.getCorners(Constants.TOP_LIMELIGHT_NAME)[0]);

        // } 
        if (camera.getName().equals(Constants.BOTTOM_LIMELIGHT_NAME)) {
            // if (camera.getTargetingResults().targets_Detector.length == 0) {
            //     return;
            // }

            // double [] corners = LimelightHelpers.getCorners(Constants.BOTTOM_LIMELIGHT_NAME);
            // System.out.println("BOTTOM:" + Arrays.toString(corners));

            
            targetPosZ = Constants.NODE_TAPE_HEIGHT_MID;
            targetPosY = 0;
        }

        tx = LimelightHelpers.getTX(camera.getName());
        System.out.println("tx: " + tx);        
        
        //double PIDPower = orientPID.calculate(tx);
        //drive.arcadeDrive(0, Constants.DRIVE_BASE_TURN_POWER * Math.signum(PIDPower) + PIDPower);
        
        int num_detected = camera.getTargetingResults().targets_Detector.length;

        ty = LimelightHelpers.getTY(camera.getName());


        if (num_detected > 0){
            double [] corners = LimelightHelpers.getCorners(camera.getName());
            System.out.println(camera.getName() + " " + Arrays.toString(corners));
            double angle = getBottomAngle(ty, typ, corners);
            determineTargetHeight(targetPosY);
            double distance = getForwardDistancePrecise(targetHeight, angle);
            System.out.println("Distance: " + distance + "meters");
        }



    

        // double[] pose = LimelightHelpers.getCameraPose_TargetSpace(Constants.TOP_LIMELIGHT_NAME);

        // System.out.println(pose.length);

        // ---------------------------------------------------------------

        // Pose3d pose3D = new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(Units.degreesToRadians(pose[3]), Units.degreesToRadians(pose[4]), Units.degreesToRadians(pose[5])));

        // System.out.println(pose3D.getX());
        // System.out.println(pose3D.getY());
        // System.out.println(pose3D.getZ());

        // ---------------------------------------------------------------

        // if (num_detected > 0) {
        //     typ = camera.getTargetingResults().targets_Detector[0].ty_pixels; // LimelightHelpers.getTYP(camera.getName());
        //     double[] pts = camera.getTargetingResults().targets_Detector[0].pts;
        //     System.out.println(pts.toString());
        // }
     
      // ty = getBottomAngle(ty, typ, LimelightHelpers.getCorners(camera.getName()));
        
       

        // System.out.println(targetHeight);

        // double forwardDistancePrecise1 = getForwardDistancePrecise(heightMid, ty);
        // double forwardDistancePrecise2 = getForwardDistancePrecise(heightTall, ty);
        // double forwardDistancePrecise = getForwardDistancePrecise(targetHeight, ty);
        // System.out.println("Distance: " + forwardDistancePrecise + ", Target Height: " + targetHeight);

        // dist = forwardDistancePrecise;

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

    public void determineTargetHeight(double targetAngle) {
        // double targetAngleRad = targetAngle * (Math.PI / 180.0);
        // double cameraAngleRad = cameraAngle * (Math.PI / 180.0);

        // (h2 - h1) = tan(a1 + a2) * d
        // double targetHeightApprox = (Math.tan(targetAngleRad + cameraAngleRad) * forwardDistanceApprox) + cameraHeight;

        if (camera.getName().equals(Constants.TOP_LIMELIGHT_NAME)) {
            if (Math.abs(targetAngle) < 7.0) {
                targetHeight = heightTall;
                // nodeHeight = Constants.NODE_TIP_HEIGHT_TOP;
            } else {
                targetHeight = heightMid;
                // nodeHeight = Constants.NODE_TIP_HEIGHT_MID;
            }
        }
        else if (camera.getName().equals(Constants.BOTTOM_LIMELIGHT_NAME)) {
            targetHeight = 0.0;
        }
    }

    public double getForwardDistancePrecise(double targetHeight, double targetAngle) {
        double targetAngleRad = Math.toRadians(targetAngle); 
        double cameraAngleRad = Math.toDegrees(cameraAngle); 

        //Trig
        double forwardDistance = (targetHeight - cameraHeight) / Math.tan(targetAngleRad + cameraAngleRad);
        return forwardDistance;
    }

    public double getBottomAngle(double ty, double typ, double[] pointsArray) {


        //Top Left
        double xShort1 = pointsArray[0];
        double yShort1 = pointsArray[1];

        //Top Right
        double xLong1 = pointsArray[2];
        double yShort2 = pointsArray[3];

        //Bottom Left
        double xShort2 = pointsArray[4];
        double yLong1 = pointsArray[5];

        //Bottom Right
        double xLong2 = pointsArray[6];
        double yLong2 = pointsArray[7];

        //Averaging values
        double xShort = (xShort1 + xShort2) / 2;
        double xLong = (xLong1 + xLong2) / 2;

        double yShort = (yShort1 + yShort2) / 2;
        double yLong = (yLong1 + yLong2) / 2;

        //Finding the center with relation to the top left
        double centerPieceX = (xShort + xLong) / 2;
        double centerPieceY = (yShort + yLong) / 2;

        //Finding the y position of the crosshair with respect to the top left
        double crosshairY = centerPieceY + typ;

      
        double tyRadians = Math.toRadians(ty);

        //Finding typ with respect to the crosshair
        double typBottom = yLong - crosshairY;

        //Converting from pixels to angles
        double newTY = Math.toDegrees(Math.atan((typBottom / typ) * Math.tan(tyRadians)));

        // TEST
        //double phi = typ + 0.5*Math.abs(yLong - yShort);
        //double netTY = Math.atan((phi / typ) * Math.tan(tyRadians))
        return newTY;

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
