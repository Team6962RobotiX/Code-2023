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
    private LimelightHelpers.LimelightTarget_Retro centerNode;
    private PIDController orientPID = new PIDController(Constants.DRIVE_ORIENT_KP, 0, 0);

    private double pickupHeight = 0.15;
    private double placingHeightMid = 1.087375;
    private double placingHeightTop = 1.33540042;


    private String placeLevel;

    /** An example command that uses an example subsystem. */
    public AutoOrient(Limelight camera, Drive drive, String placeLevel) {
        this.camera = camera;
        this.drive = drive;
        this.placeLevel = placeLevel;
        
        addRequirements(camera, drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        centerNode = camera.getTopCenterTarget();
        Pose3d nodePos = centerNode.getTargetPose_CameraSpace();
        double xPos = nodePos.getX();
        

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
