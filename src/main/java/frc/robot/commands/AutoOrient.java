package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
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

    private String name;
    private Drive drive;
    private NetworkTable table = null;
    private PIDController orientPID = new PIDController(0.002, 0.0002, 0);

    /** An example command that uses an example subsystem. */
    public AutoOrient(String name, Drive drive) {
        this.name = name;
        this.drive = drive;
        table = NetworkTableInstance.getDefault().getTable(name);
        orientPID.setTolerance(2);
        orientPID.setSetpoint(0);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (LimelightHelpers.getLatestResults(name).targetingResults.targets_Detector.length > 0) {
        //     tx = LimelightHelpers.getLatestResults(name).targetingResults.targets_Detector[0].tx;
        // }
        // if (table.getEntry("tv").getBoolean(false) == false) {
        //     return;
        // }
        double tx = table.getEntry("tx").getDouble(0);
        // System.out.println(tx);
        double PIDPower = orientPID.calculate(tx);
        drive.arcadeDrive(0, Constants.DRIVE_BASE_TURN_POWER * Math.signum(PIDPower) + PIDPower);
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
