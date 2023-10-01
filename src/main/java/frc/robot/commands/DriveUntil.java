package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

public class DriveUntil extends CommandBase {
  private Drive drive;
  private double power;
  private double secondsAfter;
  private double countedSeconds = 0.0;
  private Supplier<Boolean> condition;

  public DriveUntil(Drive drive, Supplier<Boolean> condition, double power, double secondsAfter) {
    this.drive = drive;
    this.power = power;
    this.secondsAfter = secondsAfter;
    this.condition = condition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (condition.get()) {
      countedSeconds += 0.02;
    }
    drive.tankDrive(power, power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return countedSeconds > secondsAfter && condition.get();
  }
}
