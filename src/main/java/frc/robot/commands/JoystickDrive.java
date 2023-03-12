// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase {
  private final Drive drive;
  private final Supplier<Joystick> joystickSupplier;

  public JoystickDrive(Drive drive, Supplier<Joystick> joystickSupplier) {
    this.drive = drive;
    this.joystickSupplier = joystickSupplier;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick joystick = joystickSupplier.get();

    double straightAxis = -joystick.getRawAxis(1);
    double twistAxis = -joystick.getRawAxis(2);

    double POV = joystick.getPOV();

    double straightPower = Constants.mapPower(straightAxis, Constants.DRIVE_BASE_POWER, Constants.DRIVE_POWER_LIMIT, Constants.STRAIGHT_DEADZONE);
    double turningPower = Constants.mapPower(twistAxis, Constants.DRIVE_BASE_TURN_POWER, Constants.DRIVE_TURN_POWER_LIMIT, Constants.TWIST_DEADZONE);
    
    if (POV == 315 || POV == 0 || POV == 45) {
      straightPower += Constants.DRIVE_FINE_CONTROL_POWER;
    }

    if (POV == 135 || POV == 180 || POV == 225) {
      straightPower -= Constants.DRIVE_FINE_CONTROL_POWER;
    }

    if (POV == 45 || POV == 90 || POV == 135) {
      turningPower -= Constants.DRIVE_FINE_CONTROL_POWER;
    }

    if (POV == 225 || POV == 270 || POV == 315) {
      turningPower += Constants.DRIVE_FINE_CONTROL_POWER;
    }
    
    drive.arcadeDrive(straightPower, turningPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
