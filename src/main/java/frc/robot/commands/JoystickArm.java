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

public class JoystickArm extends CommandBase {
  private final Arm arm;
  private final Supplier<Joystick> joystickSupplier;

  public JoystickArm(Arm arm, Supplier<Joystick> joystickSupplier) {
    this.arm = arm;
    this.joystickSupplier = joystickSupplier;

    addRequirements(arm);
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

    double POV = joystick.getPOV();

    double extendSpeed = Constants.mapPower(straightAxis, 0, Constants.ARM_EXTEND_SPEED, Constants.STRAIGHT_DEADZONE);
    double liftSpeed = 0;

    if (POV == 315 || POV == 0 || POV == 45) {
      extendSpeed += Constants.ARM_EXTEND_SPEED_FINE;
    }

    if (POV == 135 || POV == 180 || POV == 225) {
      extendSpeed -= Constants.ARM_EXTEND_SPEED_FINE;
    }

    if (joystick.getRawButton(6)) {
      liftSpeed += Constants.mapNumber(joystick.getThrottle(), -1, 1, Constants.ARM_LIFT_SPEED_FINE * 4, Constants.ARM_LIFT_SPEED_FINE);
    }

    if (joystick.getRawButton(4)) {
      liftSpeed -= Constants.mapNumber(joystick.getThrottle(), -1, 1, Constants.ARM_LIFT_SPEED_FINE * 4, Constants.ARM_LIFT_SPEED_FINE * 3);
    }

    arm.incrementLiftAngle(liftSpeed * (20.0 / 1000.0));
    arm.incrementExtendMeters(extendSpeed * (20.0 / 1000.0));
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
