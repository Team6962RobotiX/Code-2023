// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmExtendToPosition extends CommandBase {
  PIDController extendPID;

  private final Arm arm;
  double targetExtendPosition;

  public ArmExtendToPosition(Arm arm, double targetExtendPosition) {
    this.arm = arm;

    extendPID = new PIDController(Constants.ARM_EXTEND_KP, Constants.ARM_EXTEND_KI, Constants.ARM_EXTEND_KD);
    extendPID.setSetpoint(targetExtendPosition);

    setExtendPosition(targetExtendPosition);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extendPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setExtendPosition(targetExtendPosition);
    double PIDPower = extendPID.calculate(arm.getExtendPosition());
    arm.setExtendPower(PIDPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setExtendPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setExtendPosition(double position) {
    position = Math.min(position, arm.getMaxExtendPosition());

    position = Math.max(position, 0);

    targetExtendPosition = position;
  }
}
