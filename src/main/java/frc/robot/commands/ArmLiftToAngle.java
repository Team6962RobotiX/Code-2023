// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmLiftToAngle extends CommandBase {
  PIDController liftPID;
  ArmFeedforward liftFF;

  private final Arm arm;
  double targetLiftPosition;


  public ArmLiftToAngle(Arm arm, double targetLiftPosition) {
    this.arm = arm;

    liftFF = new ArmFeedforward(Constants.ARM_LIFT_KS, Constants.ARM_LIFT_KG, Constants.ARM_LIFT_KV);
    liftPID = new PIDController(Constants.ARM_LIFT_KP, Constants.ARM_LIFT_KI, Constants.ARM_LIFT_KD);

    setLiftPosition(targetLiftPosition);

    liftPID.setSetpoint(targetLiftPosition);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    liftPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setLiftPosition(targetLiftPosition);

    double basePower = liftFF.calculate(arm.getLiftTicks(), 0, 0);
    double PIDPower = liftPID.calculate(arm.getLiftTicks());
    arm.setLiftPower(basePower + PIDPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setLiftPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setLiftPosition(double position) {
    position = Math.min(position, Constants.ARM_LIFT_MAX_ANGLE);

    position = Math.max(position, arm.getMinLiftAngle());

    targetLiftPosition = position;
  }
}
