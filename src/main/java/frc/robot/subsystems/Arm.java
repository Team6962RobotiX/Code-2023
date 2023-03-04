// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  // MOTOR CONTROLLERS
  MotorControllerGroup lift;
  CANSparkMax lift1, lift2, extend;

  // ENCODERS
  RelativeEncoder extendEncoder;
  DutyCycleEncoder liftEncoder;

  double extendTicksPerInch = Constants.ARM_EXTEND_LIMIT / (Constants.ARM_EXTEND_INCHES - Constants.ARM_RETRACT_INCHES);

  public Arm() {

    if (!Constants.ENABLE_ARM) {
      System.out.println("Arm Disabled");
      return;
    }

    lift1 = new CANSparkMax(Constants.CAN_ARM_LIFT_1, CANSparkMax.MotorType.kBrushless);
    lift2 = new CANSparkMax(Constants.CAN_ARM_LIFT_2, CANSparkMax.MotorType.kBrushless);
    extend = new CANSparkMax(Constants.CAN_ARM_EXTEND, CANSparkMax.MotorType.kBrushless);

    lift1.restoreFactoryDefaults();
    lift2.restoreFactoryDefaults();
    extend.restoreFactoryDefaults();

    lift1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lift2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extend.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lift2.setInverted(true);
    lift = new MotorControllerGroup(lift1, lift2);

    extend.setInverted(true);
    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ARM_EXTEND_LIMIT);
    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

    extendEncoder = extend.getEncoder();
    liftEncoder = new DutyCycleEncoder(Constants.DIO_ARM_LIFT_ENCODER);
    liftEncoder.setPositionOffset(Constants.ARM_LIFT_ENCODER_OFFSET / 360.0);
    liftEncoder.setDistancePerRotation(360.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getExtendTicks() {
    return extendEncoder.getPosition();
  }

  public double getLiftAngle() {
    return liftEncoder.getDistance();
  }

  public double getMinLiftAngle() {
    double minAngle = Math.cos((Constants.ARM_HEIGHT_INCHES * extendTicksPerInch) / getMaxExtendTicks());
    if (minAngle < Constants.ARM_LIFT_MIN_ANGLE) {
      minAngle = Constants.ARM_LIFT_MIN_ANGLE;
    }
    return minAngle;
  }

  public double getMaxExtendTicks() {
    double maxExtension = Math.min(Constants.ARM_EXTEND_LIMIT, (Constants.ARM_HEIGHT_INCHES / Math.cos(liftEncoder.getDistance() / 180 * Math.PI) - Constants.ARM_RETRACT_INCHES) * extendTicksPerInch);
    if (liftEncoder.getDistance() > 90) {
      maxExtension = Constants.ARM_EXTEND_LIMIT;
    }
    return maxExtension;
  }

  public double extendInchesToTicks(double inches) {
    return (inches - Constants.ARM_RETRACT_INCHES) * extendTicksPerInch;
  }

  public void setExtendPower(double power) {
    double extendPos = extendEncoder.getPosition();

    if (extendPos > getMaxExtendTicks()) {
      power = Math.min(0, power);
    }

    if (extendPos < 0) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_EXTEND_MAX_POWER, Math.abs(power)) * Math.signum(power);

    extend.set(power);
  }

  public void setLiftPower(double power) {
    double liftAngle = liftEncoder.getDistance();

    if (liftAngle > Constants.ARM_LIFT_MAX_ANGLE) {
      power = Math.min(0, power);
    }

    if (liftAngle < getMinLiftAngle()) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_LIFT_MAX_POWER, Math.abs(power)) * Math.signum(power);

    lift.set(power);
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    lift1.setIdleMode(idleMode);
    lift2.setIdleMode(idleMode);
    extend.setIdleMode(idleMode);
  }
}
