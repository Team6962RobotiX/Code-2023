// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  CANSparkMax lift1 = new CANSparkMax(Constants.CAN_ARM_LIFT_1, CANSparkMax.MotorType.kBrushless);
  CANSparkMax lift2 = new CANSparkMax(Constants.CAN_ARM_LIFT_2, CANSparkMax.MotorType.kBrushless);
  CANSparkMax extend = new CANSparkMax(Constants.CAN_ARM_EXTEND, CANSparkMax.MotorType.kBrushless);

  MotorControllerGroup lift = new MotorControllerGroup(lift1, lift2);

  RelativeEncoder extendEncoder;
  DutyCycleEncoder liftEncoder;

  PIDController extendPID;
  PIDController liftPID;
  ArmFeedforward liftFF;

  double targetExtendMeters;
  double targetLiftAngle;
  double clampedExtendMeters;

  double nextExtendMeters;
  double nextLiftAngle;

  private ShuffleboardTab dashboard = Shuffleboard.getTab("SmartDashboard");

  public Arm() {
    if (!Constants.ENABLE_ARM) {
      System.out.println("Arm Disabled");
      return;
    }

    lift1.restoreFactoryDefaults();
    lift2.restoreFactoryDefaults();
    extend.restoreFactoryDefaults();

    lift1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lift2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extend.setIdleMode(CANSparkMax.IdleMode.kBrake);

    lift1.setSmartCurrentLimit(40);
    lift2.setSmartCurrentLimit(40);
    extend.setSmartCurrentLimit(40);

    lift2.setInverted(true);
    extend.setInverted(true);

    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) (Constants.ARM_MAX_LENGTH - Constants.ARM_STARTING_LENGTH));
    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

    extendEncoder = extend.getEncoder();
    extendEncoder.setPositionConversionFactor(1 / Constants.ARM_EXTEND_TICKS_PER_METER);

    liftEncoder = new DutyCycleEncoder(Constants.DIO_ARM_LIFT_ENCODER);
    liftEncoder.setDistancePerRotation(360.0);


    liftFF = new ArmFeedforward(Constants.ARM_LIFT_KS, Constants.ARM_LIFT_KG, Constants.ARM_LIFT_KV);
    liftPID = new PIDController(Constants.ARM_LIFT_KP, Constants.ARM_LIFT_KI, Constants.ARM_LIFT_KD);
    extendPID = new PIDController(Constants.ARM_EXTEND_KP, Constants.ARM_EXTEND_KI, Constants.ARM_EXTEND_KD);

    // targetLiftAngle = getLiftAngle();
    targetLiftAngle = getLiftAngle();
    targetExtendMeters = getExtendMeters();
    nextLiftAngle = targetLiftAngle;
    nextExtendMeters = targetExtendMeters;

    updateAngleSetpoint(targetLiftAngle);
    updateExtendSetpoint(targetExtendMeters);

    liftPID.setTolerance(Constants.ARM_LIFT_ANGLE_TOLERANCE);
    extendPID.setTolerance(Constants.ARM_EXTEND_METERS_TOLERANCE);
    liftPID.setSetpoint(targetLiftAngle);
  }

  @Override
  public void periodic() {
    if (!Constants.ENABLE_ARM) {
      return;
    }

    if (lift1.getFaults() != 0 || lift2.getFaults() != 0) {
      System.out.println("ARM LIFT MOTOR CONTROLLERS NOT WORKING");
      lift.set(0.0);
      return;
    }

    SmartDashboard.putNumber("Arm Lift", Constants.mapNumber(getLiftAngle(), getMinLiftAngle(), Constants.ARM_LIFT_MAX_ANGLE, 0, 1));
    SmartDashboard.putNumber("Arm Extend", Constants.mapNumber(getExtendMeters(), 0, getMaxExtendMeters(), 0, 1));

    setIdleMode(CANSparkMax.IdleMode.kBrake);

    // if (joystickSupplier.get().getRawButton(3)) {
    //   targetLiftAngle = Constants.mapNumber(joystickSupplier.get().getThrottle(), -1, 1, Constants.ARM_LIFT_MAX_ANGLE, getMinLiftAngle());
    // }

    updateAngleSetpoint(targetLiftAngle);
    updateExtendSetpoint(targetExtendMeters);

    double liftPIDPower = liftPID.calculate(getLiftAngle());
    setLiftPower(liftPIDPower);

    double extendPIDPower = extendPID.calculate(getExtendMeters());
    setExtendPower(extendPIDPower);

    // System.out.println("getExtendMeters()");
    // System.out.println(getExtendMeters());
    // System.out.println("extendPID.getSetpoint()");
    // System.out.println(extendPID.getSetpoint());
    // System.out.println(getMaxExtendMeters() - getExtendMeters());
    // System.out.println(getLiftAngle());
    // This method will be called once per scheduler run

    if (nextExtendMeters > targetExtendMeters) { // Extending
      targetLiftAngle = nextLiftAngle;
      if (doneLifting()) {
        targetExtendMeters = nextExtendMeters;
      }
    } else { // Retracting
      targetExtendMeters = nextExtendMeters;
      if (doneExtending()) {
        targetLiftAngle = nextLiftAngle;
      }
    }
  }

  private boolean doneLifting() {
    return getLiftAngle() > targetLiftAngle - 8 && getLiftAngle() < targetLiftAngle + 8;
  }

  private boolean doneExtending() {
    return getExtendMeters() > targetExtendMeters - 0.2 && getExtendMeters() < targetExtendMeters + 0.2;
  }

  public void fullyRetract() {
    setExtendMeters(0);
    setLiftAngle(getMinLiftAngle());
  }

  public CommandBase fullyRetractCmd() {
    return this.runOnce(() -> fullyRetract());
  }

  public void resetPID() {
    if (!Constants.ENABLE_ARM) {
      return;
    }

    liftPID.reset();
    extendPID.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getExtendMeters() {
    return extendEncoder.getPosition() - Constants.ARM_EXTEND_PADDING;
  }

  public double getLiftAngle() {
    return ((((liftEncoder.getAbsolutePosition() * 360.0) + Constants.ARM_LIFT_ENCODER_OFFSET) % 360.0) + 360.0) % 360.0;
  }

  public double getMinLiftAngle() {
    double minAngle = Math.acos((Constants.ARM_HEIGHT - Constants.ARM_PADDING_HEIGHT) / (getExtendMeters() + Constants.ARM_STARTING_LENGTH)) / Math.PI * 180.0;
    if (minAngle < Constants.ARM_LIFT_MIN_ANGLE || Double.isNaN(minAngle)) {
      minAngle = Constants.ARM_LIFT_MIN_ANGLE;
    }
    return minAngle;
    // return Constants.ARM_LIFT_MIN_ANGLE;
  }

  public double getMaxExtendMeters() {
    double maxExtension = Math.min(Constants.ARM_MAX_LENGTH, (Constants.ARM_HEIGHT - Constants.ARM_PADDING_HEIGHT) / Math.cos(getLiftAngle() / 180.0 * Math.PI)) - Constants.ARM_STARTING_LENGTH;
    if (getLiftAngle() > 90) {
      maxExtension = Constants.ARM_MAX_LENGTH - Constants.ARM_STARTING_LENGTH;
    }
    return maxExtension;
    // return Constants.ARM_MAX_LENGTH - Constants.ARM_STARTING_LENGTH;
  }

  private void setExtendPower(double power) {
    double extendPos = getExtendMeters();

    if (extendPos > getMaxExtendMeters()) {
      power = Math.min(0, power);
    }
    if (extendPos < -Constants.ARM_EXTEND_PADDING) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_EXTEND_MAX_POWER, Math.abs(power)) * Math.signum(power);
    // System.out.println(extend.getOutputCurrent());
    // System.out.println(getExtendMeters());
    if (doneExtending()) {
      extend.set(0.0);
    } else {
      extend.set(power);
    }
  }

  private void setLiftPower(double power) {
    double liftAngle = getLiftAngle();

    if (liftAngle > Constants.ARM_LIFT_MAX_ANGLE) {
      power = Math.min(0, power);
    }
    if (liftAngle < getMinLiftAngle()) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_LIFT_MAX_POWER, Math.abs(power)) * Math.signum(power);
    // System.out.println(getLiftAngle());
    // System.out.println(getMinLiftAngle());
    if (getLiftAngle() < 0) {
      return;
    }

    lift.set(power);
  }

  private void updateAngleSetpoint(double angle) {
    angle = Math.min(angle, Constants.ARM_LIFT_MAX_ANGLE);
    angle = Math.max(angle, getMinLiftAngle());

    liftPID.setSetpoint(angle);
  }

  private void setLiftAngle(double angle) {
    nextLiftAngle = angle;
  }

  private void setExtendMeters(double meters) {
    nextExtendMeters = meters;
  }

  private void updateExtendSetpoint(double meters) {
    meters = Math.min(meters, getMaxExtendMeters());
    meters = Math.max(meters, 0);

    extendPID.setSetpoint(meters);
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    lift1.setIdleMode(idleMode);
    lift2.setIdleMode(idleMode);
    extend.setIdleMode(idleMode);
  }

  public void setTargetPosition(double targetX, double targetY) {
    resetPID();
    setExtendMeters(Math.sqrt(Math.pow(targetX, 2) + Math.pow(Constants.ARM_HEIGHT - targetY, 2)) - Constants.ARM_STARTING_LENGTH);
    setLiftAngle((Math.atan((targetY - Constants.ARM_HEIGHT) / targetX) / Math.PI * 180) + 90);
  }

  public CommandBase coast() {
    return this.runOnce(() -> setIdleMode(CANSparkMax.IdleMode.kCoast));
  }

  public CommandBase setLiftPowerCmd(double power) {
    return this.runOnce(() -> setLiftPower(power));
  }

  public CommandBase brake() {
    return this.runOnce(() -> setIdleMode(CANSparkMax.IdleMode.kBrake));
  }

  public CommandBase liftToAngle(double angle) {
    return this.runOnce(() -> setLiftAngle(angle));
  }

  public CommandBase extendToLength(double length) {
    return this.runOnce(() -> setExtendMeters(length));
  }

  public CommandBase toPosition(double targetX, double targetY) {
    return this.runOnce(() -> setTargetPosition(targetX, targetY));
  }

  public void incrementLiftAngle(double increment) {
    if (nextLiftAngle > Constants.ARM_LIFT_MAX_ANGLE) {
      increment = Math.min(0, increment);
    }
    if (nextLiftAngle < getMinLiftAngle()) {
      increment = Math.max(0, increment);
    }
    nextLiftAngle += increment;
  }

  public void incrementExtendMeters(double increment) {
    if (nextExtendMeters > getMaxExtendMeters()) {
      increment = Math.min(0, increment);
    }
    if (nextExtendMeters < 0) {
      increment = Math.max(0, increment);
    }
    nextExtendMeters += increment;
  }
}
