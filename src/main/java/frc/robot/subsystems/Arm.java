// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import java.util.Map;

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

  private ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");

  private GenericEntry P = dashboard.add("P", Constants.ARM_LIFT_KP).getEntry();
  private GenericEntry I = dashboard.add("I", Constants.ARM_LIFT_KI).getEntry();
  private GenericEntry D = dashboard.add("D", Constants.ARM_LIFT_KD).getEntry();
  private GenericEntry target = dashboard.add("Target Angle", 90).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", Constants.ARM_LIFT_MIN_ANGLE, "max", Constants.ARM_LIFT_MAX_ANGLE)).getEntry();
  private GenericEntry angle = dashboard.add("Current Angle", 90).getEntry();

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

    lift2.setInverted(true);

    extend.setInverted(true);
    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) (Constants.ARM_MAX_LENGTH - Constants.ARM_STARTING_LENGTH));
    extend.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

    extendEncoder = extend.getEncoder();
    extendEncoder.setPositionConversionFactor(1 / Constants.ARM_EXTEND_TICKS_PER_METER);
    liftEncoder = new DutyCycleEncoder(Constants.DIO_ARM_LIFT_ENCODER);
    liftEncoder.setPositionOffset(Constants.ARM_LIFT_ENCODER_OFFSET / 360.0);
    liftEncoder.setDistancePerRotation(360.0);

    liftFF = new ArmFeedforward(Constants.ARM_LIFT_KS, Constants.ARM_LIFT_KG, Constants.ARM_LIFT_KV);
    liftPID = new PIDController(Constants.ARM_LIFT_KP, Constants.ARM_LIFT_KI, Constants.ARM_LIFT_KD);
    extendPID = new PIDController(Constants.ARM_EXTEND_KP, Constants.ARM_EXTEND_KI, Constants.ARM_EXTEND_KD);

    targetLiftAngle = getLiftAngle();
    targetExtendMeters = getExtendMeters();

    setLiftAngle(targetLiftAngle);
    setLiftAngle(targetExtendMeters);

    liftPID.setTolerance(Constants.ARM_LIFT_ANGLE_TOLERANCE);
    extendPID.setTolerance(Constants.ARM_EXTEND_METERS_TOLERANCE);
    liftPID.setSetpoint(targetLiftAngle);


  }

  @Override
  public void periodic() {
    if (!Constants.ENABLE_ARM) {
      return;
    }

    liftPID.setP(P.getDouble(Constants.ARM_LIFT_KP));
    liftPID.setI(I.getDouble(Constants.ARM_LIFT_KI));
    liftPID.setD(D.getDouble(Constants.ARM_LIFT_KD));
    targetLiftAngle = target.getDouble(90);

    angle.setDouble(getLiftAngle());

    setLiftAngle(targetLiftAngle);
    setExtendMeters(targetExtendMeters);

    double liftBasePower = liftFF.calculate(getLiftAngle(), 0, 0);
    double liftPIDPower = liftPID.calculate(getLiftAngle());
    // setLiftPower(liftBasePower + liftPIDPower);

    double extendPIDPower = extendPID.calculate(getExtendMeters());
    // setExtendPower(extendPIDPower);

    // System.out.println(getLiftAngle());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getExtendMeters() {
    return extendEncoder.getPosition();
  }

  public double getLiftAngle() {
    return liftEncoder.getDistance();
  }

  public double getMinLiftAngle() {
    double minAngle = Math.acos(Constants.ARM_HEIGHT / getExtendMeters());
    if (minAngle < Constants.ARM_LIFT_MIN_ANGLE) {
      minAngle = Constants.ARM_LIFT_MIN_ANGLE;
    }
    return minAngle;
    // return Constants.ARM_LIFT_MIN_ANGLE;
  }

  public double getMaxExtendMeters() {
    double maxExtension = Math.min(Constants.ARM_MAX_LENGTH, Constants.ARM_HEIGHT / Math.cos(getLiftAngle() / 180 * Math.PI)) - Constants.ARM_STARTING_LENGTH;
    if (getLiftAngle() > 90) {
      maxExtension = Constants.ARM_MAX_LENGTH;
    }
    return maxExtension;
    // return Constants.ARM_MAX_LENGTH;
  }

  public void setExtendPower(double power) {
    double extendPos = getExtendMeters();

    if (extendPos > getMaxExtendMeters()) {
      power = Math.min(0, power);
    }
    if (extendPos < 0) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_EXTEND_MAX_POWER, Math.abs(power)) * Math.signum(power);

    extend.set(power);
  }

  public void setLiftPower(double power) {
    double liftAngle = getExtendMeters();

    if (liftAngle > Constants.ARM_LIFT_MAX_ANGLE) {
      power = Math.min(0, power);
    }
    if (liftAngle < getMinLiftAngle()) {
      power = Math.max(0, power);
    }

    power = Math.min(Constants.ARM_LIFT_MAX_POWER, Math.abs(power)) * Math.signum(power);

    lift.set(power);
  }

  private void setLiftAngle(double angle) {
    angle = Math.min(angle, Constants.ARM_LIFT_MAX_ANGLE);
    angle = Math.max(angle, getMinLiftAngle());

    targetLiftAngle = angle;

    liftPID.setSetpoint(targetLiftAngle);
  }

  private void setExtendMeters(double meters) {
    meters = Math.min(meters, getMaxExtendMeters());
    meters = Math.max(meters, 0);

    targetExtendMeters = meters;

    extendPID.setSetpoint(targetExtendMeters);
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    lift1.setIdleMode(idleMode);
    lift2.setIdleMode(idleMode);
    extend.setIdleMode(idleMode);
  }

  public void setTargetPosition(double targetX, double targetY) {
    targetY -= Constants.ARM_HEIGHT;

    setExtendMeters(Math.sqrt(targetX * targetX + targetY * targetY) - Constants.ARM_STARTING_LENGTH);
    setLiftAngle(Math.atan(targetY / targetX));
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
}
