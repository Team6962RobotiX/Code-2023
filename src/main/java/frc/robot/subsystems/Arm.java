// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  // MOTOR CONTROLLERS
  MotorControllerGroup armLift;
  CANSparkMax armLift1, armLift2, armExtend;

  // ENCODERS
  RelativeEncoder armExtendEncoder;
  DutyCycleEncoder armLiftEncoder;

  double armLiftBasePower = 0;
  double armLiftTargetAngle = 0;
  double p_armLiftEncoder = 0;

  public Arm() {

    if (!Constants.ENABLE_ARM) {
      System.out.println("Arm Disabled");
      return;
    }

    armLift1 = new CANSparkMax(Constants.CAN_ARM_LIFT_1, CANSparkMax.MotorType.kBrushless);
    armLift2 = new CANSparkMax(Constants.CAN_ARM_LIFT_2, CANSparkMax.MotorType.kBrushless);
    armExtend = new CANSparkMax(Constants.CAN_ARM_EXTEND, CANSparkMax.MotorType.kBrushless);

    armLift1.restoreFactoryDefaults();
    armLift2.restoreFactoryDefaults();
    armExtend.restoreFactoryDefaults();

    armLift1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armLift2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armExtend.setIdleMode(CANSparkMax.IdleMode.kBrake);

    armLift2.setInverted(true);
    armLift = new MotorControllerGroup(armLift1, armLift2);

    armExtend.setInverted(true);
    armExtend.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.ARM_EXTEND_LIMIT);
    armExtend.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

    armExtendEncoder = armExtend.getEncoder();
    armLiftEncoder = new DutyCycleEncoder(Constants.DIO_ARM_LIFT_ENCODER);
    armLiftEncoder.setPositionOffset(Constants.ARM_LIFT_ENCODER_OFFSET / 360.0);
    armLiftEncoder.setDistancePerRotation(360.0);

    p_armLiftEncoder = armLiftEncoder.getDistance();
    armLiftTargetAngle = armLiftEncoder.getDistance();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
