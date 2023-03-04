// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  CANSparkMax clawGrab;
  DigitalInput clawStop;
  RelativeEncoder clawGrabEncoder;

  public Claw() {
    if (!Constants.ENABLE_CLAW) {
      System.out.println("Claw Disabled");
      return;
    }

    clawGrab = new CANSparkMax(Constants.CAN_CLAW_GRAB, CANSparkMax.MotorType.kBrushless);

    clawGrab.restoreFactoryDefaults();

    clawGrab.setIdleMode(CANSparkMax.IdleMode.kBrake);

    clawGrab.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.CLAW_GRAB_LIMIT);
    clawGrab.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

    clawGrabEncoder = clawGrab.getEncoder();

    clawStop = new DigitalInput(Constants.DIO_CLAW_GRAB_MICRO_SWITCH);
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
