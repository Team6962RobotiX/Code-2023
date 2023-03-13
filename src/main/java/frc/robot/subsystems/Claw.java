// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  CANSparkMax clawGrab = new CANSparkMax(Constants.CAN_CLAW_GRAB, CANSparkMax.MotorType.kBrushless);;
  DigitalInput clawStop = new DigitalInput(Constants.DIO_CLAW_GRAB_MICRO_SWITCH);;
  RelativeEncoder clawGrabEncoder = clawGrab.getEncoder();
  
  public Claw() {
    if (!Constants.ENABLE_CLAW) {
      System.out.println("Claw Disabled");
      return;
    }

    clawGrab.restoreFactoryDefaults();

    clawGrab.setIdleMode(CANSparkMax.IdleMode.kBrake);

    clawGrab.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.CLAW_GRAB_LIMIT);
    clawGrab.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    clawGrab.setIdleMode(idleMode);
  }
}
