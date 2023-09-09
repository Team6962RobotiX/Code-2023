// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicComboPopup.InvocationKeyHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN_INTAKE, CANSparkMax.MotorType.kBrushless);
  private double speed = 0.15;
  boolean intakeOn;
  boolean outputOn;
  long toggleTime;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakeOn) {
      intake();
    } else {
      output(toggleTime, System.currentTimeMillis());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void intake() {
    intakeMotor.set(speed);
  }

  private void output(long callTime, long currentTime) {
    if (currentTime - callTime < 10000) {
      intakeMotor.set(-speed);
    } else {
      intakeMotor.set(0);
    }
  }

  public CommandBase toggle() {
    return this.runOnce(() -> intakeOn = !intakeOn);
  }
}
