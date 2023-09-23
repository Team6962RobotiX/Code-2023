// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicComboPopup.InvocationKeyHandler;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.commands.*;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private PWMSparkMax intakeMotor = new PWMSparkMax(0);

  /** Creates a new ExampleSubsystem. */
  public Intake() {
  }

  @Override
  public void periodic() {
    setSpeed(0.0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public CommandBase intake() {
    return this.run(() -> {
      setSpeed(-0.4);
    });
  }

  public CommandBase output() {
    return this.run(() -> {
      setSpeed(0.6);
    });
  }
}
