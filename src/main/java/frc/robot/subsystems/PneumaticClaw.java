// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.commands.*;
import frc.robot.Constants;

public class PneumaticClaw extends SubsystemBase {

  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);;
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);;

  public PneumaticClaw() {
    if (!Constants.ENABLE_PNEUMATIC_CLAW) {
      System.out.println("Pneumatic Claw Disabled");
      compressor.disable();
      return;
    }

    compressor.enableDigital();
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public CommandBase close() {
    return this.runOnce(() -> solenoid.set(DoubleSolenoid.Value.kForward));
  }

  public CommandBase open() {
    return this.runOnce(() -> solenoid.set(DoubleSolenoid.Value.kReverse));
  }

  public CommandBase toggle() {
    return this.runOnce(() -> solenoid.toggle());
  }
}
