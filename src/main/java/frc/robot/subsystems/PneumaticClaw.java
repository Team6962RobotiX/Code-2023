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

  Compressor clawCompressor;
  DoubleSolenoid clawSolenoid;

  public PneumaticClaw() {
    if (!Constants.ENABLE_PNEUMATIC_CLAW) {
      System.out.println("Pneumatic Claw Disabled");
      return;
    }

    clawCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    clawCompressor.enableDigital();

    clawCompressor.disable();

    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    clawSolenoid.set(DoubleSolenoid.Value.kOff);
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
