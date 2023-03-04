// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
