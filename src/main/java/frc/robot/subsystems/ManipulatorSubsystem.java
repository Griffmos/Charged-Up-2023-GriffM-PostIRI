// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateHandler;

public class ManipulatorSubsystem extends SubsystemBase {

  private DoubleSolenoid gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new Manipulator. */
  public ManipulatorSubsystem() {
  }

  public void set(boolean output) {
    gripper.set(output ? Value.kForward : Value.kReverse);
  }

  public boolean get() {
    return gripper.get() == Value.kForward ? true : false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
