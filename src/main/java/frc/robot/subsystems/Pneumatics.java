/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private final DoubleSolenoid intake;

  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
    intake = new DoubleSolenoid(Constants.Pneumatics.INTAKE_FORWARD, Constants.Pneumatics.INTAKE_REVERSE);

    intake.set(Value.kReverse);
  }

  public void deployIntake() {
    intake.set(Value.kForward);
  }

  public void retractIntake() {
    intake.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
