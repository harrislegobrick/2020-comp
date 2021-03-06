/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {
  private final VictorSPX motor;
  private final DoubleSolenoid intake;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    motor = new VictorSPX(kIntake.MOTOR);
    intake = new DoubleSolenoid(kIntake.FWD_REV_SOLINOID[0], kIntake.FWD_REV_SOLINOID[1]);

    intake.set(Value.kReverse);

    motor.configFactoryDefault();
    motor.setInverted(kIntake.INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void deployIntake() {
    intake.set(Value.kForward);
  }

  public void retractIntake() {
    intake.set(Value.kReverse);
  }

  public void run() {
    motor.set(ControlMode.PercentOutput, 0.35);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
