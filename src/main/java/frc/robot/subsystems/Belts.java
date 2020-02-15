/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kBelts;

public class Belts extends SubsystemBase {
  private final WPI_VictorSPX motor;

  /**
   * Creates a new Belts.
   */
  public Belts() {
    motor = new WPI_VictorSPX(kBelts.MOTOR);

    motor.configFactoryDefault();
    motor.setInverted(kBelts.INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void run() {
    motor.set(ControlMode.PercentOutput, 0.2);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void reverse() {
    motor.set(ControlMode.PercentOutput, -0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
