/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kTurret;

public class Turret extends SubsystemBase {
  private final TalonSRX motor;

  /**
   * Creates a new Turret.
   */
  public Turret() {
    motor = new TalonSRX(kTurret.MOTOR);

    motor.configFactoryDefault();
    motor.setInverted(kTurret.INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // will need to do PID assignments here
  }

  public void setCurrent(double value) {
    motor.set(ControlMode.Current, value);
  }

  public void turnRight() {
    motor.set(ControlMode.PercentOutput, 0.2);
  }

  public void turnLeft() {
    motor.set(ControlMode.PercentOutput, -0.2);
  }

  public void setPercentOutput(double value) {
    motor.set(ControlMode.PercentOutput, value);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
