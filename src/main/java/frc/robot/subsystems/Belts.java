/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kBelts;

public class Belts extends SubsystemBase {
  private final CANSparkMax motor;

  /**
   * Creates a new Belts.
   */
  public Belts() {
    motor = new CANSparkMax(kBelts.MOTOR, MotorType.kBrushed);

    motor.restoreFactoryDefaults();
    motor.setInverted(kBelts.INVERTED);
    motor.getEncoder(EncoderType.kNoSensor, 0);
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();
  }

  public void run() {
    motor.set(0.2);
  }

  public void stop() {
    motor.set(0);
  }

  public void reverse() {
    motor.set(-0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
