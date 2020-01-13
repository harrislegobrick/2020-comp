/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax motor;
  private final CANPIDController pController;

  /**
   * Creates a new Flywheel.
   */
  public Flywheel() {
    motor = new CANSparkMax(Constants.Flywheel.MOTOR, MotorType.kBrushless);
    pController = motor.getPIDController();

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(Constants.Flywheel.INVERTED);

    pController.setFeedbackDevice(motor.getEncoder());
    pController.setP(Constants.Flywheel.kP);
    pController.setI(Constants.Flywheel.kI);
    pController.setD(Constants.Flywheel.kD);
    pController.setFF(Constants.Flywheel.kFF);
    pController.setIZone(Constants.Flywheel.kIZone);
  }

  public void setVelocity(double value) {
    pController.setReference(value, ControlType.kVelocity);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}