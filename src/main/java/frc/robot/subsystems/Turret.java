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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.kTurret;

public class Turret extends PIDSubsystem {
  private final WPI_TalonSRX motor;
  private final Limelight limelight;

  /**
   * Creates a new Turret.
   */
  public Turret(Limelight limelight){
    super(new PIDController(kTurret.kP, kTurret.kI, kTurret.kD));
    this.limelight = limelight;
    motor = new WPI_TalonSRX(kTurret.MOTOR);

    motor.configFactoryDefault();
    motor.setInverted(kTurret.INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // will need to do PID assignments here
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    motor.set(ControlMode.PercentOutput, output);
  }

  @Override
  protected double getMeasurement() {
    return limelight.getX();
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
