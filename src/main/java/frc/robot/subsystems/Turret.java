/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final TalonSRX motor = new TalonSRX(Constants.Turret.MOTOR);

  /**
   * Creates a new Turret.
   */
  public Turret() {
    motor.configFactoryDefault();

    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // will need to do PID assignments here 
  }

  public void set(ControlMode mode, double setpoint) {
    motor.set(mode, setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
