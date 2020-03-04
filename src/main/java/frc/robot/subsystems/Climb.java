/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimb;

public class Climb extends SubsystemBase {
  private final CANSparkMax motor;
  private final DoubleSolenoid extendor;
  private final DoubleSolenoid release;

  /**
   * Creates a new Climb.
   */
  public Climb() {
    motor = new CANSparkMax(kClimb.MOTOR, MotorType.kBrushless);
    extendor = new DoubleSolenoid(kClimb.FWD_REV_EXTENDOR[0], kClimb.FWD_REV_EXTENDOR[1]);
    release = new DoubleSolenoid(kClimb.FWD_REV_RELEASE[0], kClimb.FWD_REV_RELEASE[1]);

    motor.restoreFactoryDefaults();
    motor.setMotorType(MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(kClimb.INVERTED);

    retract();
    hold();
  }

  public void deploy() {
    extendor.set(Value.kForward);
  }

  public void release() {
    if (extendor.get() == Value.kForward) {
      release.set(Value.kReverse);
    }
  }

  public void hold() {
    release.set(Value.kForward);
  }

  public void retract() {
    extendor.set(Value.kReverse);
  }

  public void run() {
    if (extendor.get() == Value.kForward && release.get() == Value.kReverse) {
      motor.set(0.80);
    }
  }

  public void unwind() {
    motor.set(-1);
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
