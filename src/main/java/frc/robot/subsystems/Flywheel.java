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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kFlywheel;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax motor;
  private final CANPIDController pController;

  private int cellsShot = 0;
  private boolean dip = true;

  /**
   * Creates a new Flywheel.
   */
  public Flywheel() {
    motor = new CANSparkMax(kFlywheel.MOTOR, MotorType.kBrushless);
    pController = motor.getPIDController();

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(kFlywheel.INVERTED);

    pController.setP(kFlywheel.kP);
    pController.setI(kFlywheel.kI);
    pController.setD(kFlywheel.kD);
    pController.setFF(kFlywheel.kFF);
    pController.setIZone(kFlywheel.kIZone);
    motor.burnFlash();

    
    SmartDashboard.putNumber("Cells Shot", cellsShot);
    SmartDashboard.putNumber("motor RPM", getVelocity());
  }

  /**
   * 4000 RPM is a good value for launching from 15 ft away.
   * 
   * @param velocity : The velocity to set the motor to.
   */
  public void setVelocity(double velocity) {
    pController.setReference(velocity, ControlType.kVelocity);
    if (getVelocity() > (velocity * 0.98) && dip) {
      dip = false;
    }
    if (getVelocity() < (velocity * 0.95) && !dip) {
      cellsShot++;
      dip = true;
    }
  }

  public double getVelocity() {
    return motor.getEncoder().getVelocity();
  }

  public void stop() {
    pController.setReference(0, ControlType.kVelocity);
    dip = true;
  }

  public int getCellsShotCount() {
    return cellsShot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
