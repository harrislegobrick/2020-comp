/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX leftMaster, rightMaster;
  private final VictorSPX leftSlave, rightSlave;
  private final ADXRS450_Gyro gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftMaster = new TalonSRX(Constants.Drive.LEFT_MASTER);
    leftSlave = new VictorSPX(Constants.Drive.LEFT_SLAVE);
    rightMaster = new TalonSRX(Constants.Drive.RIGHT_MASTER);
    rightSlave = new VictorSPX(Constants.Drive.RIGHT_SLAVE);

    gyro = new ADXRS450_Gyro();

    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();


    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    gyro.calibrate();
  }

  public void drive(double left, double right) {
    leftMaster.set(ControlMode.Current, left);
    rightMaster.set(ControlMode.Current, -right);
  }

  public void setLeftSide(double value) {
    leftMaster.set(ControlMode.Current, value);
  }

  public void setRightSide(double value) {
    rightMaster.set(ControlMode.Current, value);
  }

  public void stop() {
    rightMaster.set(ControlMode.Current, 0);
    leftMaster.set(ControlMode.Current, 0);
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
