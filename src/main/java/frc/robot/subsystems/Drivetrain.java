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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static TalonSRX leftMaster, rightMaster;
  private static VictorSPX leftSlave, rightSlave;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftMaster = new TalonSRX(Constants.Drive.leftMaster);
    leftSlave = new VictorSPX(Constants.Drive.leftSlave);
    rightMaster = new TalonSRX(Constants.Drive.rightMaster);
    rightSlave = new VictorSPX(Constants.Drive.rightSlave);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  public void drive(double left, double right) {
    leftMaster.set(ControlMode.Current, left);
    rightMaster.set(ControlMode.Current, right);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
