/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  private static NetworkTable table;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setDriving();
  }

  /** should return distance in feet */
  public double getDistance() {
    double d = (Constants.Limelight.h2 - Constants.Limelight.h1) / Math.tan(Constants.Limelight.a1 + getY());
    return d;
  }

  public void setTracking() {
    table.getEntry("ledMode").setNumber(3.0);
    table.getEntry("camMode").setNumber(0.0);
  }

  public void setDriving() {
    table.getEntry("ledMode").setNumber(1.0);
    table.getEntry("camMode").setNumber(1.0);
  }

  public double getX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getY() {
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getSkew() {
    return table.getEntry("ts").getDouble(0.0);
  }

  public double getArea() {
    return table.getEntry("ta").getDouble(0.0);
  }

  public double[] getPos() {
    double[] pos = new double[6];
    pos = table.getEntry("camtran").getDoubleArray(pos);
    return pos;
  }

  public boolean targetLock() {
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  public boolean detected() {
    return table.getEntry("tl").getDouble(-1) > 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}