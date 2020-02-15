/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kLimelight;;

public class Limelight extends SubsystemBase {
  private static NetworkTable table;
  private boolean tracking = false;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setDriving();
  }

  /** @return should return distance in feet from the base of the power port */
  public double getDistance() {
    double d = (kLimelight.h2 - kLimelight.h1) / Math.tan(Math.toRadians(kLimelight.a1 + getY()));
    return d;
  }

  public void toggleTracking() {
    if (tracking) {
      setDriving();
    } else {
      setTracking();
    }
  }

  public void setTracking() {
    table.getEntry("ledMode").setNumber(3.0);
    table.getEntry("camMode").setNumber(0.0);
    tracking = true;
  }

  public void setDriving() {
    table.getEntry("ledMode").setNumber(1.0);
    table.getEntry("camMode").setNumber(1.0);
    tracking = false;
  }

  /**
   * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
   */
  public double getX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
   */
  public double getY() {
    return table.getEntry("ty").getDouble(0.0);
  }

  /** @return Skew or rotation (-90 degrees to 0 degrees) */
  public double getSkew() {
    return table.getEntry("ts").getDouble(0.0);
  }

  /**
   * @return Target Area (0% of image to 100% of image)
   */
  public double getArea() {
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * @return Results of a 3D position solution, 6 numbers: Translation (x,y,y)
   *         Rotation(pitch,yaw,roll)
   */
  public double[] getPos() {
    double[] pos = new double[6];
    pos = table.getEntry("camtran").getDoubleArray(pos);
    return pos;
  }

  /**
   * @return Whether the limelight has any valid targets
   */
  public boolean getTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  /**
   * @return Whether the limelight is detected by the robot
   */
  public boolean detected() {
    return table.getEntry("tl").getDouble(-1) > 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limelight detected", detected());
  }
}
