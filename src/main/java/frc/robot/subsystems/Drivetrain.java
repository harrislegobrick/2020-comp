/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX leftMaster, rightMaster;
  private final VictorSPX leftSlave, rightSlave;
  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftMaster = new WPI_TalonSRX(kDrivetrain.LEFT_MASTER);
    leftSlave = new VictorSPX(kDrivetrain.LEFT_SLAVE);
    rightMaster = new WPI_TalonSRX(kDrivetrain.RIGHT_MASTER);
    rightSlave = new VictorSPX(kDrivetrain.RIGHT_SLAVE);

    gyro = new AHRS();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // reset talons
    leftMaster.configFactoryDefault(kDrivetrain.LIL_TIMEOUT);
    leftSlave.configFactoryDefault(kDrivetrain.LIL_TIMEOUT);
    rightMaster.configFactoryDefault(kDrivetrain.LIL_TIMEOUT);
    rightSlave.configFactoryDefault(kDrivetrain.LIL_TIMEOUT);

    // slaves follow the masters
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // invert the motors so that positive is forward
    leftMaster.setInverted(kDrivetrain.INVERTED);
    rightMaster.setInverted(!kDrivetrain.INVERTED);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    // encoder setup
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kDrivetrain.PID_SLOT,
        kDrivetrain.LIL_TIMEOUT);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kDrivetrain.PID_SLOT,
        kDrivetrain.LIL_TIMEOUT);
    leftMaster.setSensorPhase(kDrivetrain.SENSOR_PHASE);
    rightMaster.setSensorPhase(!kDrivetrain.SENSOR_PHASE);

    // zero encoders
    zeroEncoders();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorPosition() * kDrivetrain.TICKS_TO_METERS,
        rightMaster.getSelectedSensorPosition() * kDrivetrain.TICKS_TO_METERS);
  }

  public void zeroEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void drive(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, right);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
  }

  public void stop() {
    rightMaster.set(ControlMode.PercentOutput, 0);
    leftMaster.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (kDrivetrain.kGyroReversed ? -1.0 : 1.0);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()),
        leftMaster.getSelectedSensorPosition() * kDrivetrain.TICKS_TO_METERS,
        rightMaster.getSelectedSensorPosition() * kDrivetrain.TICKS_TO_METERS);
  }
}
