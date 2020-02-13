/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class kDrivetrain {
        public static final int WHEEL_SIZE_INCH = 6;
        public static final int HUNDRED_MS_TO_SECONDS_CONVERSION = 10;
        public static final double TICKS_TO_METERS = (1.0 / 4096) * (WHEEL_SIZE_INCH * Math.PI)
                / /* convert from inch to meter */ 39.37;
        public static final double TRACK_WIDTH_METERS = 0.5511407932368775;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        public static final int LIL_TIMEOUT = 10;
        public static final int BIG_TIMEOUT = 50;
        public static final double OPEN_LOOP_RAMP = 0.5; // how many seconds it takes to go from neutral to full output

        public static final int LEFT_MASTER = 1;
        public static final int LEFT_SLAVE = 2;
        public static final int RIGHT_MASTER = 3;
        public static final int RIGHT_SLAVE = 4;

        public static final boolean kGyroReversed = true;
        public static final boolean SENSOR_PHASE = true;
        public static final boolean INVERTED = false;
        public static final int PID_SLOT = 0;

        public static final double VOLTAGE_COMP_SATURATION = 12;
        public static final boolean VOLTAGE_COMP_ENABLED = false;

        // will need characterization to find values
        public static final double S_VOLTS = 0.675;
        public static final double V_VOLTS_SECOND_PER_METER = 3.15;
        public static final double A_VOLT_SEONDS_SQUARED_PER_METER = 0.404;

        public static final double MAX_SPEED_METERS_PER_SECOND = 1;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5;
        public static final double P_DRIVE_VEL = 11.0;
    }

    public static final class kJoySticks {
        public static final int LEFT = 0;
        public static final int RIGHT = 1;

        public static final int POV_UP = 0;
        public static final int POV_RIGHT = 90;
        public static final int POV_DOWN = 180;
        public static final int POV_LEFT = 270;
    }

    public static final class kLimelight {
        public static final double a1 = 0; // mounting angle
        public static final double h1 = 0; // height of your camera above the floor (in feet)
        public static final double h2 = 6.77083; // height of the target (in feet)
    }

    public static final class kTurret {
        public static final int MOTOR = 5;
        public static final boolean INVERTED = false;
        // WILL NEED TUNING
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class kFlywheel {
        public static final int MOTOR = 6;
        public static final boolean INVERTED = false;
        // WILL NEED TUNING
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kIZone = 0;
    }

    public static final class kIntake {
        public static final int MOTOR = 5;
        public static final boolean INVERTED = true;

        public static final int INTAKE_FORWARD = 0;
        public static final int INTAKE_REVERSE = 1;
    }

    public static final class kColorWheel {
        public static final int MOTOR = 0;
        public static final boolean INVERTED = false;
    }

    public static final class kBelts {
        public static final int MOTOR = 6;
        public static final boolean INVERTED = false;
    }

    public static final class kFieldPositions {
        public static final Pose2d TRENCH_TWO = new Pose2d(9.311, -0.726, Rotation2d.fromDegrees(180));
        public static final Pose2d SHEILD_TWO = new Pose2d(9.739, -5.095, Rotation2d.fromDegrees(100));
        public static final Pose2d SHEILD_TWO_RUNUP = new Pose2d(10.339, -6.037, Rotation2d.fromDegrees(120));
        public static final Pose2d SHOOTING_POS = new Pose2d(12.909, -5.815, Rotation2d.fromDegrees(180));
        public static final Pose2d TRENCH_RUNUP = new Pose2d(11.024, -7.494, Rotation2d.fromDegrees(180));
        public static final Pose2d TRENCH_END = new Pose2d(7.255, -7.494, Rotation2d.fromDegrees(180));
        public static final Pose2d CITRUS_START = new Pose2d(12.909, -0.761, Rotation2d.fromDegrees(180));
    }
}
