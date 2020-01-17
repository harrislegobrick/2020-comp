/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public final class Drive {
        public static final int LEFT_MASTER = 0;
        public static final int LEFT_SLAVE = 1;
        public static final int RIGHT_MASTER = 2;
        public static final int RIGHT_SLAVE = 3;
    }

    public final class JoySticks {
        public static final int LEFT = 0;
        public static final int RIGHT = 1;

        public static final int POV_UP = 0;
        public static final int POV_RIGHT = 90;
        public static final int POV_DOWN = 180;
        public static final int POV_LEFT = 270;
    }

    public final class Limelight {
        public static final double a1 = 0; // mounting angle
        public static final double h1 = 2; // height of your camera above the floor (in feet)
        public static final double h2 = 6.77083; // height of the target (in feet)
    }

    public final class Turret {
        public static final int MOTOR = 5;
        public static final boolean INVERTED = false;
        // WILL NEED TUNING
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public final class Flywheel {
        public static final int MOTOR = 6;
        public static final boolean INVERTED = false;
        // WILL NEED TUNING
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kIZone = 0;
    }

    public final class Intake {
        public static final int MOTOR = 6;
        public static final boolean INVERTED = false;
    }

    public final class Pneumatics {
        public static final int INTAKE_FORWARD = 0;
        public static final int INTAKE_REVERSE = 1;
    }
}
