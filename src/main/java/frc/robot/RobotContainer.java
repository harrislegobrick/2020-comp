/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.Drive;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick lJoy = new Joystick(Constants.JoySticks.LEFT);
  private final Joystick rJoy = new Joystick(Constants.JoySticks.RIGHT);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final Flywheel flywheel = new Flywheel();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new TankDrive(lJoy::getY, rJoy::getY, rJoy::getThrottle, drivetrain));
    turret.setDefaultCommand(
        new PIDCommand(new PIDController(Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD),
            limelight::getX, 0, output -> turret.setCurrent(output), turret));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(rJoy, Constants.JoySticks.POV_RIGHT).whenPressed(new RunCommand(turret::turnRight, turret))
        .whenReleased(turret::stop, turret);
    new POVButton(rJoy, Constants.JoySticks.POV_LEFT).whenPressed(new RunCommand(turret::turnLeft, turret))
        .whenReleased(turret::stop, turret);

    new POVButton(rJoy, Constants.JoySticks.POV_UP).whenPressed(limelight::setTracking, limelight);
    new POVButton(rJoy, Constants.JoySticks.POV_DOWN).whenPressed(limelight::setDriving, limelight);

    new JoystickButton(lJoy, 1).whenHeld(new DeployIntakeCommand(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonOne() throws IOException {
    // add trajectory to follow
    RamseteCommand goToTrench = getRamseteCommand(
        TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GoToTrench.wpilib.json")));
    RamseteCommand trenchRun = getRamseteCommand(
        TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/RunTrench.wpilib.json")));

    return new InstantCommand(limelight::setTracking, limelight).andThen(new ShootCommand(flywheel).withTimeout(3))
        .andThen(new InstantCommand(limelight::setDriving, limelight)).andThen(goToTrench)
        .andThen(trenchRun.raceWith(new DeployIntakeCommand(intake)));
  }

  /**
   * Just drives straight
   * 
   * @return : the command to drive straight
   * @throws IOException
   */
  public Command getTestCommand() throws IOException {
    RamseteCommand pathOne = getRamseteCommand(
        TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GoToTrench.wpilib.json")));
    return pathOne;
  }

  /**
   * Wrapper for following a trajectory
   * 
   * @param trajectory
   * @return : returns a command that will follow a given path
   */
  private RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(),
        new SimpleMotorFeedforward(Drive.S_VOLTS, Drive.V_VOLTS_SECOND_PER_METER,
            Drive.A_VOLT_SEONDS_SQUARED_PER_METER),
        Drive.DRIVE_KINEMATICS, drivetrain::getWheelSpeeds, new PIDController(Drive.P_DRIVE_VEL, 0, 0),
        new PIDController(Drive.P_DRIVE_VEL, 0, 0), drivetrain::driveVolts, drivetrain);
  }
}
