/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final Joystick lJoy = new Joystick(kJoySticks.LEFT);
  private final Joystick rJoy = new Joystick(kJoySticks.RIGHT);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Intake intake = new Intake();
  private final Flywheel flywheel = new Flywheel();
  private final Belts belts = new Belts();
  private final Climb climb = new Climb();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new TankDrive(lJoy::getY, rJoy::getY, rJoy::getThrottle, drivetrain));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // manual limelight toggle on left stick's nub
    new POVButton(lJoy, kJoySticks.POV_UP).whenPressed(limelight::setTracking, limelight);
    new POVButton(lJoy, kJoySticks.POV_DOWN).whenPressed(limelight::setDriving, limelight);

    // intake and shooting on joystick triggers
    new JoystickButton(lJoy, 1).whenHeld(new DeployIntakeCommand(intake, belts));
    new JoystickButton(rJoy, 1).whenHeld(new ShootCommand(flywheel, limelight, belts));

    // limelight auto adjust on left joystick button closest to driverstation on top
    new JoystickButton(lJoy, 4).whenHeld(new LimelightTurnToAngleCommand(drivetrain, limelight));

    // climbing on far top buttons for left and right and right middle bottom button
    new JoystickButton(lJoy, 3).and(new JoystickButton(rJoy, 4)).whenActive(
        new InstantCommand(climb::deploy, climb).andThen(new WaitCommand(2)).andThen(climb::release, climb));
    new JoystickButton(rJoy, 6).whileHeld(climb::run, climb).whenInactive(climb::stop, climb);

    // return to shooting position (I have no idea if this will work, uncomment
    // later once everything else is tested)
    /*
     * new JoystickButton(lJoy, 6) .whenHeld( new InstantCommand(() -> new
     * MakeRamseteCommand(TrajectoryGenerator.generateTrajectory(drivetrain.getPose(
     * ), List.of(), kFieldPositions.SHOOTING_POS, kDrivetrain.CONFIG),
     * drivetrain).schedule()) .andThen(() -> drivetrain.driveVolts(0, 0),
     * drivetrain)) .whenInactive(() -> drivetrain.driveVolts(0, 0), drivetrain);
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonOne() {
    String goToTrenchJSON = "paths/GoToTrench.wpilib.json";
    String runTrenchJSON = "paths/RunTrench.wpilib.json";
    try {
      Path goToTrenchPath = Filesystem.getDeployDirectory().toPath().resolve(goToTrenchJSON);
      Path runTrenchPath = Filesystem.getDeployDirectory().toPath().resolve(runTrenchJSON);

      Trajectory goToTrenchTrajectory = TrajectoryUtil.fromPathweaverJson(goToTrenchPath);
      Trajectory runTrenchTrajectory = TrajectoryUtil.fromPathweaverJson(runTrenchPath);
      RamseteCommand goToTrench = new MakeRamseteCommand(goToTrenchTrajectory, drivetrain);
      RamseteCommand trenchRun = new MakeRamseteCommand(runTrenchTrajectory, drivetrain);

      return new InstantCommand(limelight::setTracking, limelight)
          .andThen(new ShootCommand(flywheel, limelight, belts).withTimeout(3))
          .andThen(new InstantCommand(limelight::setDriving, limelight)).andThen(goToTrench).andThen(
              trenchRun.raceWith(new DeployIntakeCommand(intake, belts)).andThen(() -> drivetrain.driveVolts(0, 0)));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectorys: " + goToTrenchJSON + " " + runTrenchJSON,
          ex.getStackTrace());
    }
    return null;
  }

  /**
   * Just <b>drives</b> straight
   * 
   * @return : the command to drive straight
   */
  public Command getTestCommand() {
    String trajectoryJSON = "paths/GoToTrench.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      RamseteCommand pathOne = new MakeRamseteCommand(
          trajectory.transformBy(drivetrain.getPose().minus(trajectory.getInitialPose())), drivetrain);
      return pathOne.andThen(() -> drivetrain.driveVolts(0, 0));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return null;
  }

  public Command getSixBallAutoCommand() {
    TrajectoryConfig reversedConfig = kDrivetrain.CONFIG;
    reversedConfig.setReversed(true);

    drivetrain.setPosition(kFieldPositions.SHOOTING_POS);

    Trajectory goToTrenchTrajectory = TrajectoryGenerator.generateTrajectory(kFieldPositions.SHOOTING_POS, List.of(),
        kFieldPositions.TRENCH_RUNUP, reversedConfig);
    RamseteCommand goToTrench = new MakeRamseteCommand(goToTrenchTrajectory, drivetrain);

    Trajectory runTrenchTrajectory = TrajectoryGenerator.generateTrajectory(kFieldPositions.TRENCH_RUNUP, List.of(),
        kFieldPositions.TRENCH_END, reversedConfig);
    RamseteCommand runTrench = new MakeRamseteCommand(runTrenchTrajectory, drivetrain);

    Trajectory returnTrajectory = TrajectoryGenerator.generateTrajectory(kFieldPositions.TRENCH_END, List.of(),
        kFieldPositions.SHOOTING_POS, kDrivetrain.CONFIG);
    RamseteCommand returnToShoot = new MakeRamseteCommand(returnTrajectory, drivetrain);

    return new InstantCommand(limelight::setTracking, limelight)
        .andThen(new ShootCommand(3, flywheel, limelight, belts).withTimeout(5))
        .andThen(limelight::setDriving, limelight).andThen(goToTrench)
        .andThen(runTrench.raceWith(new DeployIntakeCommand(intake, belts))).andThen(returnToShoot)
        .andThen(new ShootCommand(3, flywheel, limelight, belts).withTimeout(5));
  }
}
