/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.*;
import frc.robot.commands.MoveTurret.Direction;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  private final Pneumatics pneumatics = new Pneumatics();

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
    new POVButton(rJoy, Constants.JoySticks.POV_RIGHT).whenHeld(new MoveTurret(Direction.CW, turret));
    // V should do the same thing as ^ but it's inlined
    new POVButton(rJoy, Constants.JoySticks.POV_LEFT).whenHeld(new FunctionalCommand(() -> {
    }, () -> turret.setPercentOutput(-0.3), i -> turret.stop(), () -> false, turret)); // shorter notation?
    // new POVButton(rJoystick,
    // Constants.JoySticks.POV_LEFT).whileActiveContinuous(() ->
    // turret.setPercentOutput(-0.3), turret).whenInactive(turret::stop, turret); //
    // could be shorter notation idk

    new POVButton(rJoy, Constants.JoySticks.POV_UP).whenPressed(limelight::setTracking, limelight);
    new POVButton(rJoy, Constants.JoySticks.POV_DOWN).whenPressed(limelight::setDriving, limelight);

    new JoystickButton(lJoy, 1).whenHeld(new DeployIntake(intake, pneumatics));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
