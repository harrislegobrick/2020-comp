/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.Constants.*;

public class AutonSelectorCommand {
  private final Drivetrain drivetrain;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Limelight limelight;
  private final Pneumatics pneumatics;

  public AutonSelectorCommand(Drivetrain drivetrain, Flywheel flywheel, Intake intake, Limelight limelight,
      Pneumatics pneumatics) {
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.intake = intake;
    this.limelight = limelight;
    this.pneumatics = pneumatics;
  }

  public Command getAutonOne() throws IOException {
    // add trajectory to follow
    RamseteCommand goToTrench = getRamseteCommand(TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GoToTrench.wpilib.json")));
    RamseteCommand trenchRun = getRamseteCommand(TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/RunTrench.wpilib.json")));

    return new InstantCommand(limelight::setTracking, limelight)
        .andThen(new ShootCommand(flywheel).withTimeout(3).andThen(new InstantCommand(limelight::setDriving, limelight))
            .andThen(goToTrench.andThen(new DeployIntakeCommand(intake, pneumatics).withTimeout(4).alongWith(trenchRun))));
  }

  public Command getTestCommand() throws IOException {
    RamseteCommand pathOne = getRamseteCommand(
        TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/GoToTrench.wpilib.json")));
    return pathOne;
  }

  private RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(),
        new SimpleMotorFeedforward(Drive.S_VOLTS, Drive.V_VOLTS_SECOND_PER_METER,
            Drive.A_VOLT_SEONDS_SQUARED_PER_METER),
        Drive.DRIVE_KINEMATICS, drivetrain::getWheelSpeeds, new PIDController(Drive.P_DRIVE_VEL, 0, 0),
        new PIDController(Drive.P_DRIVE_VEL, 0, 0), drivetrain::driveVolts, drivetrain);
  }
}