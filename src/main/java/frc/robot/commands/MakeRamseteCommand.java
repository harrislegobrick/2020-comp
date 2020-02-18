/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.kDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class MakeRamseteCommand extends RamseteCommand {
  /**
   * Wrapper for following a trajectory.
   * 
   * @param trajectory the trajectory to follow
   * @param drivetrain the drivetrain to drive the path
   */
  public MakeRamseteCommand(Trajectory trajectory, Drivetrain drivetrain) {
    super(trajectory, drivetrain::getPose, new RamseteController(),
        new SimpleMotorFeedforward(kDrivetrain.S_VOLTS, kDrivetrain.V_VOLTS_SECOND_PER_METER,
            kDrivetrain.A_VOLT_SEONDS_SQUARED_PER_METER),
        kDrivetrain.DRIVE_KINEMATICS, drivetrain::getWheelSpeeds, new PIDController(kDrivetrain.P_DRIVE_VEL, 0, 0),
        new PIDController(kDrivetrain.P_DRIVE_VEL, 0, 0), drivetrain::driveVolts, drivetrain);
  }
}
