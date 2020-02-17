/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightTurnToAngleCommand extends PIDCommand {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  /**
   * Creates a new LimelightTurnToAngleCommand.
   */
  public LimelightTurnToAngleCommand(Drivetrain drivetrain, Limelight limelight) {
    super(
        // The controller that the command will use
        new PIDController(kTurn.kP, kTurn.kI, kTurn.kD),
        // This should return the measurement
        limelight::getX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(-output, output);
        });
    addRequirements(drivetrain, limelight);
    this.limelight = limelight;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    limelight.setTracking();
    super.initialize();
    limelight.setTracking();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.stop();
    limelight.setDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
