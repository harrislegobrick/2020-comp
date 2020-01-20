/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;

public class DeployIntakeCommand extends CommandBase {
  private Intake intake;
  private Pneumatics pneumatics;

  /**
   * Creates a new DeployIntake.
   * <p>
   * can use {@link Command#withTimeout(double)} decorator to retract after
   * seconds desired
   * </p>
   */
  public DeployIntakeCommand(Intake intake, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.pneumatics = pneumatics;
    addRequirements(intake, pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    pneumatics.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
