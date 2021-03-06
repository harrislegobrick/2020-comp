/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Belts;
import frc.robot.subsystems.Intake;

public class DeployIntakeCommand extends CommandBase {
  private Intake intake;
  private Belts belts;
  private double initTime;

  /**
   * Creates a new DeployIntake.
   * <p>
   * can use {@link Command#withTimeout(double)} decorator to retract after
   * seconds desired
   * </p>
   */
  public DeployIntakeCommand(Intake intake, Belts belts) {
    this.intake = intake;
    this.belts = belts;
    addRequirements(intake, belts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deployIntake();
    initTime = getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getFPGATimestamp() > (initTime + 0.25)) {
      intake.run();
    }
    belts.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    belts.stop();
    intake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
