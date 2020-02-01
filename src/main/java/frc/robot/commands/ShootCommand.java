/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;

public class ShootCommand extends CommandBase {
  private final Flywheel flywheel;
  private final Limelight limelight;

  /**
   * Creates a new ShootCommand.
   */
  public ShootCommand(Flywheel flywheel, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheel = flywheel;
    this.limelight = limelight;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 4000 / 15 = 267 so multiply distance by 267 to get RPM?
    if (limelight.getDistance() > 5)
      flywheel.setVelocity(267 * limelight.getDistance());
    else
      flywheel.setVelocity(4000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
