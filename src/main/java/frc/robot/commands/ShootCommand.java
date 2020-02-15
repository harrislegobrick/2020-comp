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
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;

public class ShootCommand extends CommandBase {
  private final Flywheel flywheel;
  private final Limelight limelight;
  private final Belts belts;
  private double delay = 0.5;
  private double initTime;

  /**
   * Creates a new ShootCommand.
   */
  public ShootCommand(Flywheel flywheel, Limelight limelight, Belts belts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheel = flywheel;
    this.limelight = limelight;
    this.belts = belts;
    addRequirements(flywheel, belts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 4000 / 15 = 267 so multiply distance by 267 to get RPM?
    if (limelight.getDistance() > 5) {
      flywheel.setVelocity(267 * limelight.getDistance());
    } else {
      flywheel.setVelocity(4000);
    }

    if (getFPGATimestamp() > (initTime + delay)) {
      if (flywheel.getVelocity() > 3900) {
        belts.run();
      } else {
        belts.stop();
      }
    } else {
      belts.reverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    belts.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
