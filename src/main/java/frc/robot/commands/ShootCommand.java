/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kFlywheel;
import frc.robot.subsystems.Belts;
import frc.robot.subsystems.Flywheel;

public class ShootCommand extends CommandBase {
  private final Flywheel flywheel;
  private final Belts belts;
  private double delay = 0.5;
  private double initTime;
  private int powerCellsToShoot;
  private int initCellCount;

  /**
   * Shoots untill the command is canceled
   * 
   * @param flywheel  flywheel
   * @param limelight limelight
   * @param belts     belts
   */
  public ShootCommand(Flywheel flywheel, Belts belts) {
    this(-10, flywheel, belts);
  }

  /**
   * Shoots untill the entered number of shots have been shot
   * 
   * @param powerCellsToShoot the number of shots to shoot
   * @param flywheel          flywheel
   * @param limelight         limelight
   * @param belts             belts
   */
  public ShootCommand(int powerCellsToShoot, Flywheel flywheel, Belts belts) {
    this.flywheel = flywheel;
    this.belts = belts;
    this.powerCellsToShoot = powerCellsToShoot;
    addRequirements(flywheel, belts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = getFPGATimestamp();
    initCellCount = flywheel.getCellsShotCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 4000 / 15 = 267 so multiply distance by 267 to get RPM?
    double velocity = kFlywheel.RPM;

    if (getFPGATimestamp() > (initTime + (delay / 4))) {
      flywheel.setVelocity(velocity);
    }

    if (getFPGATimestamp() > (initTime + delay)) {
      if (flywheel.getVelocity() > velocity * 0.95) {
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
    if (powerCellsToShoot > 0) {
      return flywheel.getCellsShotCount() == (initCellCount + powerCellsToShoot);
    } else {
      return false;
    }
  }
}
