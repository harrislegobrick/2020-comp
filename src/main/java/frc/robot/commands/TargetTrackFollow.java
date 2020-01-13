/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TargetTrackFollow extends CommandBase {
  private final Turret turret;
  private final Limelight limelight;
  
  /**
   * Creates a new LimelightTrack.
   */
  public TargetTrackFollow(Turret turret, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret; 
    this.limelight = limelight; 
    addRequirements(this.turret, this.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if limelight can see target, make the LL's x be the error value sent into pid loop
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
