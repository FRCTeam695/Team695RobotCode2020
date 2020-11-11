/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving.ConventionalDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConventionalDriveTrain;

public class AutonomousMove extends CommandBase {
  /**
   * Creates a new AutonomousMove.
   */

  ConventionalDriveTrain ConventionalDriveTrain_Inst; 
  int count = 0;
   
  public AutonomousMove(ConventionalDriveTrain ConventionalDriveTrain_Inst) {
    this.ConventionalDriveTrain_Inst = ConventionalDriveTrain_Inst;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ConventionalDriveTrain_Inst.setPowers(.2, -.2);
    count += 20;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count == 1000;
  }
}
