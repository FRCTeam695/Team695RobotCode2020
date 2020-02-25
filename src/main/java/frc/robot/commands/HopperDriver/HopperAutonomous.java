/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HopperDriver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AdjustableVictor;

public class HopperAutonomous extends CommandBase {
  /**
   * Creates a new HopperAutonomous.
   */
  private double t = 0;
  AdjustableVictor ControlledVictor;
  public HopperAutonomous(AdjustableVictor ControlledVictor) {

    this.ControlledVictor = ControlledVictor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t += 0.02;
    if (t > 2) {
      this.ControlledVictor.setPower(1);
    }
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
