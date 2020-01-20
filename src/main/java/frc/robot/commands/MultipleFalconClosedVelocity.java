/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconClosedLoop;


public class MultipleFalconClosedVelocity extends CommandBase {
  /**
   * Creates a new EnableCIMClosedLoop.
   */
  FalconClosedLoop[] closedLoops;
  double velocity;
  public MultipleFalconClosedVelocity(double velocity,FalconClosedLoop... loops) {
    this.velocity = velocity;
    this.closedLoops = loops;
    for (FalconClosedLoop loop: loops) {
      loop.setClosedLoopMode(ControlMode.Velocity);

    }
    addRequirements(loops);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (FalconClosedLoop loop:closedLoops) {
      loop.setVelocity(velocity); //Determine Velocity
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
