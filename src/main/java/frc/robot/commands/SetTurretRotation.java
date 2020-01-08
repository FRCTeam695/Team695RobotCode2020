/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ModelTurret;

public class SetTurretRotation extends CommandBase {
  ModelTurret TurretControlled;
  double horizontalAngle;
  double verticalAngle;
  /**
   * Creates a new SetTurretRotation.
   */
  public SetTurretRotation(ModelTurret TurretControlled,double horizontalAngle,double verticalAngle) {
    this.TurretControlled = TurretControlled;
    this.horizontalAngle = horizontalAngle;
    this.verticalAngle = verticalAngle;
    addRequirements(TurretControlled);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      TurretControlled.setXServoAngle(horizontalAngle);
      TurretControlled.setYServoAngle(verticalAngle);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
