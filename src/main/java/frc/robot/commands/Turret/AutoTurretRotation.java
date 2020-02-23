/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.enums.RotationDirection;;

public class AutoTurretRotation extends CommandBase {

  //private ModelTurret TurretControlled;
  private Turret Turret_Inst;
  private RotationDirection CurrentDirection;
  private final double velocity = .5;

  //private int panDirection = 1;
  /**
   * Creates a new SetTurretRotation.
   */
  public AutoTurretRotation(Turret Turret_Inst) {
    this.Turret_Inst = Turret_Inst;
    addRequirements(Turret_Inst);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    
  }

  //second version of execute
  @Override
  public void execute() {
    if(Turret_Inst.isTooFarLeft()) {
      CurrentDirection = RotationDirection.CLOCKWISE;
    }
    if(Turret_Inst.isTooFarRight()) {
      CurrentDirection = RotationDirection.COUNTER_CLOCKWISE;
    }
    Turret_Inst.setPower(CurrentDirection.SIGN_MODIFIER*velocity);

    
  }

  public boolean endCommand() {
    return Turret_Inst.getContourArea() >= 0.02;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return endCommand();
  }
}
