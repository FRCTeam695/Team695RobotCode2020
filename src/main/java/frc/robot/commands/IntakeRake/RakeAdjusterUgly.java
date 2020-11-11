/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeRake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRake;

public class RakeAdjusterUgly extends CommandBase {
  private IntakeRake IntakeRake_Inst;
  private Joystick ControllerDrive;
  private int axisId;
  private double lastAxisReading = 0;
  /**
   * Creates a new LowerIntakeUgly.
   */
  public RakeAdjusterUgly(IntakeRake IntakeRake_Inst, Joystick ControllerDrive,int axisId) {
    this.IntakeRake_Inst = IntakeRake_Inst;
    this.ControllerDrive = ControllerDrive;
    this.axisId = axisId;
    addRequirements(IntakeRake_Inst);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axisReading = ControllerDrive.getRawAxis(axisId);
    if (axisReading < 0.0001) {
      axisReading = 0;
    }
    boolean stateChangedToDisabled = (axisReading == 0);
    boolean stateChangedToEnabled = (lastAxisReading == 0);
    boolean enableStateToggled = (axisReading != lastAxisReading) && (stateChangedToDisabled || stateChangedToEnabled);
    lastAxisReading = axisReading;

    if (enableStateToggled) {
      if (stateChangedToDisabled) {
        IntakeRake_Inst.disableRake();
      }
      else if (stateChangedToEnabled) {
        IntakeRake_Inst.enableRake();
      }
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
