/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HopperDriver;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AdjustableVictor;

public class VictorControlJoystickAxis extends CommandBase {
  private AdjustableVictor ControlledVictor;
  private DoubleSupplier AxisInput;

  /**
   * Creates a new VictorControlJoystickAxis.
   */
  public VictorControlJoystickAxis(AdjustableVictor ControlledVictor, DoubleSupplier ControllingAxis) {
    this.ControlledVictor = ControlledVictor;
    this.AxisInput = ControllingAxis;
    addRequirements(ControlledVictor);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public VictorControlJoystickAxis(AdjustableVictor ControlledVictor,Supplier<Vector2d> JoystickAxisInputToUseYComponentOf) {
    this(ControlledVictor, () -> {return JoystickAxisInputToUseYComponentOf.get().y;});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ControlledVictor.setPower(AxisInput.getAsDouble());
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
