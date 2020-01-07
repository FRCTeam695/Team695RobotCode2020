/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;

public class CompressorController extends SubsystemBase {
  private Compressor CompressorControlled = new Compressor(0);

  /**
   * Creates a new Compressor.
   */
  public CompressorController() {

  }
  public void enableCompressor() {
    CompressorControlled.enabled();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
