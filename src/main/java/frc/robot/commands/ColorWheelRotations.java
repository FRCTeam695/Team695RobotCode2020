/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//java.util.Date javaDate = new java.util.Date();
//FixedColorWheelConstants Constants = new FixedColorWheelConstants();

import frc.robot.Constants;
import static frc.robot.Constants.FixColorConst;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.FalconClosedLoop;
import frc.robot.subsystems.dashTab.box;
import frc.robot.Dashboard;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;



public class ColorWheelRotations extends PIDCommand {
  /**
   * Creates a new EnableCIMClosedLoop.
   */
  FalconClosedLoop closedLoop;
  private static box dashBox;
  private static double finalRotationsThreshold = FixColorConst.finalRotationsThreshold;
  private static double motorCircumference = 2 * Math.PI * FixColorConst.motorWheelRadius;// cm enter radius
  private static double colorWheelCircumference = 2 * Math.PI * FixColorConst.colorWheelRadius;// enter radius cm
  private static double mechAdvantage = colorWheelCircumference / motorCircumference;
  private static double motorRotationsThreshold = (finalRotationsThreshold * mechAdvantage);
  private double motorRotations = 0;
  private double velocity = 1;
  private double maxVelocity = 10 * mechAdvantage;// number is color wheel rpm
  private double a = -1 * (velocity - maxVelocity) / (Math.pow(motorRotationsThreshold, 2) / 4);
  private static ColorWheel wheel;

  public ColorWheelRotations(ColorWheel wheel, Dashboard dash) {

    super(new PIDController(FixColorConst.kTurnP, 0, FixColorConst.kTurnD),
        // Close loop on heading
        wheel::getDistanceRotations,
        // Set reference to target
        motorRotationsThreshold,
        // Pipe output to turn robot
        output -> execFunction(output, wheel,dash));

  }

  private static void execFunction(double input, ColorWheel wheel, Dashboard dash) {
    wheel.setVelocity(input);
    dash.getFixedColorWheel().set(wheel.getDistanceRotations() / motorRotationsThreshold * 100);

}


  


  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
