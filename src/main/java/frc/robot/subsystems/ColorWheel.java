/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 ColorSensor = new ColorSensorV3(i2cPort);

  public Color getReadColor() {
    return ColorSensor.getColor();
  }

  /*******************************************************/
  //TalonFX ColorMotor = new TalonFX(0);
  // ColorMotor.configFactoryDefault();
  // ColorMotor.openloop
  // ColorMotor.setNeutralMode(NeutralMode.brake);
  public int[] speeds = { -7, -5, -1, 0, 1, 5, 7 };

  public void ColorMotorSet(int speedLevel) {
    //System.out.print(String.valueOf(speedLevel)+"  ");

    //ColorMotor.set(ControlMode.PercentOutput, speeds[speedLevel+3]);
    System.out.println(String.valueOf(speeds[speedLevel+3]));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
