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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 ColorSensor = new ColorSensorV3(i2cPort);

  public Color getReadColor() {
    return ColorSensor.getColor();
  }

  /*******************************************************/
  public int stick;
  TalonFX ColorMotor = new TalonFX(0);
  // ColorMotor.configFactoryDefault();
  // ColorMotor.openloop
  // ColorMotor.setNeutralMode(NeutralMode.brake);
  public int[] speeds = { -7, -5, -1, 0, 1, 5, 7 };

  public void ColorMotorSet(int speedLevel) {
    switch (speedLevel) {
    case -3:
      stick = speeds[0];
    case -2:
      stick = speeds[1];
    case -1:
      stick = speeds[2];
    case 0:
      stick = speeds[3];
    case 1:
      stick = speeds[4];
    case 2:
      stick = speeds[5];
    case 3:
      stick = speeds[6];
    default:
      stick = speeds[3];
    }
    ColorMotor.set(ControlMode.PercentOutput, stick);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
