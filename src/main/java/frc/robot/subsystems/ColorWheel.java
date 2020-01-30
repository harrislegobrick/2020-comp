/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kColorWheel;

public class ColorWheel extends SubsystemBase {
  private final WPI_VictorSPX motor;
  private final ColorSensorV3 sensor;

  private final ColorMatch matcher = new ColorMatch();
  private final HashMap<Color, Integer> colorToInt = new HashMap<Color, Integer>();
  private final HashMap<Integer, Color> intToColor = new HashMap<Integer, Color>();

  // in order of cw
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * Creates a new ColorWheel.
   */
  public ColorWheel() {
    sensor = new ColorSensorV3(I2C.Port.kOnboard);
    motor = new WPI_VictorSPX(kColorWheel.MOTOR);

    motor.setInverted(kColorWheel.INVERTED);

    matcher.addColorMatch(kBlueTarget);
    matcher.addColorMatch(kGreenTarget);
    matcher.addColorMatch(kRedTarget);
    matcher.addColorMatch(kYellowTarget);

    colorToInt.put(kBlueTarget, 0);
    colorToInt.put(kGreenTarget, 1);
    colorToInt.put(kRedTarget, 2);
    colorToInt.put(kYellowTarget, 3);

    intToColor.put(0, kBlueTarget);
    intToColor.put(1, kGreenTarget);
    intToColor.put(2, kRedTarget);
    intToColor.put(3, kYellowTarget);
  }

  public void runMotor() {
    motor.set(ControlMode.PercentOutput, 0.3);
  }

  public void stopMotor() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Calculates the color that the wheel needs to be over given the number of
   * colors the offset needs to be. Based upon going counterclockwise when looked
   * at from the alliance wall.
   * 
   * @param ammount  : the amount of colors over
   * @param reversed : if true then it will be a cw offset, false would be a ccw
   *                 offset
   * @return the color the sensor needs to detect for the FMS to agree upon the
   *         sent color
   */
  public Color getShiftedColor(int ammount, boolean reversed) {
    int fms = colorToInt.get(getFMSColor());
    int key = reversed ? (fms - ammount) % 4 : (fms + ammount) % 4;
    return intToColor.get(key);
  }

  public Color getRawDetectedColor() {
    return sensor.getColor();
  }

  public Color getColorWheelColor() {
    Color outputColor;
    ColorMatchResult match = matcher.matchClosestColor(getRawDetectedColor());

    if (match.color == kBlueTarget) {
      outputColor = kBlueTarget;
    } else if (match.color == kRedTarget) {
      outputColor = kRedTarget;
    } else if (match.color == kGreenTarget) {
      outputColor = kGreenTarget;
    } else if (match.color == kYellowTarget) {
      outputColor = kGreenTarget;
    } else {
      outputColor = Color.kBlack;
    }
    return outputColor;
  }

  public Color getFMSColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        return kBlueTarget;
      case 'G':
        return kGreenTarget;
      case 'R':
        return kRedTarget;
      case 'Y':
        return kYellowTarget;
      default:
        return Color.kBlack;
      }
    } else {
      return Color.kBlack;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
