/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  public static VictorSPX spinMotor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    

  @Override
  public void robotInit() {
  
  spinMotor = new VictorSPX(1)
  
  m_colorMatcher.addColorMatch(kBlueTarget);
  m_colorMatcher.addColorMatch(kGreenTarget);
  m_colorMatcher.addColorMatch(kRedTarget);
  m_colorMatcher.addColorMatch(kYellowTarget);  
       
  }

@Override
    public void robotPeriodic() {

        Color detectedColor = m_colorSensor.getColor();

        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

       if (match.color == kBlueTarget) {
         colorString = "Blue";
       } else if (match.color == kRedTarget) {
         colorString = "Red";
       } else if (match.color == kGreenTarget) {
         colorString = "Green";
       } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
       } else {
        colorString = "Unknown";
       }

       SmartDashboard.putNumber("Red", detectedColor.red);
       SmartDashboard.putNumber("Green", detectedColor.green);
       SmartDashboard.putNumber("Blue", detectedColor.blue);
       SmartDashboard.putNumber("Confidence", match.confidence);
       SmartDashboard.putString("Detected Color", colorString);
  }
}

  @Override
  public void autonomousInit() {
  
  spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +aTurn +aRotate);

  
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

  If (match.color == kBlueTarget) 
  {spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +0.2*aTurn +aRotate);
} else if (match.color == kRedTarget)
  {spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +0.3*aTurn +aRotate);
} else if (match.color == kGreenTarget) 
{spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +0.4*aTurn +aRotate);
} else if (match.color == kYellowTarget)
{spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +0.5*aTurn +aRotate);
} else {colorString = "Unknown";} 
{spinMotor.set(ControlMode.PercentOutput, -aFrontBack +aLeftRight +aTurn +aRotate);
}


  }
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
