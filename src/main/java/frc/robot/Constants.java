// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Controller {
    //USB port number of the xbox controllers
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;
   //Motor ID Numbers
    //CoralHold
    public static final int ID_CORAL_HOLD_MOTOR = 1;
    //CoralPivot 
    public static final int ID_CORAL_PIVOT = 2; //BRUSHED!!! 
  }

public static class CoralHold {
  public static final int DIO_COUNTER = 10;
  public static final double HOLD_SPEED = 0.2;
  public static final double RELEASE_SPEED = -0.25;
}
  
  public static class CoralPivot {
    public static final int DIO_LIMIT = 8; 
    public static final int DIO_ENC_A = 11;
    public static final int DIO_ENC_B = 12;
    public static final double ENC_REVS_MAX = -119;
    public static final double ENC_REVS_LEVEL2 = -4;
    public static final double ENC_REVS_LEVEL4 = -114;
    public static final double ENC_REVS_FULL_RETRACT = 0;
    public static final double MAN_EXT_SPEED = -0.6;
    public static final double MAN_RET_SPEED = 0.6;
    public static final double KP = 0.05; 
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
  }
  
  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
}

}