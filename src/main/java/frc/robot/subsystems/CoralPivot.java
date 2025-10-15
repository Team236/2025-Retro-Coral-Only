// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralPivot extends SubsystemBase {
  
  private SparkMax coralPivotMotor;
  private SparkBaseConfig coralPivotConfig;
  private Encoder coralPivotEncoder; //use for external encoders
  //private RelativeEncoder coralPivotEncoder; //use for encoders integrated into motor (Neos)
  private boolean isCoralPivotException;  //tests if limit switch exception thrown
  private DigitalInput CoralLimit;
 

    /** Creates a new CoralPivot. */
    public CoralPivot() {
    //NOTE- Using a brushed motor (NOT a Neo motor), with SparkMax controller 
    coralPivotMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_PIVOT, MotorType.kBrushed);
    //NOTE- Using external encoder (DIO channels A/B needed, rather than encoder integrated into motor)
    //coralPivotEncoder = coralPivotMotor.getEncoder();/use this for motors with integrated encoders (like Neo)
    coralPivotEncoder = new Encoder(Constants.CoralPivot.DIO_ENC_A, Constants.CoralPivot.DIO_ENC_B);

    coralPivotConfig = new SparkMaxConfig();
    coralPivotConfig.inverted(false); 
    coralPivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    coralPivotMotor.configure(coralPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
      CoralLimit = new DigitalInput(Constants.CoralPivot.DIO_LIMIT);
    } catch (Exception e) {
       isCoralPivotException = true;
      SmartDashboard.putBoolean("exception thrown for Coral limit: ", isCoralPivotException);
    }
}

// methods start here

public double getCoralEncoder() {  //gives encoder reading in revs
  //return coralPivotEncoder.getPosition(); // if using internal encoder
  //below is for extenal Bourne encoder (512 counts per rev):
  return coralPivotEncoder.getRaw(); //getRaw gets actual count unscaled by the 1, 2 or 4x scale
  //return coralPivotEncoder.get(); //gets count adjusted for the 1, 2 or 4x scale factor
}

public void resetCoralEncoder() {
  coralPivotEncoder.reset();
}

public void stopCoralPivot() {
  coralPivotMotor.set(0);
}

public double getCoralPivotSpeed() {
  return coralPivotMotor.get();
}

public boolean isCoralLimit() {
if (isCoralPivotException) {
  return true;
} else {
  return CoralLimit.get();
}
}

public boolean isFullyExtended() { //Note - revs max is a negative number
  return (getCoralEncoder() <= Constants.CoralPivot.ENC_REVS_MAX);
}

public void setCoralPivotSpeed(double speed) {
  if (speed <= 0) { //negative speed means extending 
    if (isFullyExtended()) {
      // if extend limit is hit or at the max desired extension and going out, stop 
        stopCoralPivot();
     }  else {
        //extending out but fully extended limit is not tripped, go at commanded speed
       coralPivotMotor.set(speed);
      }
 } 
  else { //speed > 0, so retracting
      if (isCoralLimit()) {
        //retract limit is hit, so stop and zero encoder
        stopCoralPivot();
        resetCoralEncoder();
      } else {
        // retract limit is not hit, so go at commanded speed
        coralPivotMotor.set(speed); 
      }
     }

}

//*** BELOW IS OLD CODE FOR USING SPARKMAX PID position control FOR A TILT SUBSYSTEM*/
//Currently using WPILib PID vice SparkMax PID due to spurious encoder
// polarity changes when run multiple autos in a row with SparkMax PID
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/

@Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Pivot Retract Limit is hit: ", isCoralLimit());
    SmartDashboard.putBoolean("Coral Pivot is fully extended: ", isFullyExtended());
    SmartDashboard.putNumber("Coral Pivot Encoder Revolutions: ", getCoralEncoder());
    SmartDashboard.putNumber("coral pivot speed: ", getCoralPivotSpeed());
  }

} 