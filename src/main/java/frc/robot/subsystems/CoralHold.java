// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralHold extends SubsystemBase {
  // SPARK MAX
  private SparkMax coralHoldMotor;
  private SparkMaxConfig coralHoldConfig;
 
  public CoralHold() {
    coralHoldMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_HOLD_MOTOR , MotorType.kBrushless);
    coralHoldConfig = new SparkMaxConfig();
    coralHoldConfig.inverted(true);
    coralHoldConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    coralHoldMotor.configure(coralHoldConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  //METHODS START HERE
  public void setCoralHSpeed(double speed) {
    coralHoldMotor.set(speed);
  }

  public void coralHStop () {
    coralHoldMotor.set(0);
  }

  public boolean isCoralSpinning() {
    boolean  spin;
    if (Math.abs(coralHoldMotor.get()) >0.08) {
      spin = true;
    }
    else {
      spin = false;
    }
    return spin;
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
   SmartDashboard.putBoolean("Is coral hold spinning: ", isCoralSpinning());
  }

}