// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class Coral extends SubsystemBase {

  private SparkMax intakeMotor = new SparkMax(CoralIntakeConstants.rightCoralMotorID, MotorType.kBrushless);
  private SparkClosedLoopController rightController = intakeMotor.getClosedLoopController();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
  //private DigitalInput beamBrake = new DigitalInput(CoralIntakeConstants.beamBreakID);

  /** Creates a new Coral. */
  public Coral() {
    rightConfig.inverted(true);

    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);

    intakeMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightController.setReference(10, ControlType.kCurrent);

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    distanceSensor.setAutomaticMode(true);
    distanceSensor.setEnabled(true);
    distanceSensor.setRangeProfile(RangeProfile.kHighAccuracy);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Outtake BB", beamBrake.get());
    SmartDashboard.putNumber("Coral Intake voltage", intakeMotor.getBusVoltage());
    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putBoolean("Range valid", distanceSensor.isRangeValid());
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
    SmartDashboard.putNumber("Timestamp", distanceSensor.getTimestamp());
  }

  public void disableSensor() {
    distanceSensor.setAutomaticMode(false);
  }

  /**
   * Manually intake the coral without a beam brake.
   * @param speed
   */
  public void manualIntakeCoral(double speed) {
    intakeMotor.set(-speed);
  }

  public boolean hasCoral() {
    double distance = distanceSensor.getRange();
    distance = Math.round(distance);
    if(distance > 1) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Intake the coral with the beam brake.
   * @param speed
   */
  public void intakeCoral(double speed) {
    if(!hasCoral()) {
      intakeMotor.set(speed);
    } else {
      stop();
    }
  }

  /**
   * Output the coral
   */
  public void shootCoral(double speed) {
    intakeMotor.set(speed);
  }


  public void shootWithoutBrake(double speed) {
    intakeMotor.set(-speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}