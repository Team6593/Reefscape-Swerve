// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class Coral extends SubsystemBase {

  private SparkMax rightMotor = new SparkMax(CoralIntakeConstants.rightCoralMotorID, MotorType.kBrushless);
  private SparkMax leftMotor = new SparkMax(CoralIntakeConstants.leftCoralMotorID, MotorType.kBrushless);
  private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
  private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private DigitalInput beamBrake = new DigitalInput(CoralIntakeConstants.beamBreakID);

  /** Creates a new Coral. */
  public Coral() {
    rightConfig.inverted(true);
    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightController.setReference(10, ControlType.kCurrent);
    leftController.setReference(10, ControlType.kCurrent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Outtake BB", beamBrake.get());
  }

  /**
   * Manually intake the coral without a beam brake.
   * @param speed
   */
  public void manualIntakeCoral(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  /**
   * Intake the coral with the beam brake.
   * @param speed
   */
  public void intakeCoral(double speed) {
    if (beamBrake.get()) {
      rightMotor.set(speed);
      leftMotor.set(speed);
    } else if (!beamBrake.get()) {
      rightMotor.set(0);
      leftMotor.set(0);
    }
}

  /**
   * Output the coral
   */
  public void shootCoral(double speed) {
    if (!beamBrake.get()) {
      rightMotor.set(speed);
      leftMotor.set(speed);
    } else if (beamBrake.get()) {
      rightMotor.set(0);
      leftMotor.set(0);
    }
  }
}