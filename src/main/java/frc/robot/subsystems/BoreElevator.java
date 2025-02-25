// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoreElevator extends SubsystemBase {

  private SparkMax rightMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);
  private SparkMax leftMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);

  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();

  private DigitalInput limitSwitch = new DigitalInput(3);

  // dio ports 0 and 1 (hypothetical)
  private Encoder encoder = new Encoder(0, 1);

  /** Creates a new RioBore. */
  public BoreElevator() {

    rightConfig.inverted(false).idleMode(IdleMode.kBrake);
    leftConfig.inverted(true).idleMode(IdleMode.kBrake);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    encoder.reset();
  }

  public void elevate(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  public double getEncoderReading() {
    return encoder.getDistance();
  }

  public boolean getSwitch() {
    return limitSwitch.get();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public void stop() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }

  @Override
  public void periodic() {

    if (!getSwitch()) {
      resetEncoder();
    }

    SmartDashboard.putBoolean("Elevator Switch Hit?", limitSwitch.get());
    SmartDashboard.putNumber("Right Bore", encoder.getDistance());
    // This method will be called once per scheduler run
  }
}
