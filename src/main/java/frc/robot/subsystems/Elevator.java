// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.EleavtorConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private SparkMax leftMotor = new SparkMax(EleavtorConstants.leftElevatorMotorID, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(EleavtorConstants.rightElevatorMotorID, MotorType.kBrushless);
  private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  /** Creates a new Elevator. */
  public Elevator() {
    leftConfig.inverted(false);
    rightConfig.inverted(true);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    leftController.setReference(30, ControlType.kCurrent);
    rightController.setReference(30, ControlType.kCurrent);
  }

  public void MoveUp(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void MoveDown(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(-speed);
  }

  public void MoveToSetpoint(double setpoint, double speed) {
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
