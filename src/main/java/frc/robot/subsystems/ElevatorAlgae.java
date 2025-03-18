// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorAlgae extends SubsystemBase {

  private SparkMax elevatorAlgaeMotor = new SparkMax(0, MotorType.kBrushless);
  private SparkMaxConfig elevatorAlgaeConfig = new SparkMaxConfig();
  private SparkClosedLoopController elevatorAlgaeController = elevatorAlgaeMotor.getClosedLoopController();
  private SparkAbsoluteEncoder elevatorAlgaeEncoder = elevatorAlgaeMotor.getAbsoluteEncoder();

  /** Creates a new ElevatorAlgae. */
  public ElevatorAlgae() {
    elevatorAlgaeConfig.idleMode(IdleMode.kBrake);
    elevatorAlgaeMotor.configure(elevatorAlgaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorAlgaeController.setReference(30, ControlType.kCurrent);
  }

  public void moveToSetpoint(double setpoint) {
    elevatorAlgaeController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
  }

  public void stop() {
    elevatorAlgaeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Algae Position", elevatorAlgaeEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
