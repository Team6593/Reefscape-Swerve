// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private TalonFX pivotMotor = new TalonFX(ClimberConstants.climberID);
  private TalonFX winchMotor = new TalonFX(ClimberConstants.winchID);
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  


  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                                                  .withCurrentLimits(
                                                    new CurrentLimitsConfigs()
                                                      .withStatorCurrentLimit(40)
                                                      .withStatorCurrentLimitEnable(true)
                                                      .withSupplyCurrentLimit(40)
                                                      .withSupplyCurrentLimitEnable(true)
                                                  );

  /** Creates a new Climber. */
  public Climber() {

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 60;
    pivotMotor.getConfigurator().apply(slot0Configs);
    winchMotor.getConfigurator().apply(slot0Configs); 

    pivotMotor.getConfigurator().apply(motorConfigs);
    winchMotor.getConfigurator().apply(motorConfigs);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    winchMotor.setNeutralMode(NeutralModeValue.Brake);

    pivotMotor.clearStickyFaults();
    winchMotor.clearStickyFaults();
    
    winchMotor.getConfigurator().setPosition(0);
    pivotMotor.getConfigurator().setPosition(0);

    motorConfigs.Voltage.withPeakForwardVoltage(Volts.of(9))
      .withPeakReverseVoltage(Volts.of(-8));

    motorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kS = .25;
    // slot0Configs.kV = .12;
    // slot0Configs.kP = .1;
    // slot0Configs.kI = 0;
    // slot0Configs.kD = .1;
    // pivotMotor.getConfigurator().apply(slot0Configs);

    // final TrapezoidProfile profile = new TrapezoidProfile(
    //   new TrapezoidProfile.Constraints(1, 4)
    // );
    // TrapezoidProfile.State goal = new TrapezoidProfile.State(4, 0);
    // TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    // final PositionVoltage request = new PositionVoltage(0).withSlot(0);

    // setpoint = profile.calculate(.020, setpoint, goal);

    // request.Position = setpoint.position;
    // request.Velocity = setpoint.velocity;
    // pivotMotor.setControl(request);

  }

  public void pivotIn(double setpoint) {
    pivotMotor.setControl(positionTorque.withPosition(setpoint));
  }

  public void reelIn(double setpoint) {
    winchMotor.setControl(request.withPosition(setpoint));
  }

  public void moveWinch(double speed) {
    winchMotor.set(speed);
  }

  public void movePivot(double speed) {
    pivotMotor.set(speed);
  }

  public boolean pivotAtSetpoint() {
    if (pivotMotor.getPosition().getValueAsDouble() > 7.1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean winchAtSetpoint() {
    if (winchMotor.getPosition().getValueAsDouble() < -795) {
      return true;
    } else {
      return false;
    }
  }

  public void pivot(double speed) {
    pivotMotor.set(speed);
    winchMotor.set(speed);
  }

  public void pivotOnly(double speed) {
    pivotMotor.set(speed);
  }

  public void stopClimber() {
    pivotMotor.stopMotor();
    winchMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Pivot Position", pivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Winch Position", winchMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
