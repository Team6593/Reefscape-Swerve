// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    
    public static class CoralIntakeConstants {
        public static final int rightCoralMotorID = 21;
        public static final int leftCoralMotorID = 22;
        public static final int beamBreakID = 0;
    }

    public static class ClimberConstants {
        public static final int climberID = 19;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;
        public static final int kJoystickPort = 0;

        public static final double kElevatorKp = 5;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0;

        public static final double kElevatorkS = 0.0; // volts (V)
        public static final double kElevatorkG = 0.762; // volts (V)
        public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
        public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg

        public static final double kSetpointMeters = 0.75;
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = 1.25;

        // distance per pulse = (distance per revolution) / (pulses per revolution)
        //  = (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse =
            2.0 * Math.PI * kElevatorDrumRadius / 4096;
    }

    public static class EleavtorConstants {
        public static final int leftElevatorMotorID = 31;
        public static final int rightElevatorMotorID = 32;
    }

    public static class LimelightConstants {
        public static final double mountAngleDegrees = 0; // grab later
        public static final double lensHeightInches = 15.5; // grab later
        public static final double goalHeightInches = 10.125; // grab later
    }

}
