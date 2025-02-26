// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    
    public static class OperatorConstants {
        public static final int L3 = 1;
        public static final int L2 = 2;
        public static final int L1 = 3;
        public static final int StopAll = 6;
    }

    public static class ElevatorConstants {
        public static final int rightElevatorMotorID = 31;
        public static final int leftElevatorMotorID = 32;
        public static final double minTick = 0;
        public static final double maxTick = 250; // grab this immediatly!!
        public static final double L4Tick = 0;
    }

    public static class CoralIntakeConstants {
        public static final int rightCoralMotorID = 21;
        public static final int leftCoralMotorID = 22;
        public static final int beamBreakID = 23;
    }

    public static class AlgaeConstants {
        public static final int pivotMotorID = 51;
        public static final int intakeMotorID = 52;
        //public static final int limitSwitchID = 54;
    }

    public static class ClimberConstants {
        public static final int climberID = 41;
        public static final int winchID = 42;
    }

    public static class LimelightConstants {
        public static final double mountAngleDegrees = 0; // grab later
        public static final double lensHeightInches = 15.5; // grab later
        public static final double goalHeightInches = 10.125; // grab later
    }

}
