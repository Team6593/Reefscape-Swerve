// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    
    public static class OperatorConstants {
        public static final int L4 = 1;
        public static final int L3 = 2;
        public static final int HOME = 3;
        public static final int StopAll = 10;
        public static final int algaeIn = 4;
        public static final int algaeOut = 5;
        public static final int algaeBack = 6;
        public static final int intakeCoral = 7;
        public static final int shootCoral = 8;
        public static final int algaePivot = 9;
    }

    public static class ElevatorConstants {
        public static final int mainElevatorID = 31;
        public static final int leftElevatorMotorID = 32;
        public static final double minTick = 0;
        public static final double maxTick = 250; // grab this immediatly!!
        public static final double L4Tick = 0;
    }

    public static class CoralIntakeConstants {
        public static final int rightCoralMotorID = 21;
        public static final int leftCoralMotorID = 22;
        public static final int beamBreakID = 0;
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
        public static final double lensHeightInches = 12.5; // grab later
        public static final double goalHeightInches = 13.125; // grab later
    }

}
