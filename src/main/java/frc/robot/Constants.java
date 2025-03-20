// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    // TODO: LLSettings and LLSettings2 setpoint values might have to be reconfigured at comp practice field

    public static class LLSettings {
        public static final double TX_VALUE = -12.2;
        // -87.08
        // 3.51
        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 1.1;
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
        // -0.16
        // -.29
        public static final double X_SETPOINT_REEF_ALIGNMENT = -0.03;
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0;
        // -0.03
        // -0.30
        public static final double Y_SETPOINT_REEF_ALIGNMENT = -0.06;
        public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHTSIDE = -0.06;
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0;
        public static final double DONT_SEE_TAG_WAIT_TIME = .5;
        public static final double POSE_VALIDATION_TIME = 3;
    }

    public static class LLSettings2 {
        public static final double TX_VALUE = -12.2;
        // -87.08
        // 3.51
        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 1.1;
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
        // -0.16
        // -.29
        public static final double X_SETPOINT_REEF_ALIGNMENT = -0.03;
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0;
        // -0.03
        // -0.30
        public static final double Y_SETPOINT_REEF_ALIGNMENT = -0.06;
        public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHTSIDE = -0.06;
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0;
        public static final double DONT_SEE_TAG_WAIT_TIME = .5;
        public static final double POSE_VALIDATION_TIME = 3;
    }
    
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
