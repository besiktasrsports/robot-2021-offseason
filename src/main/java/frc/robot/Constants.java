// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
    public static final class JoystickConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
    // Drive

    public static final class DriveConstants {
        public static final int kLeftRearMotor = 11;
        public static final int kLeftFrontMotor = 12;
        public static final int kRightRearMotor = 13;
        public static final int kRightFrontMotor = 10;
        public static final boolean kLeftRearMotorInverted = true;
        public static final boolean kLeftFrontMotorInverted = true;
        public static final boolean kRightRearMotorInverted = true;
        public static final boolean kRightFrontMotorInverted = true;

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kPDriveVel = 8.5;
        public static final double kRamseteZeta = 0.7;
        public static final double kTrackwidthMeters = 0.69;
        public static final double kMaxAutoVoltage = 10;

        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final boolean kGyroReversed = true;

        public static final double kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1524;

        public static final double kTurnP = 0.94;
        public static final double kTurnI = 0.00;
        public static final double kTurnD = 0.04;
        public static final double kMinCommand = 0.07;

        public static final double kMaxTurnRateDegPerS = 120;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
        public static final double kTurnToleranceDeg = 0.5;
        public static final double kTurnRateToleranceDegPerS = 8;

        public static final double kVisionTurnP = 0;
        public static final double kVisionMinCommand = 0;
    }
    // Intake

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 40;
        public static final int kCompressorPort = 0;
        public static final int kPCMPort = 0;
        public static final int kIntakeDoubleSolenoidPort1 = 0;
        public static final int kIntakeDoubleSolenoidPort2 = 1;
    }
    // Funnel
    public static final class FunnelConstants {

        public static final int kFunnelRightMotor = 20;
        public static final int kFunnelLeftMotor = 21;
    }
    // Accelerator
    public static final class AcceleratorConstants {
        public static final int kAcceleratorMotorPort = 50;
    }
    // Turret

    public static final class TurretConstants {

        public static final byte kTurretMotorPort = 0;
        public static final byte kTurretEncoderA = 2;
        public static final byte kTurretEncoderB = 3;
        public static final int kTurretEncoderPPR = 2048; // AMT-103
        public static final byte kToleranceInDegrees = 0;
        public static final byte kTurretHallEffect1Port = 0;
        public static final byte kTurretHallEffect2Port = 1;

        public static NeutralMode kTurretMotorMode = NeutralMode.Brake; // Brake-Coast

        public static final boolean kIsEncoderReversed = false;
        public static final boolean kIsMotorReversed = false;

        public static final double kP = 0.000;
        public static final double kI = 0.000;
        public static final double kD = 0.000;
        public static final double kS = 0.000;
        public static final double kV = 0.000;
        public static final double kA = 0.000;
    }
    // Shooter

    public static final class ShooterConstants {
        public static final int kShooterMotor1Port = 30;
        public static final int kShooterMotor2Port = 31;
        public static final boolean kShooterInvertedMode1 = true;
        public static final boolean kShooterInvertedMode2 = true;
    }

    public static final class MiscConstants {
        public static final int kLedRelayPort = 9;
        public static final int kStatusLEDPort = 0;
        public static final int KStatusLEDLength = 0;
    }
}
