// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    /** Creates a new DriveTrain. */
    public final WPI_TalonSRX leftRearMotor = new WPI_TalonSRX(DriveConstants.kLeftRearMotor);

    public final WPI_TalonSRX rightRearMotor = new WPI_TalonSRX(DriveConstants.kRightRearMotor);
    private final WPI_VictorSPX leftFrontMotor = new WPI_VictorSPX(DriveConstants.kLeftFrontMotor);
    private final WPI_VictorSPX rightFrontMotor = new WPI_VictorSPX(DriveConstants.kRightFrontMotor);

    private final DifferentialDrive m_drive = new DifferentialDrive(leftRearMotor, rightRearMotor);

    public final DifferentialDriveOdometry m_odometry;
    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private double target;
    private final Field2d m_field = new Field2d();
    private NeutralMode defaultMode = NeutralMode.Brake;

    public DriveSubsystem() {
        leftFrontMotor.setInverted(DriveConstants.kLeftFrontMotorInverted);
        leftRearMotor.setInverted(DriveConstants.kLeftRearMotorInverted);
        rightFrontMotor.setInverted(DriveConstants.kRightFrontMotorInverted);
        rightRearMotor.setInverted(DriveConstants.kRightRearMotorInverted);

        leftFrontMotor.setNeutralMode(defaultMode);
        rightFrontMotor.setNeutralMode(defaultMode);
        leftRearMotor.setNeutralMode(defaultMode);
        rightRearMotor.setNeutralMode(defaultMode);

        leftFrontMotor.follow(leftRearMotor);
        rightFrontMotor.follow(rightRearMotor);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        SmartDashboard.putData("Field", m_field);

        zeroHeading();
        resetEncoders();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(
                Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());

        m_field.setRobotPose(m_odometry.getPoseMeters().getX(), -m_odometry.getPoseMeters().getY(), new Rotation2d(Math.toRadians(-getHeading())));



        // System.out.println("Speeds : " + getWheelSpeeds());
        //System.out.println("Heading : " + getHeading());
        // System.out.println("Pose : " + getPose());
        // System.out.println("Left Encoder Pos : " + getLeftEncoderDistance());
        // System.out.println("Right Encoder Pos : " + getRightEncoderDistance());

    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftRearMotor.setVoltage(leftVolts);
        /*
        if (rightVolts >= 0) {
            rightVolts = rightVolts + 1;
        } else {
            rightVolts = rightVolts - 1;
        }
        */
        rightRearMotor.setVoltage(-rightVolts);

        m_drive.feed();
    }

    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot, true);
    }

    public void tankDrive(double l, double r) {
        m_drive.tankDrive(l, r, true);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                10.0
                        * rightRearMotor.getSelectedSensorVelocity()
                        * (1.0 / DriveConstants.kEncoderCPR)
                        * (Math.PI * DriveConstants.kWheelDiameterMeters),
                10.0
                        * leftRearMotor.getSelectedSensorVelocity()
                        * (1.0 / DriveConstants.kEncoderCPR)
                        * (-Math.PI * DriveConstants.kWheelDiameterMeters));
    }

    public double getLeftWheelVelocity() {
        return getWheelSpeeds().leftMetersPerSecond;
    }

    public double getRightWheelVelocity() {
        return getWheelSpeeds().rightMetersPerSecond;
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void resetEncoders() {
        rightRearMotor.setSelectedSensorPosition(0);
        leftRearMotor.setSelectedSensorPosition(0);
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public double getHeadingCW() {
        // Not negating
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public double getTurnRateCW() {
        // Not negating
        return m_gyro.getRate();
    }

    public double getTarget() {
        return getHeading() + target;
    }

    public void setTarget(double val) {
        target = val;
    }

    public double getRightEncoderDistance() {
        return rightRearMotor.getSelectedSensorPosition()
                * (1.0 / DriveConstants.kEncoderCPR)
                * (Math.PI * DriveConstants.kWheelDiameterMeters);
    }

    public double getLeftEncoderDistance() {
        return leftRearMotor.getSelectedSensorPosition()
                * (1.0 / DriveConstants.kEncoderCPR)
                * (-Math.PI * DriveConstants.kWheelDiameterMeters);
    }

    public double getAverageEncoderDistance() {
        return (getRightEncoderDistance() + getLeftEncoderDistance()) / (2.0);
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}
