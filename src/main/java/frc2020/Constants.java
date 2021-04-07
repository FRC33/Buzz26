/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc2020;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;
import frc2020.subsystems.SwerveModule.SwerveModuleConstants;

public final class Constants {

    // ------- CHANGE THIS VALUE DEPENDING ON WHICH ROBOT THIS IS -------
    public static final WhichRobot kWhichRobot = WhichRobot.PAT;

    public static final boolean kRapidFire = true;

    // region ------ Device constants ------
    /** Encoder counts per revolution of the Falcon motor integrated encoder */
    public static final int kFalconCPR = 2048;
    /** rps/V */
    public static final double kFalconKV = 6380d / 12 / 60; //Free speed RPM / nominal voltage / 60 seconds
    /** V/A */
    public static final double kFalconKA = 12d / 257; //Nominal voltage / stall current
    /** Encoder counts per revolution of the CANCoder when used as an encoder */
    public static final int kCANCoderCPR = 4096;
    // endregion -----------------------------

    // TeleOp drive
    /** Set this to true to drive with one joystick */
    public static final double kDriveGearReduction = 11.3666;
    public static final double kQuickTurnEnableThrottle = 0.15;
    public static final double kQuickTurnGain = 0.9;
    public static final double kSpeedTurnGain = 0.03937;
    public static final double kSpeedTurnGainIntaking = 0.03937;
    public static final double kDriveCurrentLimit = 60;
    /** If yaw rate is calculated correctly (deg/s) in {@link frc2020.subsystems.Drive#readPeriodicInputs()}, this should stay as 1 */
    public static final double kYawFactor = 1;
    /** V/deg/s */
    public static final double kYawControlGain = 0.01;
    /** V */
    public static final double kYawMaxCorrection = 4;
    public static final double kYawLead = 0.3;
    public static final double kYawLag = 0.5;
    public static final double kAutoSteerKp = 8;
    public static final double kAutoSteerKi = 0.8;
    public static final double kAutoSteerKd = 0.35;
    public static final double kAutoSteerKiZone = 3;
    public static final double kAutoSteerMaxOutput = 8;

    /*
    145 - bounce
    145 - hyperdrive
    200 - barrel
    150 - slalom
    */
    public static final double kDriveMaxLinearVelocity = Units.inchesToMeters(120);
    public static final double kDriveMaxAngularVelocity = 45.0 / 16.0; // Units do not matter
    
    public static final double kDriveJoystickDeadbandCutoff = 0.08;
    public static final double kDriveJoystickWeight = 0.5;

    public static final double kDriveSteerJoystickDeadbandCutoff = 0.08;
    public static final double kDriveSteerJoystickWeight = 1.0;

    public static final double kDriveLength = Units.inchesToMeters(16);
    public static final double kDriveWidth = Units.inchesToMeters(16);
    public static final SwerveDriveKinematics kSwerveKinematics;
    static {
        Translation2d frontRightLocation = new Translation2d(kDriveLength, -kDriveWidth);
        Translation2d frontLeftLocation = new Translation2d(kDriveLength, kDriveWidth);
        Translation2d backLeftLocation = new Translation2d(-kDriveLength, kDriveWidth);
        Translation2d backRightLocation = new Translation2d(-kDriveLength, -kDriveWidth);
    
        kSwerveKinematics = new SwerveDriveKinematics(
            frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation
        );
    }

    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveMotorId = 1;
        kFrontRightModuleConstants.kSteerMotorId = 2;
        kFrontRightModuleConstants.kSteerEncoderId = 7;

        kFrontRightModuleConstants.kSteerInverted = true;

        kFrontRightModuleConstants.kSteerMotorGearReduction = 13.2;
        kFrontRightModuleConstants.kSteerEncoderOffset = 0.764946069123652;

        kFrontRightModuleConstants.kSteerKf = (1023 * 0.4370) / 9057;
        kFrontRightModuleConstants.kSteerKp = 0.1;
        kFrontRightModuleConstants.kSteerKv *= (13.2 / 15.33);
        kFrontRightModuleConstants.kSteerKa *= (13.2 / 15.33);
    }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveMotorId = 7;
        kBackRightModuleConstants.kSteerMotorId = 8;
        kBackRightModuleConstants.kSteerEncoderId = 6;

        kBackRightModuleConstants.kSteerInverted = true;

        kBackRightModuleConstants.kSteerMotorGearReduction = 13.2;
        kBackRightModuleConstants.kSteerEncoderOffset = 0.565444764136119;

        kBackRightModuleConstants.kSteerKf = (1023 * 0.4370) / 9057;
        kBackRightModuleConstants.kSteerKp = 0.1;
        kBackRightModuleConstants.kSteerKv *= (13.2 / 15.33);
        kBackRightModuleConstants.kSteerKa *= (13.2 / 15.33);
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveMotorId = 3;
        kFrontLeftModuleConstants.kSteerMotorId = 4;
        kFrontLeftModuleConstants.kSteerEncoderId = 9;

        kFrontLeftModuleConstants.kSteerInverted = false;

        kFrontLeftModuleConstants.kSteerMotorGearReduction = 15.33;
        kFrontLeftModuleConstants.kSteerEncoderOffset = 0.828;
    }

    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();
    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveMotorId = 5;
        kBackLeftModuleConstants.kSteerMotorId = 6;
        kBackLeftModuleConstants.kSteerEncoderId = 8;

        kBackLeftModuleConstants.kSteerInverted = false;
        
        kBackLeftModuleConstants.kSteerMotorGearReduction = 15.33;
        kBackLeftModuleConstants.kSteerEncoderOffset = 0.922648673066217;
    }
    
    // Intake
    public static final double kIntakeStallCurrent = 90;
    /** Seconds intake motor(s) have to be at stall current to be considered stalled */
    public static final double kIntakeStallTime = 0.15;

    // Hood
    public static final double kHoodEncoderReduction = 7.81;
    public static final RobotConstant<Double> kHoodSensorOffset = new RobotConstant<>(0.73, 0.0);
    public static final double kHoodAngleOffset = 25;
    public static final double kHoodAngleMin = 27;
    public static final double kHoodAngleMax = 51.5;
    public static final double kHoodKp = 2.0;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 0;
    public static final double kHoodKiZone = 1;

    // Shooter
    public static final double kShooterGearReduction = 1;
    public static final double kShooterDiameter = 4.0;
    public static final double kShooterRampRate = 0;
    public static final int kShooterSlotIdx = 0;
    public static final double kShooterKp = 0.07;
    public static final double kShooterKi = 0;
    public static final double kShooterKd = 1.0;
    public static final double kShooterKf = (1023 * (11.0 / 11.5)) / 19552;
    
    // region ------ Device IDs ------
    // Talons
    public static final int kIntakeId = 9;
    public static final int kIndexerId = 10;

    public static final int kFeederId = 11;

    public static final int kShooterAId = 12;
    public static final int kShooterBId = 13;

    // CANCoders
    public static final int kHoodEncoderId = 33;

    // PWM
    public static final int kLEDId = 0;
    public static final int kHoodAId = 8;
    public static final int kHoodBId = 9;

    // DIO
    public static final int kBallSensorIds[] = {5, 4};
    public static final int kPixyDigitalInputId = 3;

    // Analog
    public static final int kPixyAnalogInputId = 0;
    
    // Solins
    public static final int kIntakeForwardId = 1;
    public static final int kIntakeReverseId = 0;
    // endregion ------------------------ -

    public static enum WhichRobot {
        PAT, VANNA
    } 

    public static class RobotConstant<T> {
        T mPatConstant;
        T mVannaConstant;

        public RobotConstant(T patConstant, T vannaConstant) {
            mPatConstant = patConstant;
            mVannaConstant = vannaConstant;
        }

        public T get() {
            return kWhichRobot == WhichRobot.VANNA ? mVannaConstant : mPatConstant;
        }
    } 
}
