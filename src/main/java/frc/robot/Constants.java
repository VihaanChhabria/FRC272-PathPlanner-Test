package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final class SwerveConstants {
        public static final int pigeonID = 29;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
            COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1
        );
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21);
        public static final double wheelBase = Units.inchesToMeters(18);
        public static final double wheelCircumference =
          chosenModule.wheelCircumference;
    

        /* Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
    
        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 3.0; //NOTE: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //NOTE: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //DEBUG: This must be tuned to specific robot

            public static final int driveMotorID = 17;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(
                -0.109
            );
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }
        
            /* Front Right Module - Module 1 */
        public static final class Mod1 { //DEBUG: This must be tuned to specific robot
        
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(
                -0.366
            );
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }
        
            /* Back Left Module - Module 2 */
        public static final class Mod2 { //DEBUG: This must be tuned to specific robot
        
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(
                -.231
            );
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }
        
            /* Back Right Module - Module 3 */
        public static final class Mod3 { //DEBUG: This must be tuned to specific robot
        
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(
                -.153
            );
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(driveKP, 0, 0), // Translation constants 
            new PIDConstants(16.0/*chosenModule.angleKP*/, 0, 0), // Rotation constants 
            maxSpeed, 
            0.56569, // Drive base radius (distance from center to furthest module) I put it in meters
            new ReplanningConfig()
        );
    }
}
