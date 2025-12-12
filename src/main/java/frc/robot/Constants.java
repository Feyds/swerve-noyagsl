package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class SwerveConstants {
        // --- Fiziksel Özellikler ---
        // Tekerlekler arası mesafe (Track Width ve Wheel Base)
        // Örnek: 24 inç x 24 inç bir robot için (yaklaşık 0.6 metre)
        public static final double TRACK_WIDTH = Units.inchesToMeters(30);
        public static final double WHEEL_BASE = Units.inchesToMeters(30);

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0; 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        
        // Dönüşüm Faktörleri (Encoder'dan metreye veya radyana)
        public static final double DRIVE_POS_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VEL_FACTOR = DRIVE_POS_FACTOR / 60.0; // RPM -> m/s
        public static final double ANGLE_POS_FACTOR = (2 * Math.PI) / ANGLE_GEAR_RATIO; // Rotasyon -> Radyan

        // --- CAN ID'leri ---
        // Pigeon 2.0
        public static final int PIGEON_ID = 20;

        // Front Left
        public static final int FL_DRIVE_ID = 2;
        public static final int FL_ANGLE_ID = 3;
        public static final int FL_CANCODER_ID = 21;
        public static final double FL_OFFSET = -Math.toRadians(0.0); // Cancoder Offseti (Dereceyi radyana çevir)

        // Front Right
        public static final int FR_DRIVE_ID = 4;
        public static final int FR_ANGLE_ID = 5;
        public static final int FR_CANCODER_ID = 22;
        public static final double FR_OFFSET = -Math.toRadians(0.0);

        // Back Left
        public static final int BL_DRIVE_ID = 6;
        public static final int BL_ANGLE_ID = 7;
        public static final int BL_CANCODER_ID = 23;
        public static final double BL_OFFSET = -Math.toRadians(0.0);

        // Back Right
        public static final int BR_DRIVE_ID = 8;
        public static final int BR_ANGLE_ID = 9;
        public static final int BR_CANCODER_ID = 24;
        public static final double BR_OFFSET = -Math.toRadians(0.0);


        public static final double ANGLE_P = 0.01; 
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        
        public static final double DRIVE_P = 0.05; 
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;

        // --- Kinematik ---
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),   // FL
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // FR
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  // BL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // BR
        );
        
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    }
}