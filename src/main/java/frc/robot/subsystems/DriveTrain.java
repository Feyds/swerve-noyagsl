package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Constants.SwerveConstants;

public class DriveTrain {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

    public DriveTrain() {
        frontLeft = new SwerveModule(SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_ANGLE_ID, SwerveConstants.FL_CANCODER_ID, SwerveConstants.FL_OFFSET);
        frontRight = new SwerveModule(SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_ANGLE_ID, SwerveConstants.FR_CANCODER_ID, SwerveConstants.FR_OFFSET);
        backLeft = new SwerveModule(SwerveConstants.BL_DRIVE_ID, SwerveConstants.BL_ANGLE_ID, SwerveConstants.BL_CANCODER_ID, SwerveConstants.BL_OFFSET);
        backRight = new SwerveModule(SwerveConstants.BR_DRIVE_ID, SwerveConstants.BR_ANGLE_ID, SwerveConstants.BR_CANCODER_ID, SwerveConstants.BR_OFFSET);

        gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
        gyro.reset();

        odometry = new SwerveDriveOdometry(
            SwerveConstants.KINEMATICS, 
            getRotation2d(), 
            getModulePositions()
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] moduleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
    }
    
    public void resetGyro() {
        gyro.reset();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}