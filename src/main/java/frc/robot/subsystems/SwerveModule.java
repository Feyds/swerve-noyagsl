package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public final String moduleName; // Modülün adı (Örn: "FrontLeft")
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;
    private final double angleOffset;

    // Constructor'a 'String name' eklendi
    public SwerveModule(String name, int driveId, int angleId, int cancoderId, double offset) {
        this.moduleName = name;
        this.angleOffset = offset;

        // --- Motor Kurulumları ---
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        angleMotor = new SparkMax(angleId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderId);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();

        configureDevices();
        resetToAbsolute(); 
    }

    private void configureDevices() {
        // --- Drive Motor Config ---
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_POS_FACTOR);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_VEL_FACTOR);
        driveConfig.closedLoop.pid(SwerveConstants.DRIVE_P, SwerveConstants.DRIVE_I, SwerveConstants.DRIVE_D);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        driveConfig.inverted(false); 
        driveConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Angle Motor Config ---
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.encoder.positionConversionFactor(SwerveConstants.ANGLE_POS_FACTOR);
        angleConfig.inverted(true); // MK4i Angle motoru terstir
        angleConfig.closedLoop.pid(SwerveConstants.ANGLE_P, SwerveConstants.ANGLE_I, SwerveConstants.ANGLE_D);
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
        angleConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteEncoderRad();
        angleEncoder.setPosition(absolutePosition);
    }

    // Offset uygulanmış açı (Robotun kullandığı)
    private double getAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double angleRad = Units.rotationsToRadians(rotations);
        return angleRad - angleOffset;
    }

    // *** OFFSET AYARI İÇİN BU METOD ÖNEMLİ ***
    // Hiçbir offset çıkarmadan ham değeri verir. Constants.java'ya yazacağın değer bu.
    public double getRawAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return Units.rotationsToRadians(rotations);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(angleEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(angleEncoder.getPosition());
        
        // Optimize işlemi (WPILib 2025 - void dönüşlü)
        desiredState.optimize(currentRotation);

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        drivePID.setReference(desiredState.speedMetersPerSecond, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
        anglePID.setReference(desiredState.angle.getRadians(), com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    // --- Dashboard'a Veri Basma Fonksiyonu ---
    public void updateTelemetry() {
        // Offset bulmak için bunu kullanacaksın:
        SmartDashboard.putNumber(moduleName + " RAW Encoder (Rad)", getRawAbsoluteEncoderRad());
        
        // Diğer veriler
        SmartDashboard.putNumber(moduleName + " Integrated Angle", angleEncoder.getPosition());
        SmartDashboard.putNumber(moduleName + " Velocity", driveEncoder.getVelocity());
    }
}