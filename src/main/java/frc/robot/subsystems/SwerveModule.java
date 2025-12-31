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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public final String moduleName;
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;
    private final double angleOffset;

    private double simDrivePosition = 0.0; // Sanal gidilen yol (metre)
    private double simAngleRad = 0.0;      // Sanal açı (radyan)
    private SwerveModuleState lastDesiredState = new SwerveModuleState(); // Son istenen hız/açı

    public SwerveModule(String name, int driveId, int angleId, int cancoderId, double offset) {
        this.moduleName = name;
        this.angleOffset = offset;

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
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_POS_FACTOR);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_VEL_FACTOR);
        driveConfig.closedLoop.pid(SwerveConstants.DRIVE_P, SwerveConstants.DRIVE_I, SwerveConstants.DRIVE_D);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        driveConfig.inverted(false); 
        driveConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.encoder.positionConversionFactor(SwerveConstants.ANGLE_POS_FACTOR);
        angleConfig.inverted(true);
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

    private double getAbsoluteEncoderRad() {
        if (RobotBase.isSimulation()) {
            return simAngleRad;
        }
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double angleRad = Units.rotationsToRadians(rotations);
        return angleRad - angleOffset;
    }

    public double getRawAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return Units.rotationsToRadians(rotations);
    }

    public SwerveModuleState getState() {
        if (RobotBase.isSimulation()) {
            return new SwerveModuleState(lastDesiredState.speedMetersPerSecond, new Rotation2d(simAngleRad));
        }
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(angleEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        // Eğer simülasyondaysak sanal veriyi dön
        if (RobotBase.isSimulation()) {
            return new SwerveModulePosition(simDrivePosition, new Rotation2d(simAngleRad));
        }
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(angleEncoder.getPosition());

        // Simülasyon için de optimize etmeliyiz ki tekerlek 180 derece ters dönmesin
        if(RobotBase.isSimulation()){
            currentRotation = new Rotation2d(simAngleRad);
       }
        
        desiredState.optimize(currentRotation);
        // Simülasyon döngüsünde kullanmak için bu değeri saklıyoruz
        this.lastDesiredState = desiredState;

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

    public void simulationPeriodic(double dt) {
        // x = v * t formülü
        // Hız (m/s) * geçen süre (0.02s) = Gidilen Mesafe
        simDrivePosition += lastDesiredState.speedMetersPerSecond * dt;
        
        // Açıyı direkt eşitliyoruz (Sanki motor anında dönmüş gibi kabul ediyoruz, PID simüle etmedik)
        simAngleRad = lastDesiredState.angle.getRadians();
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber(moduleName + " RAW Encoder (Rad)", getRawAbsoluteEncoderRad());
        SmartDashboard.putNumber(moduleName + " Integrated Angle", angleEncoder.getPosition());
        SmartDashboard.putNumber(moduleName + " Velocity", driveEncoder.getVelocity());
    }
}