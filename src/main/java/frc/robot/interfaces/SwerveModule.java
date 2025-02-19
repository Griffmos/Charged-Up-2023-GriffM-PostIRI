package frc.robot.interfaces;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;
import frc.robot.util.CTREModuleState;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private double currentOptimizedAngle = 0;
    private double stateAngle = 0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Swerve.driveKS, Swerve.driveKV, Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID,"rio");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "Default Name");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Default Name");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        updateCurrentOptimizedAngle(desiredState.angle.getDegrees());
        stateAngle = getState().angle.getDegrees();
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Swerve.wheelCircumference,
                    Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01)) ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
                Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Swerve.wheelCircumference,
                        Swerve.driveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Swerve.wheelCircumference,
                        Swerve.driveGearRatio),
                getAngle());
    }

    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0);
        mAngleMotor.set(ControlMode.PercentOutput, 0);
    }

    public void updateCurrentOptimizedAngle(double angle){
        currentOptimizedAngle = angle;
    }

    public double getOptimizedAngle(){
        return currentOptimizedAngle;
    }

    public double getStateAngle(){
        return stateAngle;
    }


    @Override
    public String toString(){
        return "OPTIMIZED ANGLE"+(int)currentOptimizedAngle;
    }
}