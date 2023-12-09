// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockToGrid extends CommandBase {
    private SwerveSubsystem s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier fieldSlowMode;

    private PIDController pid;

    private double angleGoal;

    public LockToGrid(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup, BooleanSupplier fieldSlowMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
        this.fieldSlowMode = fieldSlowMode;

        angleGoal = 180;//always will be 180, grid behind us




        //p is 0.5/180
        pid = new PIDController(0.0028, 0, 0);

        pid.enableContinuousInput(-angleGoal, angleGoal );


    }

    @Override
    public void initialize(){
      StateHandler.getInstance().setRotationOverride(true);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */


        //double minDiff = (s_Swerve.getYaw().getDegrees()>0) ? angleGoal : -angleGoal;




        double rotationPercent = pid.calculate(s_Swerve.getYaw().getDegrees(), angleGoal);

        rotationPercent = (Math.abs(rotationPercent)>0.0001) ? rotationPercent : 0;

        SmartDashboard.putNumber("PID OUTPUT", rotationPercent);

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = 0.3* MathUtil.applyDeadband(rotationPercent, Constants.stickDeadband);

        if (robotCentricSup.getAsBoolean()) {
            translationVal *= 0.6;
            strafeVal *= 0.6;
            rotationVal *= 0.3;
        }
        if (fieldSlowMode.getAsBoolean()) {
            translationVal *= 0.6;
            strafeVal *= 0.6;
            rotationVal *= 0.3;
        }

        /* Drive */
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
    }

    @Override
    public void end(boolean interrupted) {
      StateHandler.getInstance().setRotationOverride(false);
    }

    @Override
    public boolean isFinished(){
      return false;
    }
}