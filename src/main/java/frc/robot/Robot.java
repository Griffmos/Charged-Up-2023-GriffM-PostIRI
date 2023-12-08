// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.interfaces.AutoChooser;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.interfaces.LEDInterface;
import frc.robot.util.StateHandler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public static CTREConfigs ctreConfigs = new CTREConfigs();

  private boolean intakeGood = false;

  private AutoChooser selector;

  StateHandler stateHandler = StateHandler.getInstance();
  LEDInterface ledInterface = LEDInterface.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    //CameraServer.startAutomaticCapture(0);
    //PathPlannerServer.startServer(5811);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.

    robotContainer = new RobotContainer();
    this.selector = new AutoChooser();

    ledInterface.updateLed();



  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ledInterface.updateLed();


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    intakeGood = Math
        .abs(robotContainer.intakeSubsystem.getIntakeArmPosition()
            - stateHandler.getDesiredIntakePosition().getMainAngle().getAngle()) < 0.2;
        
    stateHandler.setIntakeGood(intakeGood);

    SmartDashboard.putNumber("INTAKE ARM ERROR", Math
        .abs(robotContainer.intakeSubsystem.getIntakeArmPosition()
            - stateHandler.getDesiredIntakePosition().getMainAngle().getAngle()));

    SmartDashboard.putBoolean("INTAKE GOOD TO GO", intakeGood);


  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    stateHandler.resetAutoStates();

    autonomousCommand = robotContainer.initializeAuto(selector);
    LimelightInterface.getInstance().aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance() == Alliance.Red ? OriginPosition.kRedAllianceWallRightSide
            : OriginPosition.kBlueAllianceWallRightSide);


    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    LimelightInterface.getInstance().aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance() == Alliance.Red ? OriginPosition.kRedAllianceWallRightSide
            : OriginPosition.kBlueAllianceWallRightSide);

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    stateHandler.setWantToBeHappy(false);
    stateHandler.setStickOut(false);
  

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Rotation Override", StateHandler.getInstance().getRotationOverride());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
