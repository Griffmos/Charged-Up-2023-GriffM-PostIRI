// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeCommands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.VerticalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoHalfConeBalanceNonCableProtector extends SequentialCommandGroup {
  /** Creates a new TwoHalfConeBalanceNonCableProtector. */
  public TwoHalfConeBalanceNonCableProtector(SwerveSubsystem swerve, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    StateHandler stateHandler = StateHandler.getInstance();

    final AutoFromPathPlanner acquireCone = new AutoFromPathPlanner(swerve, "AcquireConeNOCP", 2.5, 2, false, true, true);
    final AutoFromPathPlanner scoreCone = new AutoFromPathPlanner(swerve, "ScoreConeNOCP", 2.5, 2, false, true, true);
    final AutoFromPathPlanner acquireSecondCone = new AutoFromPathPlanner(swerve, "AcquireSecondConeNOCP", 2.5, 2, false, true, true);
    final AutoFromPathPlanner balanceAfterAcquire = new AutoFromPathPlanner(swerve, "BalanceAfterAcquireNOCP", 2.5, 2, false, true, true);
    
    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(acquireCone.getInitialPose())),
      new InstantCommand(() -> swerve.zeroGyro(acquireCone.getInitialPose().getRotation().getDegrees())),

      /*
       * The first line sets the desiredArmPosition.
       * When the arm is in position, the manipulator disengages.
       * When this is done, we then go back to STOW
       */
      new AutoScoreCommand(HorizontalLocations.RIGHT, VerticalLocations.HIGH, GamePieceMode.CONE),
      new ParallelCommandGroup(
        new InstantCommand(() -> stateHandler.setAutoRunIntake(true)), //set intake wheel speeds
        /* The sequential command group below will first 
         * wait for the arm to be in stow,
         * then the intake will deploy
         */
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> stateHandler.getCurrentArmPosition() == ArmPositions.STOW),
          new DeployIntakeCommand(intake)
        ),
        acquireCone
      ),
      //assuming cone has been intaked, no need to run the wheels anymore
      new InstantCommand(() -> stateHandler.setAutoRunIntake(false)),

      /*
       * Inside this paralle group, we drive back & STOW the intake
       */
      new ParallelCommandGroup(
        new StowIntakeCommand(intake, () -> true),
        scoreCone
      ),

      /*
       * Now, we're at the scoring location. 
       * The default commands should've already taken care of holding the game piece. 
       * We just need to tell the arm that we're ready to score
       */
      new InstantCommand(() -> stateHandler.setWantToScore(true)),

      new AutoScoreCommand(HorizontalLocations.LEFT, VerticalLocations.HIGH, GamePieceMode.CONE),


      new ParallelCommandGroup(
        new InstantCommand(() -> stateHandler.setAutoRunIntake(true)), //set intake wheel speeds

        new SequentialCommandGroup(
          new WaitUntilCommand(() -> stateHandler.getCurrentArmPosition() == ArmPositions.STOW),
          new DeployIntakeCommand(intake)
        ),

        acquireSecondCone
      ),

      new InstantCommand(() -> stateHandler.setAutoRunIntake(false)),
      

      new ParallelCommandGroup(
        new StowIntakeCommand(intake, () -> true),
        balanceAfterAcquire
      )

    );
  }
}
