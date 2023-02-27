// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.IntakePositions;

public class DeployIntakeCommand extends CommandBase {
  /** Creates a new DeployIntakeCommand. */
  StateHandler stateHandler = StateHandler.getInstance();

  public DeployIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakePositions currentIntakePosition = stateHandler.getCurrentIntakePosition();

    SmartDashboard.putString("DEBUG 1", currentIntakePosition.toString());
    SmartDashboard.putString("DEBUG 2", stateHandler.getGamePieceMode().toString());

    if ((currentIntakePosition == IntakePositions.STOW || currentIntakePosition == IntakePositions.FINAL_HANDOFF)
        && (stateHandler.getGamePieceMode() == GamePieceMode.CUBE || (!stateHandler.getGripperEngaged() && stateHandler.getGamePieceMode() == GamePieceMode.CONE))) {
      SmartDashboard.putBoolean("Deployed Intake", true);
      stateHandler.setDesiredIntakePosition(IntakePositions.REVERSE_HANDOFF_1);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}