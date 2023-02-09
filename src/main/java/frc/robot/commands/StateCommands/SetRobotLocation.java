// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.HorizontalLocations;

public class SetRobotLocation extends CommandBase {
  /** Creates a new SetArmLocation. */
  BooleanSupplier triangle;
  BooleanSupplier circle;
  BooleanSupplier cross;
  BooleanSupplier square;
  StateHandler stateHandler = StateHandler.getInstance();
  public SetRobotLocation(BooleanSupplier t, BooleanSupplier circle, BooleanSupplier cross, BooleanSupplier square) {
    this.triangle = t;
    this.circle = circle;
    this.cross = cross;
    this.square = square;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (triangle.getAsBoolean()) {
      stateHandler.setHorizontalLocation(HorizontalLocations.LEFT);
    } else if (circle.getAsBoolean()) {
      stateHandler.setHorizontalLocation(HorizontalLocations.CENTER);
    } else if (cross.getAsBoolean()) {
      stateHandler.setHorizontalLocation(HorizontalLocations.RIGHT);
    } else if(square.getAsBoolean()) {
      stateHandler.setHorizontalLocation(HorizontalLocations.RESET);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
