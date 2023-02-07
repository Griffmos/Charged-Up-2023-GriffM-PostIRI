package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.interfaces.BetterLimelightInterface.SpecificLimelight;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.ScoringLocations;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private ScoringLocations scoringLocations = new ScoringLocations(VerticalLocations.HIGH, HorizontalLocations.LEFT);

    private ArmPositions desiredArmPosition = ArmPositions.STOW;
    private ArmPositions currentArmPosition = ArmPositions.STOW;

    private IntakePositions desiredIntakePosition = IntakePositions.STOW;
    private IntakePositions currentIntakePosition = IntakePositions.STOW;

    private boolean intakeInPosition = false,
            hasGamePiece = false,
            armInPosition = false,
            resetManipulator = false,
            gripperEngaged = false,
            wantToScore = false,
            holdInCobra = false;


    private GamePieceMode mode = GamePieceMode.CUBE;

    private CurrentRobotDirection currentRobotDirection = CurrentRobotDirection.RIGHT;

    private SpecificLimelight currentLimelight = SpecificLimelight.LEFT_LIMELIGHT;

    private double timeSinceLastGripChange = System.currentTimeMillis();

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }
        return stateHandler;
    }

    public ScoringLocations getCurrentScoringLocation() {
        return scoringLocations;
    }

    public void setScoringLocation(VerticalLocations v, HorizontalLocations h) {
        scoringLocations.setVerticalLocation(v);
        scoringLocations.setHorizontalLocation(h);
    }

    public void setArmDesiredState(ArmPositions a) {
        if (a != desiredArmPosition) {
            armInPosition = false;
        }
        desiredArmPosition = a;
    }

    public ArmPositions getArmDesiredPosition() {
        return desiredArmPosition;
    }

    public void setIntakeInPosition(boolean intakeInPosition) {
        this.intakeInPosition = intakeInPosition;
    }

    public void setArmInPosition(boolean armInPosition) {
        this.armInPosition = armInPosition;
    }

    public void setHasGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    public void setManipulator(boolean resetManipulator) {
        this.resetManipulator = resetManipulator;
    }

    public boolean getIntakeInPosition() {
        return intakeInPosition;
    }

    public boolean getArmInPosition() {
        return armInPosition;
    }

    public boolean getHasGamePiece() {
        return hasGamePiece;
    }

    public boolean readyToClose() {
        return intakeInPosition && armInPosition && currentArmPosition == ArmPositions.STOW
                && (currentIntakePosition == IntakePositions.CONE_HANDOFF
                        || currentIntakePosition == IntakePositions.CUBE_HANDOFF);
    }

    public boolean getResetManipulator() {
        return resetManipulator;
    }

    public CurrentRobotDirection getRobotDirection() {
        return currentRobotDirection;
    }

    public void setRobotDirection(CurrentRobotDirection c) {
        currentRobotDirection = c;
    }

    public ArmPositions getCurrentArmPosition() {
        return currentArmPosition;
    }

    public void setCurrentArmPosition(ArmPositions a) {
        currentArmPosition = a;
    }

    public IntakePositions getCurrentIntakePosition() {
        return currentIntakePosition;
    }

    public void setCurrentIntakePosition(IntakePositions i) {
        currentIntakePosition = i;
    }

    public IntakePositions getDesiredIntakePosition() {
        return desiredIntakePosition;
    }

    public void setDesiredIntakePosition(IntakePositions i) {
        if (i != desiredIntakePosition) {
            intakeInPosition = false;
        }
        desiredIntakePosition = i;
    }

    public GamePieceMode getGamePieceMode() {
        return mode;
    }

    public void setGamePieceMode(GamePieceMode g) {
        mode = g;
    }

    public void setGripperEngaged(boolean g) {
        gripperEngaged = g;
    }

    public boolean getGripperEngaged() {
        return gripperEngaged;
    }

    public SpecificLimelight getSpecificLimelight() {
        if (currentRobotDirection == CurrentRobotDirection.LEFT) {
            return SpecificLimelight.LEFT_LIMELIGHT;
        } else {
            return SpecificLimelight.RIGHT_LIMELIGHT;
        }
    }

    public void setTimeSinceLastGripChange() {
        timeSinceLastGripChange = System.currentTimeMillis();
    }

    // Time in seconds since the gripper was changed
    public double getTimeSinceLastGripChange() {
        return (System.currentTimeMillis() - timeSinceLastGripChange)/1000;
    }

    public void setHoldInCobra(boolean set) {
        holdInCobra = set;
    }

    public boolean getHoldInCobra() {
        return holdInCobra;
    }

    public void setWantToScore(boolean set) {
        wantToScore = set;
    }

    public boolean getWantToScore() {
        return wantToScore;
    }

    public ArmPositions getArmPositionFromScoringLocation() {
        return ArmPositions.CONE_HIGH;
    }
}
