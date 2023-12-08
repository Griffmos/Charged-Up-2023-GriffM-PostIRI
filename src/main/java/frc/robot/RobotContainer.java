package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.AutoShootSequence;
import frc.robot.commands.Autos.TestMarkers;
import frc.robot.commands.IntakeCommands.EjectCommand;
import frc.robot.commands.IntakeCommands.EmergencyResetCommand;
import frc.robot.commands.IntakeCommands.FerryShootGroup;
import frc.robot.commands.IntakeCommands.IntakeGamePiece;
import frc.robot.commands.IntakeCommands.ShootGamePiece;
import frc.robot.commands.IntakeCommands.TestFIre;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.StateCommands.SetShootingLocation;
import frc.robot.commands.StateCommands.ShootFront;
import frc.robot.commands.SwerveCommands.AlignToTag;
import frc.robot.commands.SwerveCommands.LockToGrid;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.commands.SwerveCommands.TeleopSwerve;
import frc.robot.commands.SwerveCommands.TrajectoryToGoal;
import frc.robot.interfaces.AutoChooser;
import frc.robot.subsystems.*;
import frc.robot.util.AutonInstantiate;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;
import frc.robot.util.StateVariables.VerticalLocations;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    //need this for rumble setup, DO NOT USE
    private final XboxController xboxDriverController = new XboxController(0);
    private final PS4Controller ps4OperatorController = new PS4Controller(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);

    //GRIFFM Addtion:
    private final JoystickButton rSJoystickButton = new JoystickButton(driver, 12); //FIND THIS

    /* Operator Buttons */
    private final JoystickButton operatorSquareButton = new JoystickButton(operator, 3);
    private final JoystickButton operatorTriangleButton = new JoystickButton(operator, 4); //4
    private final JoystickButton operatorCircleButton = new JoystickButton(operator, 2);
    private final JoystickButton operatorCrossButton = new JoystickButton(operator,1);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operator, 5); //5
    private final JoystickButton operatorRightBumper = new JoystickButton(operator, 6); //6
    private final POVButton operatorUpDPad = new POVButton(operator, 0);
    private final POVButton operatorRightDPad = new POVButton(operator, 90);
    private final POVButton operatorDownDPad = new POVButton(operator, 180);
    private final POVButton operatorLeftDPad = new POVButton(operator, 270);
    private final JoystickButton centerRightButton = new JoystickButton(operator, 8);
    private final JoystickButton centerLeftButton = new JoystickButton(operator, 7);


    /* Subsystems */
    public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    //public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(() -> operator.getRawAxis(2), () -> operator.getRawAxis(3));
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(() -> operator.getRawAxis(5), () -> driver.getRawAxis(3));//Up is negative, so sign change


    public final ControllerRumble controllerRumble = new ControllerRumble(xboxDriverController, ps4OperatorController);
    public final ShuffleboardSubsystem shuffleboardSubsystem = new ShuffleboardSubsystem();


    //AutonInstantiate
    AutonInstantiate autonInstantiate = AutonInstantiate.getInstance(s_Swerve);



    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        setDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //DRIVER CONTROLLER BINDINGS
        yButton.toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        aButton.toggleOnTrue(new SwerveXWheels(s_Swerve));
        bButton.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        new Trigger(() -> driver.getRawAxis(3) > 0.2).whileTrue(new IntakeGamePiece(IntakeWheelSpeeds.INTAKE));
        // new Trigger(() -> driver.getRawAxis(2) > 0.2).whileTrue(new AlignToTag(s_Swerve,() -> -driver.getRawAxis(translationAxis)));
        //new Trigger(() -> driver.getRawAxis(2) > 0.2).whileTrue(new TrajectoryToGoal(s_Swerve));

        //GRIFFM ADDITION

        rSJoystickButton.toggleOnTrue(
            new LockToGrid(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> leftBumper.getAsBoolean(),
                        () -> rightBumper.getAsBoolean()));


        /*
         * Note for Aalind for button bindings:
         * - right trigger = intake
         * - right bumper = slow mode = use for charge station
         * - a = lock swerve wheels 
         * - b = reset swerve modules
         * - y = pigeon reset
         */


        //OPERATOR CONTROLLER BINDINGS
        operatorUpDPad.onTrue(new SetShootingLocation(VerticalLocations.HIGH));
        operatorDownDPad.onTrue(new SetShootingLocation(VerticalLocations.LOW));
        operatorLeftDPad.onTrue(new SetShootingLocation(VerticalLocations.MID));
        operatorCrossButton.whileTrue(new ShootGamePiece(s_Swerve));
        centerRightButton.whileTrue(EmergencyResetCommand.getInstance(intakeSubsystem));

        operatorSquareButton.onTrue(new SetIntakePosition(IntakePositions.INTAKE));
        operatorTriangleButton.onTrue(new SetIntakePosition(IntakePositions.SHOOT_TALL));
        operatorCircleButton.onTrue(new ShootFront());
        
        operatorRightBumper.onTrue(new FerryShootGroup());
 
        operatorLeftBumper.whileTrue(new EjectCommand());
  
    } 

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> leftBumper.getAsBoolean(),
                        () -> rightBumper.getAsBoolean()));

    }

    public Command initializeAuto(AutoChooser selector){
        return selector.startMode(s_Swerve, intakeSubsystem);
    }
}