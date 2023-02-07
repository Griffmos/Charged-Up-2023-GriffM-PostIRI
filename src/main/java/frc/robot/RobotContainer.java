package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands.OldArmDefaultCommand;
import frc.robot.commands.ArmCommands.ArmDefaultCommand;
import frc.robot.commands.ArmCommands.ArmToPosition;
import frc.robot.commands.IntakeCommands.IntakeAndLiftCommand;
import frc.robot.commands.Scoring.ManipulatorDefaultCommand;
import frc.robot.commands.Scoring.ManualScore;
import frc.robot.commands.SwerveCommands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.StateVariables.ArmPositions;

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
    private final Joystick test = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton squareButton = new JoystickButton(operator, 3);
    private final JoystickButton triangleButton = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
    private final JoystickButton circleButton = new JoystickButton(operator, 2);
    private final JoystickButton crossButton = new JoystickButton(operator,1);

    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton testAButton = new JoystickButton(test, 2);
    private final JoystickButton testBButton = new JoystickButton(test, 1);
    private final JoystickButton testXButton = new JoystickButton(test, XboxController.Button.kX.value);
    private final JoystickButton testYButton = new JoystickButton(test, 3);

    /* Operator Buttons */
    private final JoystickButton operatorXButton = new JoystickButton(operator, PS4Controller.Button.kCross.value);
    private final JoystickButton operatorSquareButton = new JoystickButton(operator,
            PS4Controller.Button.kSquare.value);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ManipulatorSubsystem gripper = new ManipulatorSubsystem();

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        aButton.onTrue(new ArmToPosition(ArmPositions.STOW));
        bButton.onTrue(new ArmToPosition(ArmPositions.COBRA_FORWARD));
        xButton.onTrue(new ArmToPosition(ArmPositions.CONE_HIGH));
        yButton.onTrue(new ArmToPosition(ArmPositions.CUBE_MID));

        testYButton.toggleOnTrue(new ManualScore());

    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        intakeSubsystem.setDefaultCommand(new IntakeAndLiftCommand(intakeSubsystem, () -> triangleButton.getAsBoolean(),
        () -> driver.getRawAxis(3),
        () -> squareButton.getAsBoolean(),
        () -> crossButton.getAsBoolean()));

        
        armSubsystem.setDefaultCommand(new ArmDefaultCommand(armSubsystem));

        gripper.setDefaultCommand(new ManipulatorDefaultCommand(gripper, () -> testAButton.getAsBoolean(), () -> testBButton.getAsBoolean()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
