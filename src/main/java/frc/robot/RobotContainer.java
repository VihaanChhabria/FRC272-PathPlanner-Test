package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopSwerve;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private boolean isRobotCentric = false;

    private final JoystickButton DriverStart = new JoystickButton(
        driver,
        XboxController.Button.kStart.value
    );

    private final JoystickButton DriverY = new JoystickButton(
        driver,
        XboxController.Button.kY.value
    );
    
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve(); 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis) * .85,
                () -> driver.getRawAxis(strafeAxis) * .85, 
                () -> driver.getRawAxis(rotationAxis) * .45, 
                () -> isRobotCentric
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        
        // Auto Configurations

        // NamedCommands.registerCommand("lorem", new ipsum);
        //SmartDashboard("ExampleAuto", new PathPlannerAuto("ExampleAuto"));
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("ExampleAuto", new PathPlannerAuto("ExampleAuto"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        DriverStart.onTrue(
            new InstantCommand(() -> isRobotCentric = !isRobotCentric)
        );

        DriverY.onTrue(new InstantCommand(() -> this.s_Swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
