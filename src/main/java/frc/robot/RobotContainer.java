package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  /* Controllers */

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Auto Configurations

    NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> SmartDashboard.putString("AutonEvent", "StopIntake")));
    NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> SmartDashboard.putString("AutonEvent", "StartIntake")));
    NamedCommands.registerCommand("ShootRing", new InstantCommand(() -> SmartDashboard.putString("AutonEvent", "ShootRing")));
    //SmartDashboard("ExampleAuto", new PathPlannerAuto("ExampleAuto"));
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption(
      "ExampleAuto",
      new InstantCommand(() -> s_Swerve.setPose(new Pose2d(2.00, 7.00, new Rotation2d(Units.degreesToRadians(0)))))
        .andThen(new PathPlannerAuto("ExampleAuto"))
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.addOption("S1_FR1_BR1_BR2", generateAuton("S1_FR1_BR1_BR2"));
    autoChooser.addOption("S2_FR2_FR1_FR1_BR1_BR2", generateAuton("S2_FR2_FR1_FR1_BR1_BR2"));
    autoChooser.addOption("S2_FR2_FR3_FR1_BR1", generateAuton("S2_FR2_FR3_FR1_BR1"));
    autoChooser.addOption("S2_FR2_FR3_FR1_BR1_BR2", generateAuton("S2_FR2_FR3_FR1_BR1_BR2"));
    autoChooser.addOption("S3", generateAuton("S3"));
    autoChooser.addOption("S3_BR5_BR4", generateAuton("S3_BR5_BR4"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command generateAuton(String autonName){
    Pose2d startingPose;
    Character startingNum = autonName.charAt(1);

    switch (startingNum) {
        case '1':
            startingPose = new Pose2d(0.66, 6.73, new Rotation2d(Units.degreesToRadians(-116.81)));
            break;

        case '2':
            startingPose = new Pose2d(1.36, 5.54, new Rotation2d(Units.degreesToRadians(179.60)));
            break;

        case '3':
            startingPose = new Pose2d(0.67, 4.39, new Rotation2d(Units.degreesToRadians(120.07)));
            break;
    
        default:
            startingPose = new Pose2d();
            break;
    }

    return new InstantCommand(() -> s_Swerve.setPose(startingPose))
        .andThen(new PathPlannerAuto("ExampleAuto"));
  }
}
