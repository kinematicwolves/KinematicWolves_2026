package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.SwerveProfile;
import frc.robot.commands.AimAndShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot content is declared. 
 * It connects subsystems, controllers, and commands.
 */
public class RobotContainer {

    /* --- SUBSYSTEMS --- */
    private final Swerve m_swerve = new Swerve(
        TunerConstants.DrivetrainConstants, 
        TunerConstants.FrontLeft, TunerConstants.FrontRight, 
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final Vision m_vision = new Vision(m_swerve);
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Launcher m_launcher = new Launcher();
    private final Lighting m_lighting = new Lighting(); 
    // private final Climber m_climber = new Climber();

    /* --- CONTROLLERS --- */
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);

    /* --- AUTONOMOUS CHOOSER --- */
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        registerPathPlannerCommands(); 
        configureDefaultCommands();    
        configureBindings();           

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        // --- ADDED: BACKGROUND MATCH TIMER ---
        // Schedules a command that runs forever in the background, updating the dashboard.
        Commands.run(() -> {
            SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        }).ignoringDisable(true).withName("MatchTimerTelemetry").schedule();
    }

    /** Maps PathPlanner strings to Java commands for autonomous routines. */
    private void registerPathPlannerCommands() {
        NamedCommands.registerCommand("DeployIntake", m_intake.setPivotCommand(IntakeProfile.kPivotDownPosition));
        NamedCommands.registerCommand("RunRollers", m_intake.runRollersCommand(IntakeProfile.kRollerVoltage));
        NamedCommands.registerCommand("CloseHubShot", Launcher.closeShotCommand(m_launcher, m_indexer, m_intake));
        NamedCommands.registerCommand("AutoShoot", AimAndShoot.autoAimAndShoot(m_swerve, m_vision, m_launcher, m_indexer, m_intake));
    }

    /** Default commands run continuously when no other command is scheduled. */
    private void configureDefaultCommands() {
        // DRIVER: Joysticks drive the robot (Negatives corrected for field orientation)
        m_swerve.setDefaultCommand(m_swerve.applyDrive(
            () -> -m_driver.getLeftY() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
            () -> -m_driver.getLeftX() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
            () -> -m_driver.getRightX() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() 
        ));

        // m_climber.setDefaultCommand(m_climber.manualOverrideCommand(
        //     () -> Math.abs(m_operator.getRightY()) > 0.1 ? -m_operator.getRightY() * 12.0 : 0.0
        // ));
    }

    /** Wires the Xbox controller buttons to our command factories. */
    private void configureBindings() {

        /* --- DRIVER CONTROLS --- */
        m_driver.a().whileTrue(m_swerve.applyBrake());
        m_driver.y().onTrue(m_swerve.resetHeading()); 
        m_driver.b().whileTrue(m_intake.exhaustCommand());
        m_driver.leftTrigger().whileTrue(m_intake.deploySequenceCommand());
        //m_driver.leftBumper().onTrue(m_intake.setPivotCommand(IntakeProfile.kPivotUpPosition));
        //m_driver.x().whileTrue(GoToTower.autoClimbCommand(m_swerve)); 
        m_driver.rightTrigger().whileTrue(
            AimAndShoot.teleopAimAndShoot(
                m_swerve, m_vision, m_launcher, m_indexer, m_intake, m_driver.getHID(),
                () -> -m_driver.getLeftY() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(), 
                () -> -m_driver.getLeftX() * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() 
            )
        );


        /* --- OPERATOR CONTROLS --- */
        m_operator.a().whileTrue(Launcher.closeShotCommand(m_launcher, m_indexer, m_intake));
        // m_operator.rightTrigger().onTrue(m_climber.extendCommand());
        // m_operator.leftTrigger().onTrue(m_climber.climbCommand());
        m_operator.leftBumper().whileTrue(
            m_intake.exhaustCommand().alongWith(m_indexer.reverseIndexerCommand())
            );
        
        m_operator.rightBumper().whileTrue(new InstantCommand(() -> m_intake.setRollerVoltage(IntakeProfile.kRollerVoltage)));
        m_operator.b()
            .onTrue(m_intake.setPivotCommand(IntakeProfile.kPivotUpPosition))
            .onFalse(m_intake.setPivotCommand(IntakeProfile.kPivotDownPosition));

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}