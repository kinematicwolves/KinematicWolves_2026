package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.SwerveProfile;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.autoCommands.TowerTrajectory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    // --- SUBSYSTEMS ---
    private final Swerve m_swerve = new Swerve(
        TunerConstants.DrivetrainConstants, 
        TunerConstants.FrontLeft, TunerConstants.FrontRight, 
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final Vision m_vision = new Vision(m_swerve);
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Launcher m_launcher = new Launcher();
    //private final Climber m_climber = new Climber();

    // --- CONTROLLERS ---
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);
    private final CommandXboxController m_technician = new CommandXboxController(2);

    // --- AUTONOMOUS CHOOSER ---
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        registerPathPlannerCommands();
        configureDefaultCommands();
        configureBindings();

        // Build the auto chooser from PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    /**
     * Registers named commands for use in PathPlanner's GUI.
     */
    private void registerPathPlannerCommands() {
        // Basic subsystem commands
        NamedCommands.registerCommand("DeployIntake", m_intake.setPivotCommand(IntakeProfile.kPivotDownPosition));
        NamedCommands.registerCommand("RunRollers", m_intake.runRollersCommand(IntakeProfile.kRollerVoltage));
        
        // Sourced directly from our Launcher subsystem
        NamedCommands.registerCommand("CloseHubShot", Launcher.closeShotCommand(m_launcher, m_indexer));
        
        // Complex auto sequences sourced from our dedicated command classes
        NamedCommands.registerCommand("AutoShoot", AimAndShoot.autoAimAndShoot(m_swerve, m_vision, m_launcher, m_indexer));
    }

    /**
     * Sets up the default commands that run when no buttons are being pressed.
     */
    private void configureDefaultCommands() {
        // DRIVER: Standard Field-Centric Swerve Drive
        m_swerve.setDefaultCommand(m_swerve.applyDrive(
            () -> -m_driver.getLeftY() * SwerveProfile.kMaxSpeed,
            () -> -m_driver.getLeftX() * SwerveProfile.kMaxSpeed,
            () -> -m_driver.getRightX() * SwerveProfile.kMaxAngularRate
        ));

        // OPERATOR: Manual Climber Winch Override (Right Stick Y)
        // m_climber.setDefaultCommand(m_climber.manualOverrideCommand(
        //     () -> Math.abs(m_operator.getRightY()) > 0.1 ? -m_operator.getRightY() * 12.0 : 0.0
        // ));
    }

    /**
     * Wires the Xbox controller buttons to our command factories.
     */
    private void configureBindings() {

        /* ========================================= */
        /* DRIVER CONTROLS                           */
        /* ========================================= */

        // Start/Back: Reset Gyro
        m_driver.start().onTrue(m_swerve.resetHeading());

        // X Button: Auto-Align to Climbing Tower using our dedicated Pathfinding class
        m_driver.x().whileTrue(TowerTrajectory.autoClimbCommand(m_swerve));


        /* ========================================= */
        /* OPERATOR CONTROLS                         */
        /* ========================================= */

        // LEFT TRIGGER: Intake Deploy Sequence (Now encapsulated in Intake.java!)
        m_operator.leftTrigger().whileTrue(m_intake.deploySequenceCommand());

        // RIGHT BUMPER: Intake Retract (Stop rollers, pivot up)
        m_operator.rightBumper().onTrue(m_intake.setPivotCommand(IntakeProfile.kPivotUpPosition));

        // LEFT BUMPER: Exhaust / Clear Jam (Reverse Intake + Reverse Indexer)
        m_operator.leftBumper().whileTrue(
            m_intake.exhaustCommand().alongWith(m_indexer.reverseIndexerCommand())
        );

        // RIGHT TRIGGER: Shoot (Aim & Shoot) - Uses our dedicated Teleop Command!
        m_operator.rightTrigger().whileTrue(
            AimAndShoot.teleopAimAndShoot(
                m_swerve, m_vision, m_launcher, m_indexer,
                () -> -m_driver.getLeftY() * SwerveProfile.kMaxSpeed,
                () -> -m_driver.getLeftX() * SwerveProfile.kMaxSpeed
            )
        );

        // B BUTTON: Close/Home Shot Fallback
        m_operator.b().whileTrue(Launcher.closeShotCommand(m_launcher, m_indexer));

        // CLIMBER CONTROLS (Y = Extend, A = Retract/Climb)
        // m_operator.y().onTrue(m_climber.extendCommand());
        // m_operator.a().onTrue(m_climber.climbCommand());

        /* ========================================= */
        /* OPERATOR CONTROLS                         */
        /* ========================================= */

        // While the technician holds the A button, the shooter uses the dashboard values
        m_technician.a().whileTrue(m_launcher.technicianTuningCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}