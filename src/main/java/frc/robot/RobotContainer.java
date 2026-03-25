package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.SwerveProfile;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // --- SUBSYSTEMS ---
    private final Swerve m_swerve = new Swerve(
        TunerConstants.DrivetrainConstants, 
        TunerConstants.FrontLeft, TunerConstants.FrontRight, 
        TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final VisionSubsystem m_vision = new VisionSubsystem(m_swerve);
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Launcher m_launcher = new Launcher();
    private final Climber m_climber = new Climber();

    // --- CONTROLLERS ---
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);

    // --- AUTONOMOUS CHOOSER ---
    private final SendableChooser<Command> m_autoChooser;

    // A PID Controller to smoothly rotate the chassis to the target
    private final PIDController m_aimController = new PIDController(5.0, 0.0, 0.1);

    public RobotContainer() {
        // Tell the PID controller that -180 degrees and +180 degrees are the same thing
        m_aimController.enableContinuousInput(-Math.PI, Math.PI);
        m_aimController.setTolerance(Units.degreesToRadians(2.0));

        registerPathPlannerCommands();
        configureDefaultCommands();
        configureBindings();

        // Build the auto chooser from PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    /** Helper: Which Hub are we shooting at? */
    private Translation2d getTargetHub() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? FieldConstants.kRedHub : FieldConstants.kBlueHub;
    }

    /** Helper: Calculate distance from Robot Pose to the Hub */
    private double getOdometryDistanceMeters() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        return robotPos.getDistance(getTargetHub());
    }

    /** Helper: Calculate the rotational speed needed to face the Hub */
    private double getOdometryAimRate() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        Translation2d target = getTargetHub();
        
        // Calculate the angle from the robot to the target
        Rotation2d targetHeading = target.minus(robotPos).getAngle();
        
        // BACKWARDS OFFSET: Because your shooter is on the BACK, 
        // we add 180 degrees so the back faces the target, not the front!
        targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));

        // Calculate the PID output based on our current rotation vs target rotation
        return m_aimController.calculate(
            m_swerve.getPose().getRotation().getRadians(), 
            targetHeading.getRadians()
        );
    }

    /**
     * Registers named commands for use in PathPlanner's GUI.
     */
    private void registerPathPlannerCommands() {
        NamedCommands.registerCommand("DeployIntake", m_intake.setPivotCommand(IntakeProfile.kPivotDownPosition));
        NamedCommands.registerCommand("RunRollers", m_intake.runRollersCommand(IntakeProfile.kRollerVoltage));
        NamedCommands.registerCommand("CloseHubShot", createFenderShotCommand());
        NamedCommands.registerCommand("AutoShoot", createAutoShootCommand());
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
        // Applies a small deadband so stick drift doesn't run the motors
        m_climber.setDefaultCommand(m_climber.manualOverrideCommand(
            () -> Math.abs(m_operator.getRightY()) > 0.1 ? -m_operator.getRightY() * 12.0 : 0.0
        ));
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

        // X Button: Auto-Align to Climbing Tower using PathPlanner On-the-Fly
        m_driver.x().whileTrue(createAutoClimbCommand());

        /* ========================================= */
        /* OPERATOR CONTROLS                         */
        /* ========================================= */

        // LEFT TRIGGER: Intake Deploy Sequence
        // 1. Move pivot down. 2. Wait until physically down. 3. Turn on rollers.
        Command deploySequence = m_intake.setPivotCommand(IntakeProfile.kPivotDownPosition)
            .alongWith(
                Commands.waitUntil(m_intake::isIntakeDown)
                .andThen(m_intake.runRollersCommand(IntakeProfile.kRollerVoltage))
            );
        m_operator.leftTrigger().whileTrue(deploySequence);

        // RIGHT BUMPER: Intake Retract (Stop rollers, pivot up)
        m_operator.rightBumper().onTrue(m_intake.setPivotCommand(IntakeProfile.kPivotUpPosition));

        // LEFT BUMPER: Exhaust / Clear Jam (Reverse Intake + Reverse Indexer)
        m_operator.leftBumper().whileTrue(
            m_intake.exhaustCommand().alongWith(m_indexer.reverseIndexerCommand())
        );

        // RIGHT TRIGGER: Shoot (Aim & Shoot)
        m_operator.rightTrigger().whileTrue(createShootCommand());

        // B BUTTON: Fender/Home Shot Fallback
        m_operator.b().whileTrue(createFenderShotCommand());

        // CLIMBER CONTROLS (Y = Extend, A = Retract/Climb)
        m_operator.y().onTrue(m_climber.extendCommand());
        m_operator.a().onTrue(m_climber.climbCommand());
    }

    /* ========================================================= */
    /* COMPLEX COMMAND COMPOSITIONS                              */
    /* ========================================================= */

    /**
     * Auto "Shoot Anywhere" Command.
     * Uses Odometry to aim and spool, fires when ready, and safely ends in 3 seconds.
     */
    private Command createAutoShootCommand() {
        // The Firing Sequence (This is our "Deadline")
        Command fireSequence = Commands.sequence(
            // Wait until the Swerve is facing the target AND the Launcher is up to speed
            Commands.waitUntil(() -> m_aimController.atSetpoint() && m_launcher.isReadyToFire()),
            // Run the indexer for 3 seconds to shoot the ball, then move on
            m_indexer.feedShooterCommand().withTimeout(3) //TODO: Might need to adust timer
        );

        // The Background Tasks (Aiming and Spooling via Odometry)
        return Commands.deadline(
            fireSequence, 
            m_swerve.applySlowDrive(() -> 0.0, () -> 0.0, this::getOdometryAimRate), 
            m_launcher.continuousAimCommand(this::getOdometryDistanceMeters)        
        )
        // Safety Stop: Ensure everything shuts off before PathPlanner takes over
        .andThen(Commands.runOnce(() -> {
            m_indexer.stop();
            m_launcher.stop();
        }))
        .withName("AutoShootAnywhere");
    }

    /**
     * Uses Odometry (Pose2d) to aim
     */
    private Command createShootCommand() {
        return Commands.parallel(
            // Rotate using Odometry math
            m_swerve.applySlowDrive(
                () -> -m_driver.getLeftY() * SwerveProfile.kMaxSpeed,
                () -> -m_driver.getLeftX() * SwerveProfile.kMaxSpeed,
                this::getOdometryAimRate
            ),

            // Rev shooter based on Odometry distance
            m_launcher.continuousAimCommand(this::getOdometryDistanceMeters),

            // Wait for Launcher and Swerve Heading to be ready
            Commands.waitUntil(() -> m_launcher.isReadyToFire() && m_aimController.atSetpoint())
                    .andThen(m_indexer.feedShooterCommand())
        ).withName("PoseShootCommand");
    }

    /**
     * A fallback sequence for when vision fails or you are pressed against the hub
     */
    private Command createFenderShotCommand() {
        return Commands.parallel(
            m_launcher.hubShotCommand(),
            Commands.waitUntil(m_launcher::isReadyToFire)
                    .andThen(m_indexer.feedShooterCommand())
        ).withName("FallbackShotSequence");
    }

    /**
     * Generates a live PathPlanner trajectory to the correct alliance's climbing tower.
     */
    private Command createAutoClimbCommand() {
        return Commands.defer(() -> {
            // Check alliance AT THE TIME THE BUTTON IS PRESSED. Default to Blue if unknown.
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            
            // TODO: Update these exact X/Y tower coordinates
            Pose2d redTowerPose = new Pose2d(15.902, 4.059, Rotation2d.fromDegrees(180));
            Pose2d blueTowerPose = new Pose2d(1.648, 4.059, Rotation2d.fromDegrees(0));
            
            // Select target based on alliance
            Pose2d targetPose = isRed ? redTowerPose : blueTowerPose;

            PathConstraints constraints = new PathConstraints(
                3.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540)
            );

            return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
            
        }, java.util.Set.of(m_swerve));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}