// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.HopperRollersFeedCommand;
import frc.robot.commands.HopperRollersStillCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.IntakeRollersFeedCommand;
import frc.robot.commands.IntakeRollersIntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.IntakeDeploySubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandXboxController driverController = new CommandXboxController(1);
  final CommandXboxController operatorController = new CommandXboxController(0);
  
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                            "swerve"));
                                                                                
  private final IntakeDeploySubsystem intakeDeploy = new IntakeDeploySubsystem();
  private final IntakeRollersSubsystem intakeRollers = new IntakeRollersSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Intake Commands
  Command intakeRollersIntake = new IntakeRollersIntakeCommand(intakeRollers, 0.5);
  Command intakeRollersReverse = new IntakeRollersIntakeCommand(intakeRollers, -0.5);
  Command intakeRollersFeed = new IntakeRollersFeedCommand(intakeRollers, 0.5); // was .5, probably don't need fast speed?
  Command intakeRollersMidPass = new IntakeRollersFeedCommand(intakeRollers, 0.5); // was .5, probably don't need fast speed?
  Command intakeRollersFarPass = new IntakeRollersFeedCommand(intakeRollers, 0.5); // was .5, probably don't need fast speed?
  Command intakeRollersStill = new IntakeRollersIntakeCommand(intakeRollers, 0.0);
  Command intakeExtend = new IntakeExtendCommand(intakeDeploy);
  Command intakeRetract = new IntakeRetractCommand(intakeDeploy);
  // Command intakeLaunchRetract = new IntakeRetractCommand(intakeDeploy); 
  // Command intakePassRetract = new IntakeRetractCommand(intakeDeploy); 
  // Command intakeLaunchRetractDelay = new WaitCommand(0.5);
  // Command intakePassRetractDelay = new WaitCommand(0.5);
  Command intakeRollersIntakeAuto = new IntakeRollersIntakeCommand(intakeRollers, 0.5);
  Command intakeRollersFeedAuto = new IntakeRollersIntakeCommand(intakeRollers, 0.5);
  Command intakeRollersStillAuto = new IntakeRollersIntakeCommand(intakeRollers, 0.0);
  Command intakeExtendAuto = new IntakeExtendCommand(intakeDeploy);
  Command intakeRetractAuto = new IntakeRetractCommand(intakeDeploy);

  // Hopper Commands
  Command hopperFeed = new HopperRollersFeedCommand(hopper, 0.6);
  Command hopperMidPass = new HopperRollersFeedCommand(hopper, 0.6);
  Command hopperFarPass = new HopperRollersFeedCommand(hopper, 0.6);
  Command hopperStill = new HopperRollersStillCommand(hopper);
  Command hopperFeedAuto = new HopperRollersFeedCommand(hopper, 0.5);
  Command hopperStillAuto = new HopperRollersStillCommand(hopper);

  // Feeder Commands
  Command feederFeed = new FeederCommand(feeder,0.6);
  Command feederMidPass = new FeederCommand(feeder, 0.6); // passing related, if we need
  Command feederFarPass = new FeederCommand(feeder, 0.6); // passing related, if we need
  Command feederStill = new FeederCommand(feeder, 0.0);
  Command feedDelay = new WaitCommand(0.2); // .2 seems pretty good? Maybe less delay is possible?
  Command midPassDelay = new WaitCommand(0.2); // .2 seems pretty good? Maybe less delay is possible?
  Command farPassDelay = new WaitCommand(0.2); // .2 seems pretty good? Maybe less delay is possible?
  Command feederFeedAuto = new FeederCommand(feeder, 0.6);
  Command feederStillAuto = new FeederCommand(feeder, 0.0);

  // Launcher Commands
  Command launcherLaunch = new LauncherCommand(launcher, 0.48);
  Command launcherMidPass = new LauncherCommand(launcher, 0.7); // Maybe we can see what a higher velocity shot looks like for passing?
  Command launcherFarPass = new LauncherCommand(launcher, 0.9); // Maybe we can see what a higher velocity shot looks like for passing?
  Command launcherStill = new LauncherCommand(launcher, 0.0);
  Command launcherLaunchAuto = new LauncherCommand(launcher, 0.5);
  Command launcherStillAuto = new LauncherCommand(launcher, 0.0);

  //Swerve Commands

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY(),
                                                                () -> driverController.getLeftX())
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                           .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveWithHeadingSnaps = new AbsoluteDriveAdv(drivebase,
                                              () -> -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                          OperatorConstants.DEADBAND),
                                              () -> -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                          OperatorConstants.DEADBAND),
                                              () -> MathUtil.applyDeadband(driverController.getRightX(),
                                                                          OperatorConstants.RIGHT_X_DEADBAND),
                                              driverController.getHID()::getYButtonPressed,
                                              driverController.getHID()::getAButtonPressed,
                                              driverController.getHID()::getXButtonPressed,
                                              driverController.getHID()::getBButtonPressed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // PathPlanner Commands
    NamedCommands.registerCommand("intakeDeployAuto", intakeExtendAuto);
    NamedCommands.registerCommand("intakeRetractAuto", intakeRetractAuto);
    NamedCommands.registerCommand("intakeFeedAuto", intakeRollersFeedAuto);
    NamedCommands.registerCommand("intakeRollersIntakeAuto", intakeRollersIntakeAuto);
    NamedCommands.registerCommand("intakeRollersStillAuto", intakeRollersStillAuto);
    NamedCommands.registerCommand("feederFeedAuto", feederFeedAuto);
    NamedCommands.registerCommand("feederStillAuto", feederStillAuto);
    NamedCommands.registerCommand("hopperFeedAuto", hopperFeedAuto);
    NamedCommands.registerCommand("hopperStillAuto", hopperStillAuto);
    NamedCommands.registerCommand("launcherLaunchAuto", launcherLaunchAuto);
    NamedCommands.registerCommand("launcherStillAuto", launcherStillAuto);

    // Auto chooser — must be built after NamedCommands are registered
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

     // Configure the trigger bindings
    configureBindings();

    // Set default commands for subsystems
     drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    //drivebase.setDefaultCommand(driveWithHeadingSnaps);
    intakeRollers.setDefaultCommand(intakeRollersStill);
    hopper.setDefaultCommand(hopperStill);
    feeder.setDefaultCommand(feederStill);
    launcher.setDefaultCommand(launcherStill);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver bindings
    driverController.leftBumper().onTrue(new InstantCommand(drivebase :: zeroGyro));
    driverController.rightBumper().whileTrue(drivebase.run(drivebase::lock));
    new Trigger(driverController.a()).onTrue(driveWithHeadingSnaps);

    // Operator bindings
    operatorController.rightBumper().onTrue(intakeExtend);
    operatorController.leftBumper().onTrue(intakeRetract);
    operatorController.rightTrigger().whileTrue(intakeRollersIntake);
    operatorController.leftTrigger().whileTrue(intakeRollersReverse);

    // When A is held: run launcher, and in parallel run a sequence that waits
    operatorController.a().whileTrue(launcherLaunch
                                      .alongWith(drivebase.run(drivebase::lock))
                                      .alongWith(feedDelay.andThen(feederFeed
                                                            .alongWith(hopperFeed)
                                                            .alongWith(intakeRollersFeed))));
    
    operatorController.b().whileTrue(launcherMidPass
                                      .alongWith(midPassDelay.andThen(feederMidPass
                                                            .alongWith(hopperMidPass)
                                                            .alongWith(intakeRollersMidPass))));
    operatorController.y().whileTrue(launcherFarPass
                                      .alongWith(farPassDelay.andThen(feederFarPass
                                                            .alongWith(hopperFarPass)
                                                            .alongWith(intakeRollersFarPass))));
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