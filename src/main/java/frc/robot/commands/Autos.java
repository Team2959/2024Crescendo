// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocation;

public final class Autos
{
    public enum autoStartPosition
    {
        Center,
        RedAmp,
        BlueAmp,
        RedSource,
        BlueSource,
        Nowhere,
    };
    
    public static SendableChooser<Command> sendableChooserFullPathPlanner(RobotContainer container)
    {
        SendableChooser<Command> sendableChooser = AutoBuilder.buildAutoChooser();
        // SendableChooser<Command> sendableChooser = AutoBuilder.buildAutoChooser("Center Two Note");
        return sendableChooser;
    }

    public static SendableChooser<autoStartPosition> sendableChooserAngleStart(RobotContainer container)
    {
        SendableChooser<autoStartPosition> sendableChooser = new SendableChooser<>();
        sendableChooser.setDefaultOption("Center", autoStartPosition.Center);
        sendableChooser.addOption("RedAmpSide", autoStartPosition.RedAmp);
        sendableChooser.addOption("BlueAmpSide", autoStartPosition.BlueAmp);
        sendableChooser.addOption("RedSourceSide", autoStartPosition.RedSource);
        sendableChooser.addOption("BlueSourceSide", autoStartPosition.BlueSource);
        sendableChooser.addOption("The Bot Ain't Even On The Field", autoStartPosition.Nowhere);
        return sendableChooser;
    }

    public static SendableChooser<Command> sendableChooser(RobotContainer container)
    {
        SendableChooser<Command> sendableChooser = new SendableChooser<>();
        sendableChooser.setDefaultOption("Nothing", new WaitCommand(0));
        sendableChooser.addOption("Center Shoot and Leave",
            shootNoteAndPathLeave(container, ShooterSubsystem.ShooterLocation.CenterSpeaker, "Leave From Center"));
        sendableChooser.addOption("Red Right Shoot and Leave", //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.RightSpeaker, true));
        sendableChooser.addOption("Red Left Shoot and Leave",  //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.LeftSpeaker,true ));
        sendableChooser.addOption("Blue Right Shoot and Leave", //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.RightSpeaker, false));
        sendableChooser.addOption("Blue Left Shoot and Leave",  //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.LeftSpeaker,false ));
        sendableChooser.addOption("Blue Two Note Center and Leave",
            centerShootAndPickUpCenterNoteAndLeave(container));  
        sendableChooser.addOption("Red Two Note Center and Leave",
            redCenterShootAndPickUpCenterNoteAndLeave(container));  
        sendableChooser.addOption("Three Note Center",
            threeNoteCenter(container));  
        sendableChooser.addOption("Shoot Only Right",
          initialSpeakerNoteShot(container, ShooterLocation.RightSpeaker));
        sendableChooser.addOption("Shoot Only Left",
          initialSpeakerNoteShot(container, ShooterLocation.LeftSpeaker));
        return sendableChooser;
    }

    public static void registerPathPlannerNamedCommands(RobotContainer container)
    {
        // Register Named Commands - things to do at marked points, e.g. start intake, shoot, etc
        // Must match names in PathPlanner!
        NamedCommands.registerCommand("startIntake", new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)
            .alongWith(new InstantCommand(() ->
                container.m_shooterSubsystem.stopShooterMotor(), container.m_shooterSubsystem)));

        NamedCommands.registerCommand("setShooterCenter", new InstantCommand(() ->
            container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.CenterSpeaker), container.m_shooterSubsystem));
        NamedCommands.registerCommand("setShooterLeft", new InstantCommand(() ->
            container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.LeftSpeaker), container.m_shooterSubsystem));
        NamedCommands.registerCommand("setShooterRight", new InstantCommand(() ->
            container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.RightSpeaker), container.m_shooterSubsystem));
        NamedCommands.registerCommand("startShooterTrap", new InstantCommand(() ->
            container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.Trap), container.m_shooterSubsystem));

        NamedCommands.registerCommand("delayedShootNote",
            new WaitCommand(container.m_delayTimeForShooter)
            .andThen(new FeedNoteIntoShooterCommand(container.m_intakeSubsystem)));
        NamedCommands.registerCommand("shootNote", new FeedNoteIntoShooterCommand(container.m_intakeSubsystem));

        NamedCommands.registerCommand("stopShooter", new InstantCommand(() ->
            container.m_shooterSubsystem.stopShooterMotor(), container.m_shooterSubsystem));
    }

    private static Command runFirstPathFromPathPlanner(RobotContainer container, String name)
    {
        return Commands.sequence(
            new InstantCommand(() -> {
                var path = PathPlannerPath.fromPathFile(name);
                // path.getPreviewStartingHolonomicPose()
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                    if (alliance.get() == DriverStation.Alliance.Red)
                    {
                        var newPath = path.flipPath();
                        container.m_driveSubsystem.resetOdometry(newPath.getPathPoses().get(0));
                        return;
                    }
                }
                container.m_driveSubsystem.resetOdometry(path.getPathPoses().get(0));
            }),
            runPathFromPathPlanner(name));
    }

    private static Command runPathFromPathPlanner(String name)
    {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    private static Command shootNoteAndPathLeave(
        RobotContainer container,
        ShooterLocation speakerLocation,
        String autoPathName)
    {
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            runFirstPathFromPathPlanner(container, "Leave From Center")
                .alongWith(stopShooterAfterNoteDelivery(container))
            ); //why is this code sad?
    }

    private static Command shootNoteAndLeave(
        RobotContainer container,
        ShooterLocation speakerLocation,
        boolean isRedAlliance)
    {
        // var isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        var isRightSide = speakerLocation == ShooterLocation.RightSpeaker;
        if (isRedAlliance)
        {
            if (isRightSide)
                return shootNoteAndLeaveAtAngle(container, speakerLocation, false);
            else
                return shootAndLeaveStraight(container, speakerLocation);
        }
        else
        {
            if (isRightSide)
                return shootAndLeaveStraight(container, speakerLocation);
            else
                return shootNoteAndLeaveAtAngle(container, speakerLocation, true);
        }
    }

    private static Command shootNoteAndLeaveAtAngle(
        RobotContainer container,
        ShooterLocation speakerLocation,
        boolean goRight)
    {
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            new InstantCommand(() -> container.m_driveSubsystem.drive(2, goRight ? -2 : 2, 0, true), container.m_driveSubsystem),
            new WaitCommand(1.0),
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, true), container.m_driveSubsystem)
            ); //why is this code sad?
    }

    private static Command shootAndLeaveStraight(
        RobotContainer container,
        ShooterLocation speakerLocation)
    {
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            new InstantCommand(() -> container.m_driveSubsystem.drive(2, 0, 0, false), container.m_driveSubsystem),
            new WaitCommand(1.75),
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem)
            ); //why is this code sad?
    }

    private static Command threeNoteCenter(RobotContainer container)
    {
        var speakerLocation = ShooterLocation.CenterSpeaker;
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            runFirstPathFromPathPlanner(container, "Leave From Center")
                .alongWith(
                    stopShooterAfterNoteDelivery(container),
                    new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)),
            runPathFromPathPlanner("Return to Center")
                .alongWith(new InstantCommand(() ->
                    container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)),
            new FeedNoteIntoShooterCommand(container.m_intakeSubsystem),
            stopShooterAfterNoteDelivery(container),
            runPathFromPathPlanner("CenterLine Note 1 Pick")
                .alongWith(new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)),
            runPathFromPathPlanner("CenterLine Note 1 Shoot")
                .alongWith(new InstantCommand(() ->
                    container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)),
            new FeedNoteIntoShooterCommand(container.m_intakeSubsystem),
            stopShooterAfterNoteDelivery(container)
            );
    }

    private static Command centerShootAndPickUpCenterNoteAndLeave(
        RobotContainer container)
    {
        var speakerLocation = ShooterLocation.CenterSpeaker;
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            // runFirstPathFromPathPlanner(container, "Center and Back with Events")
            //     .alongWith(
            //         stopShooterAfterNoteDelivery(container),
            //         new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)),
            runFirstPathFromPathPlanner(container, "Leave From Center")
                .alongWith(
                    stopShooterAfterNoteDelivery(container),
                    new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)),
            runPathFromPathPlanner("Return to Center")
                .alongWith(new InstantCommand(() ->
                    container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)),
            new FeedNoteIntoShooterCommand(container.m_intakeSubsystem),
            // new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)
            //     .alongWith(new InstantCommand(() -> container.m_driveSubsystem.drive(2, 0, 0, true), container.m_driveSubsystem)
            //         .andThen(new WaitCommand(1.0))
            //         .andThen(new InstantCommand(() -> container.m_driveSubsystem.drive(-2, 0, 0, true), container.m_driveSubsystem))),
            // new WaitCommand(1.0),
            // new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, true), container.m_driveSubsystem),
            // new InstantCommand(() -> container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)
            //     .alongWith(new WaitCommand(container.m_delayTimeForShooter)
            //     .andThen(new FeedNoteIntoShooterCommand(container.m_intakeSubsystem))),
            stopShooterAfterNoteDelivery(container),
            runPathFromPathPlanner("Leave From Center")
            );
    }

    private static Command redCenterShootAndPickUpCenterNoteAndLeave(
        RobotContainer container)
    {
        var speakerLocation = ShooterLocation.CenterSpeaker;
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, false), container.m_driveSubsystem),
            initialSpeakerNoteShot(container, speakerLocation),
            new NoteIntakeFromFloorCommand(container.m_intakeSubsystem)
                .alongWith(new InstantCommand(() -> container.m_driveSubsystem.drive(1.5, 0, 0, true), container.m_driveSubsystem)
                    .andThen(new WaitCommand(1.5))
                    .andThen(new InstantCommand(() -> container.m_driveSubsystem.drive(-2, 0, 0, true), container.m_driveSubsystem))),
            new WaitCommand(1.0)
                .alongWith(new InstantCommand(() ->
                    container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)),
            new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, true), container.m_driveSubsystem),
            new FeedNoteIntoShooterCommand(container.m_intakeSubsystem),
            stopShooterAfterNoteDelivery(container)
            );
    }

    private static Command initialSpeakerNoteShot(
        RobotContainer container,
        ShooterLocation speakerLocation)
    {
        return Commands.sequence(
            new InstantCommand(() -> container.m_shooterSubsystem.controlShooterToVelocity(speakerLocation), container.m_shooterSubsystem)
                .alongWith(new WaitCommand(container.m_delayTimeForShooter)
                    .andThen(new FeedNoteIntoShooterCommand(container.m_intakeSubsystem)))
            );
    }

    private static Command stopShooterAfterNoteDelivery(
        RobotContainer container)
    {
        return Commands.sequence(
            new WaitCommand(0.25),
            new InstantCommand(() -> container.m_shooterSubsystem.stopShooterMotor()));
    }

    private static Command driveToPose(RobotContainer container)
    {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return Commands.sequence(
            new InstantCommand(() -> container.m_driveSubsystem.resetPoseFromLimelight())
            .andThen(AutoBuilder.pathfindToPose(container.m_driveSubsystem.m_targetDriveToPose2d, constraints))
        );
    }

    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
