// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

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
    public static SendableChooser<Command> sendableChooser(RobotContainer container)
    {
        SendableChooser<Command> sendableChooser = new SendableChooser<>();
        sendableChooser.setDefaultOption("Nothing", new WaitCommand(0));
        sendableChooser.addOption("Center Shoot and Leave",
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.CenterSpeaker));
        sendableChooser.addOption("Right Shoot and Leave", //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.RightSpeaker));
        sendableChooser.addOption("Left Shoot and Leave",  //this will just shoot and go straight out, without rotating the robot
            shootNoteAndLeave(container, ShooterSubsystem.ShooterLocation.LeftSpeaker));
        sendableChooser.addOption("Two Note Center and Leave",
            centerShootAndPickUpCenterNoteAndLeave(container));  
        return sendableChooser;
    }

    public static void registerPathPlannerNamedCommands(RobotContainer container)
    {
        // Register Named Commands - things to do at marked points, e.g. start intake, shoot, etc
        // Must match names in PathPlanner!
        NamedCommands.registerCommand("grabNoteFromFloor", new NoteIntakeFromFloorCommand(container.m_intakeSubsystem));

        NamedCommands.registerCommand("startShooterCenter", new InstantCommand(() ->
            container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.CenterSpeaker), container.m_shooterSubsystem));
        // NamedCommands.registerCommand("startShooterLeft", new InstantCommand(() ->
        //     container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.LeftSpeaker), container.m_shooterSubsystem));
        // NamedCommands.registerCommand("startShooterRight", new InstantCommand(() ->
        //     container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.RightSpeaker), container.m_shooterSubsystem));
        // NamedCommands.registerCommand("startShooterTrap", new InstantCommand(() ->
        //     container.m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.Trap), container.m_shooterSubsystem));

        NamedCommands.registerCommand("waitAndFeedNoteIntoShooter",
            new WaitCommand(container.m_delayTimeForShooter)
            .andThen(new FeedNoteIntoShooterCommand(container.m_intakeSubsystem)));
        NamedCommands.registerCommand("feedNoteIntoShooter", new FeedNoteIntoShooterCommand(container.m_intakeSubsystem));

        NamedCommands.registerCommand("stopShooter", new InstantCommand(() ->
            container.m_shooterSubsystem.stopShooterMotor(), container.m_shooterSubsystem));
    }

    private static Command runPathFromPathPlanner(String name)
    {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    private static Command shootNoteAndLeave(
        RobotContainer container,
        ShooterLocation speakerLocation)
    {
        return Commands.sequence(
            initialSpeakerNoteShot(container, speakerLocation),
            runPathFromPathPlanner("Leave From Center")
                .alongWith(stopShooterAfterNoteDelivery(container))
            // new InstantCommand(() -> container.m_driveSubsystem.drive(2, 0, 0, true), container.m_driveSubsystem),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> container.m_driveSubsystem.drive(0, 0, 0, true), container.m_driveSubsystem)
            );
    }

    private static Command centerShootAndPickUpCenterNoteAndLeave(
        RobotContainer container)
    {
        var speakerLocation = ShooterLocation.CenterSpeaker;
        return Commands.sequence(
            initialSpeakerNoteShot(container, speakerLocation),
            runPathFromPathPlanner("Leave From Center")
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

    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
