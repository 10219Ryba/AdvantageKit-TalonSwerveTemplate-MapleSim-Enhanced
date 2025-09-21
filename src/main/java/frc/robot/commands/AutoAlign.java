// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

/*
 * Goal: AutoAlign to reef apriltags using vision and robotstate
 * Needs: Drive subsystem, Vision subsystem, RobotState
 * Plan: Use vision and do some sort of calculation to get to the nearest apriltag
 *       Use RobotState to get the current position of the robot
 *       Use a PID controller to drive to the apriltag
 *       Use a PID controller to align to the apriltag
 *       Stop when the robot is aligned to the apriltag
 *       Maybe use a timeout to stop the command if it takes too long
 *       Have a bunch of LoggedTunableNumbers to tune the PID controllers
 */
public class AutoAlign extends Command {
    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("AutoAlign/DrivekP");
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("AutoAlign/DrivekD");
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoAlign/ThetakP");
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoAlign/ThetakD");
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber("AutoAlign/DriveMaxVelocity");
    private static final LoggedTunableNumber driveMaxAcceleration =
            new LoggedTunableNumber("AutoAlign/DriveMaxAcceleration");
    private static final LoggedTunableNumber thetaMaxVelocity = new LoggedTunableNumber("AutoAlign/ThetaMaxVelocity");
    private static final LoggedTunableNumber thetaMaxAcceleration =
            new LoggedTunableNumber("AutoAlign/ThetaMaxAcceleration");

    static {
        drivekP.initDefault(0.5);
        drivekD.initDefault(0.0);
        thetakP.initDefault(1.0);
        thetakD.initDefault(0.0);
        driveMaxVelocity.initDefault(1.0); // meters per second
        driveMaxAcceleration.initDefault(1.0); // meters per second squared
        thetaMaxVelocity.initDefault(Math.PI); // radians per second
        thetaMaxAcceleration.initDefault(Math.PI); // radians per second squared
    }

    private final Drive drive;
    private final Supplier<Pose2d> target;
    private final Supplier<Pose2d> robot = RobotState.getInstance()::getEstimatedPose;

    private final double cameraIndex;

    private final PIDController driveController = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
    private final PIDController thetaController = new PIDController(0, 0, 0, Constants.loopPeriodSecs);

    public AutoAlign(Drive drive, Supplier<Pose2d> target, double cameraIndex) {
        this.drive = drive;
        this.target = target;
        this.cameraIndex = cameraIndex;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (drivekP.hasChanged(hashCode())
                || drivekD.hasChanged(hashCode())
                || thetakP.hasChanged(hashCode())
                || thetakD.hasChanged(hashCode())) {
            driveController.setP(drivekP.get());
            driveController.setD(drivekD.get());
            thetaController.setP(thetakP.get());
            thetaController.setD(thetakD.get());
        }

        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();
        Pose2d error = targetPose.relativeTo(currentPose);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
