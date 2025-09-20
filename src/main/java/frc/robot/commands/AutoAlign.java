// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class AutoAlign extends Command {
    private PIDController xController, yController, thetaController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private Drive drive;
    private Vision vision;
    private double tagID = -1;

    public AutoAlign(boolean isRightScore, Drive drive, Vision vision) {
        xController = new PIDController(AutoConstants.xP, AutoConstants.xI, AutoConstants.xD); // Vertical
        yController = new PIDController(AutoConstants.yP, AutoConstants.yI, AutoConstants.yD); // Horizontal
        thetaController =
                new PIDController(AutoConstants.thetaP, AutoConstants.thetaI, AutoConstants.thetaD); // Rotation
        this.isRightScore = isRightScore;
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        thetaController.setSetpoint(AutoConstants.thetaSetpoint);
        thetaController.setTolerance(AutoConstants.thetaTolerance);
        xController.setSetpoint(AutoConstants.xSetpoint);
        xController.setTolerance(AutoConstants.xTolerance);

        yController.setSetpoint(AutoConstants.ySetpoint);
        yController.setTolerance(AutoConstants.yTolerance);

        // tagID = drive.
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();

        double xSpeed = xController.calculate(drive.getChassisSpeeds().vxMetersPerSecond);
        double ySpeed = yController.calculate(drive.getChassisSpeeds().vyMetersPerSecond);
        double theta = thetaController.calculate(drive.getRotation().getDegrees());

        drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, theta));

        // drive.poseEstimator.

    }
}
