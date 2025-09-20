// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.FieldConstants.fieldWidth;
import static frc.robot.FieldConstants.startingLineX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MirrorUtil;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoTest {
    private final Drive drive;

    public Command testSomewhere() {
        return Commands.runOnce(
                        () -> {
                            RobotState.getInstance()
                                    .resetPose(AllianceFlipUtil.apply(MirrorUtil.apply(new Pose2d(
                                            startingLineX - TunerConstants.robotWidth / 2.0,
                                            fieldWidth / 2.0,
                                            Rotation2d.kPi))));
                        },
                        drive)
                .andThen(new DriveToPose(
                        drive,
                        () -> AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose())));
    }
}
