/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.IOException;

enum CoordinateSystem {
    ROBOT,
    WORLD,
    TARGET_HEADING
}

@Config
public class BaseController extends LinearOpMode {

    public static double DRIVE_TURN_ACCEL = 5.75;
    public static double DRIVE_TURN_JERK = 100;
    public static double DRIVE_TURN_VEL_SMOOTHING_THRESHOLD = 1;
    public static double DRIVE_JERK = 100;
    public static double DRIVE_ACCEL = 8;
    public static double DRIVE_VEL_SMOOTHING_THRESHOLD = 0.1;
    public ElapsedTime runtime = new ElapsedTime();

    public final double MM_TO_IN = 1/25.4;
    public final double FIELD_SIZE = 141.345;
    public final double TILE_SIZE = FIELD_SIZE/6.0;

    // movement stuff
    public SampleMecanumDrive drive;
    public Localizer localizer;
    private double targetHeading = 0;
    private double turnVelocity = 0;
    private Vector2d moveDir = new Vector2d();
    private final double ROTATION_DAMPENING_THRESHOLD = Math.toRadians(45);
    private final double ROTATION_POWER = 0.9;
    private final TargetFollower xMoveDirFollower = new TargetFollower(DRIVE_ACCEL, DRIVE_JERK, DRIVE_VEL_SMOOTHING_THRESHOLD);
    private final TargetFollower yMoveDirFollower = new TargetFollower(DRIVE_ACCEL, DRIVE_JERK, DRIVE_VEL_SMOOTHING_THRESHOLD);
    private final TargetFollower turnVelFollower = new TargetFollower(DRIVE_TURN_ACCEL, DRIVE_TURN_JERK, DRIVE_TURN_VEL_SMOOTHING_THRESHOLD);

    // rotation stuff
    public final double RIGHT_ANGLE = Math.PI/2.0;

    private double lastTick = 0.0;
    public double deltaTime = 0.0;

    public boolean useSmoothMovement = true;

    public void applyTargetHeading() {
        double turnDiff = normalizeAngle(targetHeading - localizer.getPoseEstimate().getHeading(), AngleUnit.RADIANS);
        telemetry.addData("turn diff", turnDiff);
        double tvel = Math.max(Math.min(turnDiff, ROTATION_DAMPENING_THRESHOLD), -ROTATION_DAMPENING_THRESHOLD)/ ROTATION_DAMPENING_THRESHOLD * ROTATION_POWER;
        if (Math.abs(tvel) > 0.02) {
            turnVelocity = tvel;
        } else {
            turnVelocity = 0;
        }
    }

    public void setTurnVelocity(double tvel) {
        turnVelocity = tvel;
    }

    public void setTargetHeading(double thead) {
        targetHeading = normalizeAngle(thead, AngleUnit.RADIANS);
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setMoveDir(Vector2d dir, CoordinateSystem space) {
        if (dir.distTo(new Vector2d()) == 0) {
            moveDir = new Vector2d();
            return;
        }
        double transformedHeading;
        Pose2d transformedDir;
        double rawHeading = Math.atan2(dir.getY(), dir.getX());
        double robotHeading = localizer.getPoseEstimate().getHeading();
        if (space == CoordinateSystem.WORLD) {
            transformedHeading = rawHeading - robotHeading;
        } else if (space == CoordinateSystem.TARGET_HEADING) {
            double headingInWorldSpace = rawHeading - targetHeading;
            transformedHeading = headingInWorldSpace - robotHeading;
        } else {
            transformedHeading = rawHeading;
        }

        moveDir = new Vector2d(Math.cos(transformedHeading), Math.sin(transformedHeading)).times(dir.distTo(new Vector2d()));

    }
    public void baseInitialize() {

        drive = new SampleMecanumDrive(hardwareMap);
        localizer = drive.getLocalizer();

        // INITIALIZATION TELEMETRY
        {
            telemetry.addData("Status", "Initialized.");

            // load configuration
            try {
                File file = AppUtil.getInstance().getSettingsFile("DriveTrainConfig.json");
                String serializedConfig = ReadWriteFile.readFileOrThrow(file);
                telemetry.addData("Got config:", serializedConfig);
            } catch (IOException error) {
                telemetry.addData("Error while loading config: ", error.getMessage());
            }

            telemetry.addData("Game", "Press START to run >>");
            telemetry.update();
        }

    }

    public void baseUpdate() {

        deltaTime = runtime.seconds() - lastTick;
        lastTick = runtime.seconds();



        // OTHER TELEMETRY AND POST-CALCULATION STUFF
        {
            xMoveDirFollower.setTargetPosition(moveDir.getX());
            yMoveDirFollower.setTargetPosition(moveDir.getY());
            turnVelFollower.setTargetPosition(turnVelocity);
            xMoveDirFollower.update();
            yMoveDirFollower.update();
            turnVelFollower.update();
            xMoveDirFollower.setMaxAcceleration(DRIVE_JERK);
            xMoveDirFollower.setMaxVelocity(DRIVE_ACCEL);
            xMoveDirFollower.setVelocityDampeningThreshold(DRIVE_VEL_SMOOTHING_THRESHOLD);
            yMoveDirFollower.setMaxAcceleration(DRIVE_JERK);
            yMoveDirFollower.setMaxVelocity(DRIVE_ACCEL);
            yMoveDirFollower.setVelocityDampeningThreshold(DRIVE_VEL_SMOOTHING_THRESHOLD);
            turnVelFollower.setVelocityDampeningThreshold(DRIVE_TURN_VEL_SMOOTHING_THRESHOLD);
            turnVelFollower.setMaxVelocity(DRIVE_TURN_ACCEL);
            turnVelFollower.setMaxAcceleration(DRIVE_TURN_JERK);
            if (!drive.isBusy()) {
                drive.setDriveSignal(new DriveSignal(
                        useSmoothMovement ? new Pose2d(
                                xMoveDirFollower.getCurrentPosition() * 50,
                                yMoveDirFollower.getCurrentPosition() * 50,
                                turnVelocity * 3.25
                        )
                                : new Pose2d(moveDir, turnVelocity * 3.25)
                ));
            }
            drive.update();
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Local Movement Vector", moveDir);
            //telemetry.addData("Local Displacement from Motor Encoders", displacement);
            Vector2d displacement = localizer.getPoseEstimate().vec();
            telemetry.addData("X Displacement (Inches)", displacement.getX());
            telemetry.addData("Y Displacement (Inches)", displacement.getY());
            telemetry.addData("Heading", Math.toDegrees(localizer.getPoseEstimate().getHeading()));
            telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
        }
    }

    @Override
    public void runOpMode() {

    }
}
