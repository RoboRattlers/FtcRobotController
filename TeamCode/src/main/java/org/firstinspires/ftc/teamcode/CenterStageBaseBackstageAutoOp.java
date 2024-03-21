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

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.CLAW_DIST;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.DIST_FROM_TILE_CENTER_WHEN_AUTO_INIT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.atomic.AtomicInteger;

class SpikeMarkOffset {
    public static final Vector2d LEFT = new Vector2d(-TILE_SIZE/2, TILE_SIZE * 0.25);
    public static final Vector2d CENTER = new Vector2d(0, TILE_SIZE * 0.5);
    public static final Vector2d RIGHT = new Vector2d(TILE_SIZE/2, TILE_SIZE * 0.25);
}

enum ParkPosition {
    CORNER,
    CENTER
}

@Autonomous
public class CenterStageBaseBackstageAutoOp extends BaseAutoOp {

    CenterStageHardware gameHardware;

    CenterStagePropDetectionPipeline propDetectionPipeline;
    public double initialHeading = 0;
    public static double CLAW_DIST = 12;
    public double spikeMarkAngle = 0;
    public double mirrorFactor = 1;
    public ParkPosition parkPosition = ParkPosition.CORNER;

    public double[] detectionColor = new double[]{255, 0, 0};

    private Pose2d getFinalPose(Pose2d pose) {
        return new Pose2d(pose.getX() , pose.getY() * this.mirrorFactor, pose.getHeading() * this.mirrorFactor);
    }

    private Vector2d getFinalVec(Vector2d vec) {
        return new Vector2d(vec.getX(), vec.getY() * this.mirrorFactor);
    }

    private double getFinalHeading(double heading) {
        return heading * this.mirrorFactor;
    }

    public void autoOpInitialize () {

        gameHardware = new CenterStageHardware(hardwareMap, telemetry);
        gameHardware.initPose();
        gameHardware.clawOpen = false;
        gameHardware.clawShelf.setPosition(0.2);
        gameHardware.update(true);

        propDetectionPipeline = new CenterStagePropDetectionPipeline(new Size(1, 1), this.detectionColor);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        camera.setPipeline(propDetectionPipeline);

        AtomicInteger zone = new AtomicInteger(-1);

        // get zone
        addPhase(() -> {
        }, () -> {
            zone.set(propDetectionPipeline.getZone());
        }, () -> !drive.isBusy() || timeInPhase > 3);

        // go to claw lowering position
        addPhase(() -> {
            setCurrentHeadingAs(this.initialHeading + Math.PI);
            gameHardware.clawPose = ClawPose.DRIVE_LOW;
            Pose2d currentPose = localAdd(getFinalPose(gridToFieldPose(3, 0, this.initialHeading)), new Pose2d(2.25, -(TILE_SIZE - ROBOT_WIDTH)/2));
            drive.setPoseEstimate(currentPose);
            Vector2d spikeMarkOffsetVec = (zone.get() == 0 ? SpikeMarkOffset.LEFT
                    : zone.get() == 1 ? SpikeMarkOffset.CENTER
                    : SpikeMarkOffset.RIGHT).rotated(this.spikeMarkAngle);
            gameHardware.setTargetArmRotation(800);
            Trajectory traj = drive.trajectoryBuilder(currentPose)
                    .lineToSplineHeading(new Pose2d(getFinalVec(gridToFieldVec(3.3, 0.5)).plus(spikeMarkOffsetVec).plus(new Vector2d(CLAW_DIST, 0)), Math.PI))
                    .addDisplacementMarker(() -> {
                        gameHardware.clawPose = ClawPose.FORWARD;
                    })
                    .splineToConstantHeading(
                            getFinalVec(gridToFieldVec(3, 1)).plus(spikeMarkOffsetVec).plus(new Vector2d(CLAW_DIST, 0)),
                            Math.atan2(spikeMarkOffsetVec.getY(), spikeMarkOffsetVec.getX())
                    )
                    .addDisplacementMarker(() -> {
                        gameHardware.clawOpen = true;
                    })
                    .build();

            drive.followTrajectoryAsync(traj);
        }, () -> {
            telemetry.addData("Initial Heading?", this.initialHeading);
            gameHardware.update(false);
        }, () -> !drive.isBusy());

        addPhase(() -> {
            gameHardware.setTargetArmRotation(1600);
            gameHardware.setTargetArmExtension(-1500);
            gameHardware.setTargetPokerRotation(0.175);
            gameHardware.clawPose = ClawPose.DRIVE;
            Vector2d zoneBoardOffset = new Vector2d(0, map((double) zone.get(), 0, 2, TILE_SIZE/6.5, -TILE_SIZE/6.5, true) + 2.5);
            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(3)
                    .splineToConstantHeading(getFinalVec(gridToFieldVec(4.2, 1)).plus(zoneBoardOffset), 0)
                    .build();

            drive.followTrajectoryAsync(traj);
        }, () -> {
            gameHardware.update(false);
        }, () -> !drive.isBusy());

        addPhase(() -> {
        }, () -> {
            setMoveDir(new Vector2d(-0.2, 0), CoordinateSystem.ROBOT);
            if (timeInPhase > 0.5) {
                gameHardware.pusherPos = PusherPosition.TWO_PIXEL;
            }
            gameHardware.update(false);
        }, () -> timeInPhase > 1);

        addPhase(() -> {
            gameHardware.setTargetArmRotation(500);
            gameHardware.setTargetArmExtension(0);
            gameHardware.clawPose = ClawPose.FORWARD;
        }, () -> {
            gameHardware.update(false);
        }, () -> timeInPhase > 0.7);

        addPhase(() -> {
            setMoveDir(new Vector2d(), CoordinateSystem.ROBOT);
            gameHardware.setTargetArmRotation(100);
            gameHardware.setTargetArmExtension(0);
            gameHardware.clawPose = ClawPose.FORWARD;
            Trajectory traj;
            if (parkPosition == ParkPosition.CORNER) {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(3)
                        .splineToConstantHeading(getFinalVec(gridToFieldVec(4.5, 0)), 0)
                        .splineToConstantHeading(getFinalVec(gridToFieldVec(5, 0)), 0)
                        .build();
            } else {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(3)
                        .splineToConstantHeading(getFinalVec(gridToFieldVec(4.5, 2)), 0)
                        .splineToConstantHeading(getFinalVec(gridToFieldVec(5, 2)), 0)
                        .build();
            }

            drive.followTrajectoryAsync(traj);
        }, () -> {
            gameHardware.update(false);
        }, () -> !drive.isBusy());

    }

}

