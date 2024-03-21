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

@Autonomous
public class CenterStageBaseAudienceAutoOp extends BaseAutoOp {

    CenterStageHardware gameHardware;

    CenterStagePropDetectionPipeline propDetectionPipeline;
    public double initialHeading = 0;
    public static double CLAW_DIST = 12;
    public double spikeMarkAngle = 0;
    public double mirrorFactor = 1;
    public ParkPosition parkPosition = ParkPosition.CENTER;

    public double[] detectionColor = new double[]{255, 0, 0};

    private Pose2d getFinalPose(Pose2d pose) {
        return new Pose2d(pose.getX() , pose.getY() * mirrorFactor, pose.getHeading() * mirrorFactor);
    }

    private Vector2d getFinalVec(Vector2d vec) {
        return new Vector2d(vec.getX(), vec.getY() * mirrorFactor);
    }

    private double getFinalHeading(double heading) {
        return heading * mirrorFactor;
    }

    public void autoOpInitialize () {

        gameHardware = new CenterStageHardware(hardwareMap, telemetry);
        gameHardware.initPose();
        gameHardware.clawOpen = false;
        gameHardware.clawShelf.setPosition(0.2);
        gameHardware.update(true);

        propDetectionPipeline = new CenterStagePropDetectionPipeline(new Size(), detectionColor);

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
            setCurrentHeadingAs(initialHeading + Math.PI);
            Pose2d currentPose = localAdd(getFinalPose(gridToFieldPose(1, 0, initialHeading)), new Pose2d(2.25, -(TILE_SIZE - ROBOT_WIDTH)/2));
            drive.setPoseEstimate(currentPose);
            Vector2d spikeMarkOffsetVec = (zone.get() == 0 ? SpikeMarkOffset.LEFT
                    : zone.get() == 1 ? SpikeMarkOffset.CENTER
                    : SpikeMarkOffset.RIGHT).rotated(spikeMarkAngle);
            Trajectory traj = drive.trajectoryBuilder(currentPose)
                    .lineToSplineHeading(new Pose2d(getFinalVec(gridToFieldVec(1, 0.5)).plus(spikeMarkOffsetVec).plus(new Vector2d(-CLAW_DIST, 0)), 0))
                    .addDisplacementMarker(() -> {
                        gameHardware.clawPose = ClawPose.FORWARD;
                    })
                    .splineToConstantHeading(
                            getFinalVec(gridToFieldVec(1, 1)).plus(spikeMarkOffsetVec).plus(new Vector2d(-CLAW_DIST, 0)),
                            Math.atan2(spikeMarkOffsetVec.getY(), spikeMarkOffsetVec.getX())
                    )
                    .addDisplacementMarker(() -> {
                        gameHardware.clawOpen = true;
                    })
                    .build();

            drive.followTrajectoryAsync(traj);
        }, () -> {
            gameHardware.update(false);
        }, () -> !drive.isBusy());

        // go park
        addPhase(() -> {
            gameHardware.setTargetArmRotation(0);
            gameHardware.setTargetArmExtension(0);
            gameHardware.clawPose = ClawPose.DRIVE;
            Trajectory traj;
            if (parkPosition == ParkPosition.CENTER) {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(getFinalPose(gridToFieldPose(0.2, 2, 0)), Math.PI/2)
                        .splineToSplineHeading(getFinalPose(gridToFieldPose(4.8, 2.05, 0)), 0)
                        .build();
            } else {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(getFinalPose(gridToFieldPose(3, 2, 0)), 0)
                        .splineToSplineHeading(getFinalPose(gridToFieldPose(4.8, 0, 0)), 0)
                        .build();
            }

            drive.followTrajectoryAsync(traj);
        }, () -> {
            gameHardware.update(false);
        }, () -> !drive.isBusy());

        // raise arm
        final int armExtensionPos = -5000;
        addPhase(() -> {
            gameHardware.setTargetArmRotation(1100);
            gameHardware.setTargetArmExtension(armExtensionPos);
            gameHardware.clawPose = ClawPose.DRIVE;
        }, () -> {
            gameHardware.update(false);
        }, () -> Math.abs(gameHardware.armExtender.getCurrentPosition() - armExtensionPos) < 100);

        // place pixels
        addPhase(() -> {
            gameHardware.pusherPos = PusherPosition.TWO_PIXEL;
            gameHardware.setTargetPokerRotation(0.175);
        }, () -> {
            gameHardware.update(false);
        }, () -> timeInPhase > 1);

        addPhase(() -> {
            gameHardware.setTargetPokerRotation(0.5);
            gameHardware.setTargetArmRotation(1000);
            gameHardware.setTargetArmExtension(0);
        }, () -> {
            gameHardware.update(false);
        }, () -> timeInPhase > 5);


    }

}

