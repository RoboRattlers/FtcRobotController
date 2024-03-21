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

import static org.firstinspires.ftc.teamcode.util.MoreMath.lerp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config

public class PokerAddTuner2 extends LinearOpMode {

    public static double shoulder_pos = 0.7;
    public static double wrist_pos = 0.5;
    public static double pokerRotation = 0;
    public static double shelfRotation = 0;

    private static Vector2d pokerEndpointOffset = new Vector2d(-53, 200);
    private static Vector2d rotatorShaftXOffset = new Vector2d(-32, 0);
    private static Vector2d rotatorShaftYOffset = new Vector2d(0, 240);
    private static Vector2d pokerEndpoint = rotatorShaftXOffset.plus(rotatorShaftYOffset).plus(pokerEndpointOffset);
    public static double startPokerEndpointX = pokerEndpoint.getX();
    public static double startPokerEndpointY = pokerEndpoint.getY();
    public static double endPokerEndpointX = pokerEndpoint.getX();
    public static double endPokerEndpointY = pokerEndpoint.getY();
    public static double oscillation_speed_mult = 2;
    public static boolean clawOpen = true;

    @Override
    public void runOpMode() {

        CenterStageHardware gameHardware = new CenterStageHardware(hardwareMap, telemetry);
        ElapsedTime runtime = new ElapsedTime();
        gameHardware.clawPose = ClawPose.LOAD;
        gameHardware.setTargetArmRotation(500);
        gameHardware.setTargetArmExtension(0);
        gameHardware.update(true);

        waitForStart();



        while (opModeIsActive()) {

            double pos = map(Math.sin(runtime.seconds() * oscillation_speed_mult), -1, 1, 0, 1, true);
            gameHardware.setIKPokerEndpoint(new Pose2d(
                    lerp(startPokerEndpointX, endPokerEndpointX, pos),
                    lerp(startPokerEndpointY, endPokerEndpointY, pos),
                    pokerRotation
            ));

            gameHardware.clawPose = ClawPose.LOAD;
            gameHardware.clawOpen = clawOpen;
            gameHardware.clawShelf.setPosition(shelfRotation);
            gameHardware.update(true);
            telemetry.update();
        }
    }
}
