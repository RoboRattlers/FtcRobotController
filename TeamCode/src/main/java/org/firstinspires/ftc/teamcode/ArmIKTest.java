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

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config

public class ArmIKTest extends LinearOpMode {

    // mm
    private static Vector2d pokerEndpointOffset = new Vector2d(-53, 180);
    private static Vector2d rotatorShaftXOffset = new Vector2d(-32, 0);
    private static Vector2d rotatorShaftYOffset = new Vector2d(0, 240);
    private static Vector2d pokerEndpoint = rotatorShaftXOffset.plus(rotatorShaftYOffset).plus(pokerEndpointOffset);
    public static double pokerEndpointX = pokerEndpoint.getX();
    public static double pokerEndpointY = pokerEndpoint.getY();
    private static TargetFollower pokerEndpointXFollower = new TargetFollower(100, 50, 40);

    private static TargetFollower pokerEndpointYFollower = new TargetFollower(100, 50, 40);
    private static TargetFollower pokerRotationFollower = new TargetFollower(160, 300, 30);
    public static double pokerRotation = 0; // degrees

    private double armCountPerRotation = ((1.0+(46.0/11.0)) * 28.0) * 28.0;
    private double pokerRotatorCountPerRotation = 1.0/(300.0/360.0);
    private double armCountPerMm = 1425.1/120; //120mm of distance on the pulley

    private final double initialRotatorShaftAngle = Math.atan2(rotatorShaftYOffset.getY(), rotatorShaftXOffset.getX());

    private final double ARM_RETRACTED_LENGTH = 240;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        pokerEndpointXFollower.update(pokerEndpointX);

        pokerEndpointYFollower.update(pokerEndpointY);

        Servo pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        DcMotorEx armExtender = hardwareMap.get(DcMotorEx.class,  "ArmExtender"); // expansion hub motor 1
        DcMotorEx armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0

        armRotator.setTargetPosition((int) (armCountPerRotation/4));
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setTargetPosition(0);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(1);
        armRotator.setPower(1);

        waitForStart();

        while (opModeIsActive()) {

            pokerEndpointXFollower.setTargetPosition(pokerEndpointX);

            pokerEndpointYFollower.setTargetPosition(pokerEndpointY);
            pokerEndpointXFollower.update();

            pokerEndpointYFollower.update();
            pokerRotationFollower.setTargetPosition(pokerRotation);
            pokerRotationFollower.update();
            pokerEndpoint = new Vector2d(pokerEndpointXFollower.getCurrentPosition(), pokerEndpointYFollower.getCurrentPosition());
            Vector2d rotatorShaftEndpoint = pokerEndpoint.minus(pokerEndpointOffset.rotated(Math.toRadians(pokerRotationFollower.getCurrentPosition())));
            double rotatorShaftAngle = Math.atan2(rotatorShaftEndpoint.getY(), rotatorShaftEndpoint.getX());
            Vector2d armEndpoint = rotatorShaftEndpoint.minus(rotatorShaftXOffset.rotated(rotatorShaftAngle - initialRotatorShaftAngle));
            double armRotation = Math.atan2(armEndpoint.getY(), armEndpoint.getX());
            double armLength = armEndpoint.distTo(new Vector2d());

            armRotator.setTargetPosition(clamp((int) (armRotation / (Math.PI * 2) * armCountPerRotation), 100, 1600));
            armExtender.setTargetPosition((int) clamp(-(armLength - ARM_RETRACTED_LENGTH) * armCountPerMm, -6000, 0));
            double realPokerRotation = (pokerRotationFollower.getCurrentPosition()/360) - (armRotation / (Math.PI * 2) - 0.25);
            pokerRotator.setPosition((-realPokerRotation) * pokerRotatorCountPerRotation + 0.5);


            //armRotator.setTargetPosition((int) armRotator_pos);
            //armExtender.setTargetPosition((int) armExtender_pos);
            //pokerRotator.setPosition(pokerRotator_pos);

            dashboardTelemetry.addData("Arm Rotation", armRotation);
            dashboardTelemetry.addData("Arm Extender Target", armExtender.getTargetPosition());

            dashboardTelemetry.addData("Arm Length", armLength);

            dashboardTelemetry.addData("Arm Endpoint", armEndpoint);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
}
