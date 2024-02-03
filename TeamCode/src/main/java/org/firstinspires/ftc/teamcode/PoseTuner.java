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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config

public class PoseTuner extends LinearOpMode {


    public static double pusher_pos = 0.3;
    public static double pokerRotator_pos = 0.3;
    public static double armRotator_pos = 0;
    public static double armExtender_pos = 0;
    public static double shoulder_pos = 0.5;
    public static double wrist_pos = 0.5;
    public static double elbow_pos = 0.5;
    public static double claw_pos = 0.5;

    @Override
    public void runOpMode() {

        Servo clawShoulder = hardwareMap.get(Servo.class, "ClawShoulder"); // expansion hub servo 0
        Servo clawElbow = hardwareMap.get(Servo.class, "ClawElbow"); // expansion hub servo 1
        Servo clawWrist = hardwareMap.get(Servo.class, "ClawWrist"); // expansion hub servo 2
        Servo clawOpener = hardwareMap.get(Servo.class, "ClawOpener"); // expansion hub servo 3
        Servo pokerPusher = hardwareMap.get(Servo.class, "PokerPusher"); // control hub servo 0
        Servo pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        DcMotorEx armExtender = hardwareMap.get(DcMotorEx.class,  "ArmExtender"); // expansion hub motor 1
        DcMotorEx armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0

        armRotator.setTargetPosition(300);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setTargetPosition(0);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(0.5);
        armRotator.setPower(0.1);

        waitForStart();

        while (opModeIsActive()) {

            clawShoulder.setPosition(shoulder_pos);
            clawWrist.setPosition(wrist_pos);
            clawElbow.setPosition(elbow_pos);
            clawOpener.setPosition(claw_pos);
            armRotator.setTargetPosition((int) armRotator_pos);
            armExtender.setTargetPosition((int) armExtender_pos);
            pokerPusher.setPosition(pusher_pos);
            pokerRotator.setPosition(pokerRotator_pos);

            telemetry.update();
        }
    }
}
