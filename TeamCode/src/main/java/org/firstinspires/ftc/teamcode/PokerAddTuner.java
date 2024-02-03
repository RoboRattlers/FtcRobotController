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

import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config

public class PokerAddTuner extends LinearOpMode {

    public static double shoulder_start_pos = 0.62;
    public static double shoulder_end_pos = 0.8;
    public static double elbow_start_pos = 0.4;
    public static double elbow_end_pos = 0.5;
    public static double wrist_start_pos = 0.7;
    public static double wrist_end_pos = 0.6;
    public static double arm_extender_pos = -300;
    public static double poker_rotator_pos = 1;

    @Override
    public void runOpMode() {

        Servo clawShoulder = hardwareMap.get(Servo.class, "ClawShoulder");
        Servo clawElbow = hardwareMap.get(Servo.class, "ClawElbow");
        Servo clawWrist = hardwareMap.get(Servo.class, "ClawWrist");
        Servo pokerRotator = hardwareMap.get(Servo.class, "PokerRotator");
        DcMotorEx armExtender = hardwareMap.get(DcMotorEx.class, "ArmExtender");
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        armExtender.setTargetPosition(0);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(1);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

            double pos = Math.sin(runtime.seconds() * 0.5);

            clawShoulder.setPosition(map(pos, -1, 1, shoulder_start_pos, shoulder_end_pos, true));
            clawWrist.setPosition(map(pos, -1, 1, wrist_start_pos, wrist_end_pos, true));
            clawElbow.setPosition(map(pos, -1, 1, elbow_start_pos, elbow_end_pos, true));
            pokerRotator.setPosition(poker_rotator_pos);
            armExtender.setTargetPosition((int) arm_extender_pos);

            telemetry.update();
        }
    }
}
