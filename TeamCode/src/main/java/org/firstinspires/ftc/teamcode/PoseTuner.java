package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class PoseTuner extends LinearOpMode {

    public static double shoulder = 0.5;
    public static double elbow = 0.5;
    public static double wrist = 0.5;
    public static double claw = 0.5;

    @Override
    public void runOpMode() {

        DcMotorEx w1 = hardwareMap.get(DcMotorEx.class, "TopRightWheel");
        DcMotorEx w2 = hardwareMap.get(DcMotorEx.class, "BottomRightWheel");
        DcMotorEx w3 = hardwareMap.get(DcMotorEx.class, "BottomLeftWheel");
        DcMotorEx w4 = hardwareMap.get(DcMotorEx.class, "TopLeftWheel");



        DcMotorEx[] list = new DcMotorEx[]{w1, w2, w3, w4};
        double[] powers = new double[]{1, 0.5, -0.5, -1};

        waitForStart();

        while (opModeIsActive()) {

            for (int i = 0; i < 4; i++) {
                DcMotorEx motor = list[i];
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setPower(powers[i]);
            }

            telemetry.addData("w1", w1);
            telemetry.update();
        }

    }
}
