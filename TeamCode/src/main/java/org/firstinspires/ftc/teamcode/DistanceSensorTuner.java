package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class DistanceSensorTuner extends LinearOpMode {
    public static double BOARD_ALIGNMENT_MAX_POWER_DIST = 200; //mm
    public static double BOARD_ALIGNMENT_TARGET_DIST = 220; // mm
    public static double BOARD_ALIGNMENT_MAX_POWER = 0.4;
    public static double BOARD_ALIGNMENT_EXPONENT = 2;

    private double powerCurve(double num) {
        return BOARD_ALIGNMENT_MAX_POWER * Math.pow(Math.abs(clamp(num, -1, 1)), 1/BOARD_ALIGNMENT_EXPONENT) * Math.signum(num);
    }

    @Override
    public void runOpMode() {

        DcMotorEx w1 = hardwareMap.get(DcMotorEx.class, "TopRightWheel");
        DcMotorEx w2 = hardwareMap.get(DcMotorEx.class, "BottomRightWheel");
        DcMotorEx w3 = hardwareMap.get(DcMotorEx.class, "BottomLeftWheel");
        DcMotorEx w4 = hardwareMap.get(DcMotorEx.class, "TopLeftWheel");

        Servo pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        DcMotorEx armExtender = hardwareMap.get(DcMotorEx.class,  "ArmExtender"); // expansion hub motor 1
        DcMotorEx armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0

        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "BackboardDistanceSensor");

        armRotator.setTargetPosition(300);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setTargetPosition(0);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(0.5);
        armRotator.setPower(0.1);


        DcMotorEx[] list = new DcMotorEx[]{w1, w2, w3, w4};
        double[] powers = new double[]{1, 1, -1, -1};

        waitForStart();

        while (opModeIsActive()) {

            double motion = powerCurve((BOARD_ALIGNMENT_TARGET_DIST - distanceSensor.getDistance(DistanceUnit.MM))/BOARD_ALIGNMENT_MAX_POWER_DIST);

            for (int i = 0; i < 4; i++) {
                DcMotorEx motor = list[i];
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setPower(powers[i] * motion);
            }

            pokerRotator.setPosition(0.175);
            armRotator.setTargetPosition(1400);
            armExtender.setTargetPosition(-2000);
            telemetry.update();
        }

    }
}
