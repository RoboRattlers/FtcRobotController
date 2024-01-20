package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Config
@TeleOp
public class PoseTuner extends LinearOpMode {

    public static String[] ActuatorNames = new String[16];
    public static double[] ActuatorPositions = new double[16];

    @Override
    public void runOpMode() {

        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<Servo> servos = hardwareMap.getAll(Servo.class);

        int i = 0;
        for (DcMotor motor : motors) {
            ActuatorNames[i] = motor.getDeviceName();
            ActuatorPositions[i] = motor.getCurrentPosition();
            i++;
        }
        for (Servo servo : servos) {
            ActuatorNames[i] = servo.getDeviceName();
            ActuatorPositions[i] = 0.5;
            i++;
        }

        waitForStart();

        while (opModeIsActive()) {
            int e = 0;
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
                motor.setTargetPosition((int) ActuatorPositions[e]);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                e++;
            }
            for (Servo servo : servos) {
                telemetry.addData(servo.getDeviceName(), servo.getPosition());
                servo.setPosition(ActuatorPositions[e]);
                e++;
            }
            telemetry.update();
        }

    }
}
