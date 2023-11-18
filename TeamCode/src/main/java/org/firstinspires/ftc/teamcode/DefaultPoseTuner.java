package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class DefaultPoseTuner extends LinearOpMode {

    public static int armSwingerPos = 0;
    public static int armExtensorPos = 0;
    public static double clawSwingerPos = 0.5;
    public static double clawOpenerPos = 0.5;

    @Override
    public void runOpMode() {
        DcMotorEx armSwinger = hardwareMap.get(DcMotorEx.class, "ArmSwinger");
        DcMotorEx armExtensor = hardwareMap.get(DcMotorEx.class, "ArmExtensor");
        Servo clawSwinger = hardwareMap.get(Servo.class, "ClawSwinger");
        Servo clawOpener = hardwareMap.get(Servo.class, "ClawOpener");
        armSwinger.setTargetPosition(armSwingerPos);
        armExtensor.setTargetPosition(armExtensorPos);
        armSwinger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            armSwinger.setPower(0.1);
            armExtensor.setPower(0.1);
            armSwinger.setTargetPosition(armSwingerPos);
            armExtensor.setTargetPosition(armExtensorPos);
            clawSwinger.setPosition(clawSwingerPos);
            clawOpener.setPosition(clawOpenerPos);


            telemetry.update();
        }
    }
}
