package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class GenericMaxVelocityTuner extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double currentPos;
    double startPos;
    double maxVelocity = 0.0;
    double dir = 1.0;
    public static String DEVICE_NAME = "";
    public static double TICKS_PER_REVOLUTION = 537.7;
    public static int MIN_REVOLUTIONS = 0;
    public static int MAX_REVOLUTIONS = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, DEVICE_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        startPos = motor.getCurrentPosition();

        telemetry.addData("WARNING", "Make sure variables are set before starting.");
        telemetry.addData("Info", "If min revolutions == max revolutions, the motor will spin without limits.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = Math.abs(motor.getVelocity());
            currentPos = motor.getCurrentPosition() - startPos;
            if (MIN_REVOLUTIONS != MAX_REVOLUTIONS) {
                if (currentPos > MAX_REVOLUTIONS * TICKS_PER_REVOLUTION) {
                    dir = -1;
                } else if (currentPos < MIN_REVOLUTIONS * TICKS_PER_REVOLUTION) {
                    dir = 1;
                }
            }

            motor.setPower(dir);

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity (TPS)", currentVelocity);
            telemetry.addData("current velocity (RPM)", currentVelocity/TICKS_PER_REVOLUTION * 60);
            telemetry.addData("maximum velocity (TPS)", maxVelocity);
            telemetry.addData("maximum velocity (RPM)", maxVelocity/TICKS_PER_REVOLUTION * 60);
            telemetry.addData("kF", 32767 / maxVelocity);
            telemetry.update();
        }
    }
}
