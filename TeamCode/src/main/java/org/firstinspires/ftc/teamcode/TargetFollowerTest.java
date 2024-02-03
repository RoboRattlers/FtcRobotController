package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class TargetFollowerTest extends LinearOpMode {

    public static double maxVelocity = 1, minVelocityMult = 0, velocityDampeningThreshold = 0.3, acceleration = 10;
    public static double target = 1;


    @Override
    public void runOpMode() {

        TargetFollower follower = new TargetFollower(maxVelocity, acceleration, 20);//

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        waitForStart();
        while (opModeIsActive()) {
            follower.setMaxVelocity(maxVelocity);
            follower.setMaxAcceleration(acceleration);
            follower.setMinVelocityMult(minVelocityMult);
            follower.setVelocityDampeningThreshold(velocityDampeningThreshold);
            follower.setTargetPosition(target);
            follower.update();
            String str = "";
            for (int i = 0; i < follower.getCurrentPosition(); i++) {
                str += "I";
            }
            dashboardTelemetry.addData("pos", follower.getCurrentPosition());
            dashboardTelemetry.addData("accel", follower.getCurrentAcceleration());
            dashboardTelemetry.addData("vel", follower.getCurrentVelocity());
            dashboardTelemetry.update();
            telemetry.addData("Stuff", str);
        }

    }
}
