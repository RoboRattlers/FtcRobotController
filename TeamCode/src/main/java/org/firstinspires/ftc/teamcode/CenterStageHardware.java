package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterStageHardware {

    private HardwareMap hardwareMap;
    // 0.85 for down, 0.55 for up
    private Servo clawSwinger;
    private Servo clawOpener;

    public boolean clawOpen = false;

    public CenterStageHardware(HardwareMap hardware) {
        hardwareMap = hardware;
        clawSwinger = hardwareMap.get(Servo.class, "ClawSwinger");
        clawOpener = hardwareMap.get(Servo.class, "ClawOpener");
    }

    public void defaultPose() {
        clawOpener.setPosition(0.91);
        clawSwinger.setPosition(0.55);
    }

    public void rotateClaw(double angle) {
        clawSwinger.setPosition(map(angle, 0, 1, 0.91, 0.55, true));
    }

    public void update() {
        clawOpener.setPosition((clawOpen && clawSwinger.getPosition() > 0.6) ? 0.91
                : clawOpen ? 0.8
                : 1);
    }

}
