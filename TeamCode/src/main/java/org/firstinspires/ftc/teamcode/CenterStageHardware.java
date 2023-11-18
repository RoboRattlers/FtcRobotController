package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterStageHardware {

    private HardwareMap hardwareMap;
    private DcMotorEx armSwinger = hardwareMap.get(DcMotorEx.class, "ArmSwinger");
    public ViperSlide336mm slide = new ViperSlide336mm(hardwareMap.get(DcMotorEx.class, "ArmExtensor"));
    // 0.8 for up, 0.5 for middle, 0.1 for down
    private Servo clawSwinger = hardwareMap.get(Servo.class, "ClawSwinger");
    private Servo clawOpener = hardwareMap.get(Servo.class, "ClawOpener");

    public boolean clawOpen = false;

    public TargetFollower armSwingerFollower = new TargetFollower(2000, 2000);

    public CenterStageHardware(HardwareMap hardware) {
        hardwareMap = hardware;
        armSwingerFollower.velocityDampeningThreshold = 300;
    }

    public void defaultPose() {
        armSwingerFollower.targetPosition = 0;
        slide.setTargetHeight(0);
        clawOpener.setPosition(0.91);
        clawSwinger.setPosition(0.8);
    }

    public void rotateArmToBackboard() {
        armSwingerFollower.targetPosition = -1600;
        clawSwinger.setPosition(0.8);
    }

    public void rotateArmFlat() {
        armSwingerFollower.targetPosition = 0;
    }

    public void rotateClaw(double angle) {
        angle = map(angle, 0, Math.PI, 0.2, 0.8, true);
        if (armSwingerFollower.targetPosition < -10) {
            angle = Math.min(angle, 0.5);
        }
        clawSwinger.setPosition(angle);
    }

    public void update() {
        armSwingerFollower.update(armSwinger.getCurrentPosition(), armSwinger.getVelocity());
        armSwinger.setVelocity(armSwingerFollower.getCurrentVelocity());
        if (armSwingerFollower.targetPosition > -10 && slide.getTargetHeight() > 0) {
            slide.setTargetHeight(0);
            clawSwinger.setPosition(0.8);
        }
        if (armSwingerFollower.targetPosition < -10) {
            clawSwinger.setPosition(Math.min(clawSwinger.getPosition(), 0.5));
        }

        clawOpener.setPosition((clawOpen && armSwingerFollower.targetPosition < -10) ? 0.91
                : clawOpen ? 0.8
                : 1);
        slide.update();
    }

}
