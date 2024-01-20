package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterStageHardware {

    private final Servo clawShoulder;
    private final Servo clawElbow;
    private final Servo clawWrist   ;
    private final Servo pokerPusher;
    private final DcMotorEx armRotator;
    private final Servo pokerRotator;
    private final DcMotorEx armExtender;
    private HardwareMap hardwareMap;
    // 0.85 for down, 0.55 for up
    private Servo clawOpener;

    public boolean clawOpen = false;

    public CenterStageHardware(HardwareMap hardware) {
        hardwareMap = hardware;
        clawShoulder = hardwareMap.get(Servo.class, "ClawShoulder"); // expansion hub servo 0
        clawElbow = hardwareMap.get(Servo.class, "ClawElbow"); // expansion hub servo 1
        clawWrist = hardwareMap.get(Servo.class, "ClawWrist"); // expansion hub servo 2
        clawOpener = hardwareMap.get(Servo.class, "ClawOpener"); // expansion hub servo 3
        pokerPusher = hardwareMap.get(Servo.class, "PokerPusher"); // control hub servo 0
        pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0
        armExtender = hardwareMap.get(DcMotorEx.class, "ArmExtender"); // expansion hub motor 1
    }

    public void defaultPose() {

    }

    public void update() {

    }

}
