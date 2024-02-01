package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;
import static org.firstinspires.ftc.teamcode.util.MoreMath.modulo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterStageHardware {

    private Servo clawShoulder;
    private Servo clawElbow;
    private Servo clawWrist;
    private Servo pokerPusher;
    private DcMotorEx armRotator;
    private Servo pokerRotator;
    private DcMotorEx armExtender;
    private Servo clawOpener;
    private HardwareMap hardwareMap;
    private int clawPose = 0;

    private double[][] clawPoses = new double[][]{
            new double[]{0.5, 0.5, 0.5}, // initialization
            new double[]{0.5, 0.5, 0.5}, // grab
            new double[]{0.5, 0.5, 0.5}, // place onto board
            new double[]{0.5, 0.5, 0.5}, // pre-push onto poker
            new double[]{0.5, 0.5, 0.5} // push onto poker
    };

    private boolean clawOpen = false;

    public CenterStageHardware(HardwareMap hardware) {
        hardwareMap = hardware;
        clawShoulder = hardwareMap.get(Servo.class, "ClawShoulder"); // expansion hub servo 0
        clawElbow = hardwareMap.get(Servo.class, "ClawElbow"); // expansion hub servo 1
        clawWrist = hardwareMap.get(Servo.class, "ClawWrist"); // expansion hub servo 2
        clawOpener = hardwareMap.get(Servo.class, "ClawOpener"); // expansion hub servo 3
        pokerPusher = hardwareMap.get(Servo.class, "PokerPusher"); // control hub servo 0
        pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0
        armExtender = hardwareMap.get(DcMotorEx.class,  "ArmExtender"); // expansion hub motor 1

        armExtender.setTargetPosition(0);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(1);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator.setTargetPosition(0);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setPower(1);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPokerPusherPos(int pos) {
        armRotator.setTargetPosition((1 - pos) * 700 + 100);
    }

    public void drivePose() {
        clawPose = 2;
        armRotator.setTargetPosition(800);
        pokerRotator.setPosition(0.5);
    }

    public void prepareForGrab() {
        /*clawPose = 1;
        clawOpen = false;
        armRotator.setTargetPosition(500);
        pokerRotator.setPosition(0.5);*/
    }

    public void prepareForPush() {
        armExtender.setTargetPosition(0);
    }

    public void prepareForArmPlace() {
        armExtender.setTargetPosition(-1500);
    }

    public void prepareForClawPlace() {
        armRotator.setTargetPosition(1000);
        pokerRotator.setPosition(0.5);
        clawPose = 2;
    }

    public void incrementClawPose(int amnt) {
        clawPose += amnt;
        clawPose = (int) modulo(clawPose, clawPoses.length);
    }

    public void updateClawOpener() {
        clawOpener.setPosition(clawOpen ? 0.5 : 0.5);
    }

    public void toggleClawOpen() {
        clawOpen = !clawOpen;
    }

    public void updateClawPose() {
        double[] currentClawPose = clawPoses[clawPose];
        clawShoulder.setPosition(currentClawPose[0]);
        clawElbow.setPosition(currentClawPose[1]);
        clawWrist.setPosition(currentClawPose[2]);
    }

    public void defaultPose() {
        /*clawPose = 0;
        updateClawPose();
        clawOpen = false;
        updateClawOpener();*/
    }

    public void update() {
        updateClawPose();
        updateClawOpener();
    }

}
