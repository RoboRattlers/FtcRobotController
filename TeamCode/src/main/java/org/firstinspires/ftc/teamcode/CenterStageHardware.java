package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.modulo;

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

    private TargetFollower armExtenderFollower = new TargetFollower(6, 10, 0.01); // 0.9 = 90 degrees
    private TargetFollower armRotatorFollower = new TargetFollower(1.25, 10, 0.01); // 4 = full extension
    private TargetFollower clawShoulderFollower = new TargetFollower(10, 10, 0.025); // normal servo parameters
    private TargetFollower clawElbowFollower = new TargetFollower(5, 10, 0.025);
    private TargetFollower clawWristFollower = new TargetFollower(5, 10, 0.025);

    public static double POKER_ADD_WRIST_POS = 0.4;
    public static double POKER_ADD_ELBOW_POS = 0.3;
    public static double ARM_UP_ENCODER_POS = 800;
    public static double POKER_PUSHER_PULLED_POS = 0.265;
    public static double POKER_PUSHER_PUSHED_POS = 0.16;

    private double[][] clawPoses = new double[][]{
            new double[]{0.62, 0.5, 0.5}, // initialization/drive
            new double[]{0.62, 0.725, 0.27}, // grab
            //new double[]{0.5, 0.5, 0.5}, // place onto board
            new double[]{0.62, POKER_ADD_ELBOW_POS, POKER_ADD_WRIST_POS}, // pre-push onto poker
            new double[]{0.8, POKER_ADD_ELBOW_POS, POKER_ADD_WRIST_POS} // push onto poker
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

        pokerPusher.setPosition(0.265);
        pokerRotator.setPosition(0.5);

        armExtender.setTargetPosition(0);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(1);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator.setTargetPosition(0);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setPower(1);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armRotator.setVelocityPIDFCoefficients(1, 0, 0, 0);
        //armRotator.set
    }

    public void setArmRotation(int pos) {
        armRotatorFollower.setTargetPosition(pos);
    }

    public void drivePose() {
        clawPose = 0;
        armRotatorFollower.setTargetPosition(0);
        pokerRotator.setPosition(0);
    }

    public void prepareForGrab() {
        clawPose = 1;
        clawOpen = false;
        armRotator.setTargetPosition(500);
        pokerRotator.setPosition(0.5);
        pokerPusher.setPosition(POKER_PUSHER_PULLED_POS);
    }

    public void prepareForPush() {
        armExtenderFollower.setTargetPosition(0);
        pokerPusher.setPosition(POKER_PUSHER_PULLED_POS);
    }

    public void prepareForArmPlace() {
        armExtenderFollower.setTargetPosition(1);
    }

    public void prepareForClawPlace() {
        armRotatorFollower.setTargetPosition(0.2);
        pokerRotator.setPosition(0.5);
        clawPose = 2;
    }

    public void incrementClawPose(int amnt) {
        clawPose += amnt;
        clawPose = (int) modulo(clawPose, clawPoses.length);
    }

    public void updateClawOpener() {
        clawOpener.setPosition(clawOpen ? 0.75 : 0.4);
    }

    public void toggleClawOpen() {
        clawOpen = !clawOpen;
    }

    public void updateClawPose() {
        double[] currentClawPose = clawPoses[clawPose];
        clawShoulderFollower.update();
        clawElbowFollower.update();
        clawWristFollower.update();
        clawShoulderFollower.setTargetPosition(currentClawPose[0]);
        clawWristFollower.setTargetPosition(currentClawPose[1]);
        clawElbowFollower.setTargetPosition(currentClawPose[2]);
        clawShoulder.setPosition(clawShoulderFollower.getTargetPosition());
        clawElbow.setPosition(clawElbowFollower.getTargetPosition());
        clawWrist.setPosition(clawWristFollower.getTargetPosition());
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
        armRotatorFollower.update();
        armExtenderFollower.update();
        armExtender.setTargetPosition((int) (clamp(armExtenderFollower.getCurrentPosition() * -2000, -8000, 0)));
        armRotator.setTargetPosition((int) (armRotatorFollower.getCurrentPosition() * ARM_UP_ENCODER_POS));
    }

}
