package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CenterStageHardware {
    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo clawWrist;
    public Servo pokerPusher;
    public DcMotorEx armRotator;
    public Servo pokerRotator;
    public DcMotorEx armExtender;
    public Servo clawOpener;
    private HardwareMap hardwareMap;

    private TargetFollower armExtenderFollower = new TargetFollower(6, 10, 0.01); // 0.9 = 90 degrees
    private TargetFollower armRotatorFollower = new TargetFollower(1.25, 10, 0.01); // 4 = full extension
    private TargetFollower clawShoulderFollower = new TargetFollower(10, 10, 0.025); // normal servo parameters
    private TargetFollower clawElbowFollower = new TargetFollower(5, 10, 0.025);
    private TargetFollower clawWristFollower = new TargetFollower(5, 10, 0.025);
    private double[] pusherPositions = new double[]{0.29, 0.18, 0.16};
    public int pusherPos = 0;
    public int clawPose = 0;

    private double[][] clawPoses = new double[][]{
            new double[]{0.5, 0.15, 0.55}, // drive
            new double[]{0.62, 0.64, 0.35}, // grab
            new double[]{0.66, 0.08, 0.76}, // pre-push onto poker
            new double[]{0.8, 0.14, 0.65}, // push onto poker
    };

    public boolean clawOpen = false;
    public boolean smoothArmExtension = true;

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
    }

    public void setTargetArmRotation(double pos) {
        armRotatorFollower.setTargetPosition(clamp(pos, 0, 1300));
    }
    public void setTargetArmExtension(double pos) {
        armExtenderFollower.setTargetPosition(clamp(pos, -7500, 0));
    }

    public double getTargetArmExtension() {
        return armExtenderFollower.getTargetPosition();
    }

    public void setTargetPokerRotation(double pos) {
        pokerRotator.setPosition(pos);
    }

    public void initPose() {
        clawOpen = false;
        clawPose = 0;

        setTargetArmRotation(500);
        setTargetArmExtension(0);
        pokerRotator.setPosition(0.5);
        pusherPos = 0;

        update(true);
    }

    public void update(boolean instantaneous) {
        armRotatorFollower.update();
        armExtenderFollower.update();
        armExtender.setTargetPosition((int) (clamp(
                (smoothArmExtension || instantaneous) ? armExtenderFollower.getTargetPosition() : armExtenderFollower.getCurrentPosition()
                , -7500, 0)));
        armRotator.setTargetPosition((int) (instantaneous ? armRotatorFollower.getTargetPosition() : armRotatorFollower.getCurrentPosition()));
        pokerPusher.setPosition(pusherPositions[pusherPos]);

        double[] currentClawPose = clawPoses[clawPose];
        clawShoulderFollower.update();
        clawElbowFollower.update();
        clawWristFollower.update();
        clawShoulderFollower.setTargetPosition(currentClawPose[0]);
        clawElbowFollower.setTargetPosition(currentClawPose[1]);
        clawWristFollower.setTargetPosition(currentClawPose[2]);
        clawShoulder.setPosition(instantaneous ? clawShoulderFollower.getTargetPosition() : clawShoulderFollower.getCurrentPosition());
        clawElbow.setPosition(instantaneous ? clawElbowFollower.getTargetPosition() : clawElbowFollower.getCurrentPosition());
        clawWrist.setPosition(instantaneous ? clawWristFollower.getTargetPosition() : clawWristFollower.getCurrentPosition());
        clawOpener.setPosition(clawOpen ? 0.3 : 0.6);
    }

}
