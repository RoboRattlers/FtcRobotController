/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;

/* TODO:
- Add precise/slow mode
- Make rotation in 90 degree increments in fast mode
- Make an actually good acceleration integrator to calculate position based on IMU readings
- Make something to identify the bot's location using cameras
    - Use edge detection to identify borders of the images set up around the field?
- Use TensorFlow Object Detection to identify cones
*/

// android studio test
// This class operates the wheels by interfacing with the gamepad and calculates motion/location data for the drive train.
@TeleOp(name="Old TeleOp (in case I messed something up horribly)", group="Linear Opmode")
public class TeleOp_Old extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Gamepad lastGamepadState = new Gamepad();
    private Gamepad currentGamepadState = new Gamepad();
    private double encoderTicksPerRevolution = 537.7;
    private String CONTROL_STRING = "Left stick to move/strafe"
            + "\nX to open or close claw"
            + "\nRight trigger to raise arm, left trigger to lower arm"
            + "\nA to enter free movement mode"
            + "\nRight stick to rotate in free movement mode"
            + "\nBumpers to rotate 90 degrees"
            + "\nY to calibrate orientation against wall";

    // distance sensor stuff
    private Rev2mDistanceSensor distanceSensorR = null;
    private Rev2mDistanceSensor distanceSensorL = null;
    private double distanceBetweenSensors = 144; // mm, need to refine value
    private double WALL_CONSIDERATION_THRESHOLD = Math.toRadians(2.5); // radians duh

    // arm stuff
    private DcMotor armMotor;
    private int armMinEncoderValue = -3000;
    private int armMaxEncoderValue = 0; // for now, change it once we get a concrete value
    public int goalArmEncoderValue = 0;
    public int armMovementStepSize = -1000;
    private final double FULL_ARM_POWER_ENCODER_TICK_THRESHOLD = 20.0;

    // claw stuff
    private Servo clawServoL;
    private Servo clawServoR;
    private double openClawPosL = 0.8 + 0.05;
    private double openClawPosR = 0.275 - 0.05;
    private double closedClawPosL = 0.575;
    private double closedClawPosR = 0.5;
    public boolean clawOpen = false;

    // wheel stuff
    private DcMotor[] wheelMap; // list of the wheel DcMotors
    private double[] wheelPowers = {0, 0, 0, 0}; // the final powers that are applied to the wheels
    private double[] axialMovementMap = {1, 1, 1, 1}; // base wheel powers required for axial (i.e. forward/backward) movement
    private double[] lateralMovementMap = {1, -1, -1, 1}; // base wheel powers required for lateral (i.e. side to side) movement
    private double[] turnMap = {-1, -1, 1, 1}; // base wheel powers required for turning
    private double wheelDiameter = 100.0; // millimeters
    private double wheelDistancePerEncoderTick = (100.0 * Math.PI)/encoderTicksPerRevolution;

    // movement stuff
    private boolean freeMovement = false;
    private VectorF movementVector = new VectorF(0, 0, 0, 1); // movement vector is in the bot's local space
    public VectorF displacementVector = new VectorF(0, 0, 0, 0);
    private double freeMoveSpeed = 0.25; // multiplier for strafing speeds in "free movement" mode
    private double freeTurnSpeed = 0.25; // multiplier for turning speeds in "free movement" mode
    private double masterPower = 0.9; // master multiplier of wheel powers
    private int[] lastWheelEncoders = {0, 0, 0, 0};
    private int[] currentWheelEncoders = {0, 0, 0, 0};

    // rotation stuff
    private final double RIGHT_ANGLE = Math.PI/2.0;
    private BNO055IMU imu;
    private OpenGLMatrix rotationMatrix = OpenGLMatrix.identityMatrix();
    private OpenGLMatrix targetRotationMatrix = OpenGLMatrix.identityMatrix();
    private double referenceRotation = 0.0; // frame of reference rotation, used to ensure that right-angle increments are aligned with the field
    private double rotation = 0.0; // rotation of the bot as compared to the reference rotation
    private double targetRotation = 0.0; // target for "rotation" variable, achieved by turning the bot
    private double rotationDampeningThreshold = RIGHT_ANGLE * (60.0/90.0); // threshold before the motors begin to lessen their power
    private double rotationPower = 0.9; // multiplier for rotation speed
    private double turnVelocity = 0.0; //

    private double lastTick = 0.0;
    private double deltaTime = 0.0;

    // generic math functions
    public double modulo(double a, double b) {
        return  (a % b + b) % b;
    }

    public double normalizeAngle(double angle, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            return modulo((angle + 180), 360) - 180;
        } else {
            return modulo((angle + Math.PI), 2 * Math.PI) - Math.PI;
        }
    }

    public double lerp(double a, double b, double alpha) {
        return a + (b - a) * alpha;
    }

    public double roundToNearest(double num, double interval) {
        return (Math.round(num/interval) * interval);
    }

    // yeah bro
    public void applyMovement() {
        float axialMovement = movementVector.get(1);
        float lateralMovement = movementVector.get(0);
        double maxPowerMagnitude = 1;
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = (axialMovementMap[i] * axialMovement + lateralMovementMap[i] * lateralMovement + turnMap[i] * turnVelocity);
            maxPowerMagnitude = Math.max(maxPowerMagnitude, wheelPowers[i]);
        }
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = wheelPowers[i]/maxPowerMagnitude;
            wheelMap[i].setPower(wheelPowers[i] * masterPower);
        }
    }

    // rotation
    
    private void updateRotationData() {
        Orientation rawOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        rotation = normalizeAngle(rawOrientation.firstAngle - referenceRotation, AngleUnit.RADIANS);
        Orientation currentOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) rotation, 0, 0, 0);
        rotationMatrix = currentOrientation.getRotationMatrix();
    }

    public void setTurnVelocity(double vel) {
        turnVelocity = vel;
    }

    public void applyTargetRotation() {
        float turnDiff = (float) normalizeAngle(targetRotation - rotation, AngleUnit.RADIANS);
        telemetry.addData("turn diff", turnDiff);
        setTurnVelocity((float) (Math.max(Math.min(turnDiff, rotationDampeningThreshold), -rotationDampeningThreshold)/rotationDampeningThreshold * rotationPower));
    }

    public void setReferenceRotation(double val) {
        targetRotation = targetRotation - referenceRotation;
        referenceRotation = val;
        setTargetRotation(referenceRotation + targetRotation);
        updateRotationData();
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
        targetRotation = normalizeAngle(targetRotation, AngleUnit.RADIANS);
        Orientation targetOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) targetRotation, 0, 0, 0);
        targetRotationMatrix = targetOrientation.getRotationMatrix();
    }
    

    // movement/strafing
    public void setWorldMovementVector(VectorF vector) {
        movementVector = rotationMatrix.multiplied(vector);
        applyMovement();
    }

    public void setMovementVectorRelativeToTargetOrientation(VectorF vector) {
        vector = targetRotationMatrix.inverted().multiplied(vector);
        setWorldMovementVector(vector);
    }

    public void setLocalMovementVector(VectorF vector) {
        movementVector = vector;
        applyMovement();
    }

    public VectorF getWorldMovementVector() {
        return new VectorF(0, 0, 0, 1);
    }

    public VectorF getLocalMovementVector() {
        return movementVector;
    }

    @Override
    public void runOpMode() {

        // Update control hub firmware to Firmware version 1.8.2

        // DISTANCE SENSOR SETUP
        {
            distanceSensorR = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceR");
            distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceL");
        }

        // WHEEL SETUP
        {
            wheelMap = new DcMotor[]{
                    hardwareMap.get(DcMotor.class, "TopRightWheel"),
                    hardwareMap.get(DcMotor.class, "BottomRightWheel"),
                    hardwareMap.get(DcMotor.class, "TopLeftWheel"),
                    hardwareMap.get(DcMotor.class, "BottomLeftWheel")
            };

            for (int i = 0; i < 4; i++) {
                DcMotor motor = wheelMap[i];
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            wheelMap[0].setDirection(DcMotor.Direction.REVERSE);
            wheelMap[1].setDirection(DcMotor.Direction.REVERSE);
            wheelMap[2].setDirection(DcMotor.Direction.FORWARD);
            wheelMap[3].setDirection(DcMotor.Direction.FORWARD);
        }

        // IMU SETUP
        {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.mode = BNO055IMU.SensorMode.NDOF;
            // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);
        }

        // ARM SETUP

        {
            armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Arm Status", "Reset arm motor encoder.");
            armMotor.setTargetPosition(goalArmEncoderValue);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // CLAW SETUP
        {
            clawServoL = hardwareMap.get(Servo.class, "ClawL");
            clawServoR = hardwareMap.get(Servo.class, "ClawR");
            clawServoL.setPosition(openClawPosL);
            clawServoR.setPosition(openClawPosR);
        }

        // INITIALIZATION TELEMETRY
        {
            telemetry.addData("Gyroscope Status", imu.isGyroCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Magnetometer Status", imu.isMagnetometerCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Accelerometer Status", imu.isAccelerometerCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Status", "Initialized.");

            // load configuration
            try {
                File file = AppUtil.getInstance().getSettingsFile("DriveTrainConfig.json");
                String serializedConfig = ReadWriteFile.readFileOrThrow(file);
                telemetry.addData("Got config:", serializedConfig);
            } catch (IOException error) {
                telemetry.addData("Error while loading config: ", error.getMessage());
            }

            telemetry.addData("Game", "Press START to run >>");
            telemetry.update();
        }



        waitForStart();
        runtime.reset();

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // MAIN LOOP
        while (opModeIsActive()) {

            deltaTime = runtime.seconds() - lastTick;
            lastTick = runtime.seconds();

            telemetry.addData("Controls", CONTROL_STRING);

            // MOVEMENT DELTA CALCULATIONS
            {
                System.arraycopy(currentWheelEncoders, 0, lastWheelEncoders, 0, 4);

                for (int i = 0; i < 4; i++) {
                    currentWheelEncoders[i] = wheelMap[i].getCurrentPosition();
                }

                /*
                tr = lateral - strafe - turn
                br = lateral + strafe - turn
                tl = lateral + strafe + turn
                bl = lateral - strafe + turn

                tr - br = (lateral - strafe - turn) - (lateral + strafe - turn)
                = lateral - lateral - strafe - strafe - turn + turn
                = -2strafe

                tr - bl = (lateral - strafe - turn) - (lateral - strafe + turn)
                = lateral - strafe - turn - lateral + strafe - turn
                = lateral - lateral + strafe - strafe - turn - turn
                = 0 + 0 + -2turn

                lateral = tr + tl = br + bl
                strafe = (tl - bl)/2 = (br - tr)/2
                turn = (tl - br)/2 = (bl - tr)/2
                 */

                double deltaTR = (double) currentWheelEncoders[0] - lastWheelEncoders[0];
                double deltaBR = (double) currentWheelEncoders[1] - lastWheelEncoders[1];
                double deltaTL = (double) currentWheelEncoders[2] - lastWheelEncoders[2];
                double deltaBL = (double) currentWheelEncoders[3] - lastWheelEncoders[3];

                double forwardTicks = (deltaTR + deltaTL + deltaBR + deltaBL) / 2.0; // avg of the two formulas
                double strafeTicks = (deltaTL - deltaBL + deltaBR - deltaTR) / 4.0; // avg of the two formulas
                double turnTicks = (deltaTL - deltaBR + deltaBL - deltaTR) / 4.0; // avg of the two formulas

                double forwardDisplacement = forwardTicks * wheelDistancePerEncoderTick;
                double sidewaysDisplacement = strafeTicks * wheelDistancePerEncoderTick;
                displacementVector = new VectorF(
                        displacementVector.get(0) + ((float) sidewaysDisplacement),
                        displacementVector.get(1) + ((float) forwardDisplacement), 0, 0);
            }

            // ROTATION CALCULATIONS
            {
                updateRotationData();
                setTargetRotation(normalizeAngle(targetRotation, AngleUnit.RADIANS)); // normalize the target rotation
            }

            // MOVEMENT HANDLING
            {
                VectorF rawMoveVector;
                double xMoveFactor = currentGamepadState.left_stick_x
                        + (currentGamepadState.dpad_right ? 1 : 0)
                        - (currentGamepadState.dpad_left ? 1 : 0);
                double yMoveFactor = currentGamepadState.left_stick_y
                        + (currentGamepadState.dpad_down ? 1 : 0)
                        - (currentGamepadState.dpad_up ? 1 : 0);
                if (Math.abs(xMoveFactor) + Math.abs(yMoveFactor) > 0.05) {
                    double magnitude = Math.sqrt(Math.pow(currentGamepadState.left_stick_x, 2) + Math.pow(currentGamepadState.left_stick_y, 2));
                    magnitude = Math.min(magnitude, 1.0);
                    double angle = Math.atan2(currentGamepadState.left_stick_y, currentGamepadState.left_stick_x);
                    double newAngle = roundToNearest(angle, RIGHT_ANGLE/2.0);
                    rawMoveVector = new VectorF((float) (Math.cos(newAngle) * magnitude), (float) (Math.sin(newAngle) * magnitude), 0, 1);
                } else {
                    rawMoveVector = new VectorF(0, 0, 0, 1);
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    freeMovement = !freeMovement;
                }
                if (!freeMovement) {
                    // turning
                    if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                        setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                        setTargetRotation(targetRotation + RIGHT_ANGLE);
                    }
                    if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                        setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                        setTargetRotation(targetRotation - RIGHT_ANGLE);
                    }
                    // application
                    if (rawMoveVector.magnitude() < 0.01) {
                        applyTargetRotation();
                        setLocalMovementVector(new VectorF(0, 0, 0, 1));
                    } else {
                        setTurnVelocity(0);
                        setLocalMovementVector(rawMoveVector);
                    }

                } else { // free movement
                    // just a bunch of application this is ez
                    setLocalMovementVector(rawMoveVector.multiplied((float) freeMoveSpeed));
                    setTargetRotation(rotation);
                    setTurnVelocity(freeTurnSpeed * -currentGamepadState.right_stick_x);
                }
            }

            // ODOMETRY HANDLING
            {
                double distanceL = distanceSensorL.getDistance(DistanceUnit.MM);
                double distanceR = distanceSensorR.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance L", distanceL);
                telemetry.addData("Distance R", distanceR);
                double angleAgainstWall = Math.atan((distanceL - distanceR) / distanceBetweenSensors); // clockwise turn == negative angle
                telemetry.addData("Angle Against Wall", Math.toDegrees(angleAgainstWall));

                if (distanceL != Rev2mDistanceSensor.distanceOutOfRange && distanceR != Rev2mDistanceSensor.distanceOutOfRange) {
                    // TEST IF THE "ROTATION" VALUE INCREASES OR DECREASES WITH CLOCKWISE TURNS!!!
                    double angleOfWall = rotation - angleAgainstWall;
                    telemetry.addData("Angle of Wall", Math.toDegrees(angleOfWall));
                    if (currentGamepadState.y && !lastGamepadState.y) { // set this wall as the new frame of reference
                        setReferenceRotation(angleOfWall);
                        setTargetRotation(0);
                    }
                    boolean isWall = Math.abs(angleOfWall - roundToNearest(angleOfWall, RIGHT_ANGLE)) < WALL_CONSIDERATION_THRESHOLD;
                    telemetry.addData("Wall?", isWall ? "Yay" : "Nay");
                } else {
                    telemetry.addData("Wall?", "Nay");
                }
            }

            // ARM HANDLING
            {
                double armDir = currentGamepadState.right_trigger - currentGamepadState.left_trigger;
                if (currentGamepadState.right_trigger > 0.5 && lastGamepadState.right_trigger <= 0.5) {
                    goalArmEncoderValue += armMovementStepSize;
                }
                if (currentGamepadState.left_trigger > 0.5 && lastGamepadState.left_trigger <= 0.5) {
                    goalArmEncoderValue -= armMovementStepSize;
                }
                goalArmEncoderValue = Math.min(Math.max(goalArmEncoderValue, armMinEncoderValue), armMaxEncoderValue);
                armMotor.setTargetPosition(goalArmEncoderValue);
                int realArmEncoderValue = armMotor.getCurrentPosition();
                double power = Math.min(Math.abs((double) (goalArmEncoderValue - realArmEncoderValue))/FULL_ARM_POWER_ENCODER_TICK_THRESHOLD, 1);
                armMotor.setPower(power);
                telemetry.addData("Arm Goal Encoder", goalArmEncoderValue);
                telemetry.addData("Arm Actual Encoder", realArmEncoderValue);
            }

            // CLAW HANDLING
            {
                if (currentGamepadState.x && !lastGamepadState.x) {
                    clawOpen = !clawOpen;
                }
                clawServoL.setPosition(clawOpen ? openClawPosL : closedClawPosL);
                clawServoR.setPosition(clawOpen ? openClawPosR : closedClawPosR);
            }

            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                applyMovement();
                telemetry.addData("Run Time", runtime.toString());
                telemetry.addData("Local Movement Vector", movementVector);
                telemetry.addData("Local Displacement from Motor Encoders", displacementVector);
                telemetry.addData("Rotation", Math.toDegrees(rotation));
                telemetry.addData("Target Rotation", Math.toDegrees(targetRotation));
                lastGamepadState.copy(currentGamepadState);
                currentGamepadState.copy(gamepad1);
                telemetry.addData("Current Gampead", currentGamepadState.toString());
                telemetry.addData("Last Gampead", lastGamepadState.toString());
                telemetry.update();
            }
        }
    }}
