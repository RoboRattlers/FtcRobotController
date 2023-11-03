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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.File;
import java.io.IOException;

public class BaseController extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public double encoderTicksPerRevolution = 537.7;

    public final double IN_TO_MM = 25.4;
    public final double FIELD_SIZE = 141.345 * IN_TO_MM;
    public final double TILE_SIZE = FIELD_SIZE/6.0;

    // wheel stuff
    private DcMotor[] wheelMap; // list of the wheel DcMotors
    private double[] wheelPowers = {0, 0, 0, 0}; // the final powers that are applied to the wheels
    private double[] axialMovementMap = {1, 1, 1, 1}; // base wheel powers required for axial (i.e. forward/backward) movement
    private double[] lateralMovementMap = {1, -1, -1, 1}; // base wheel powers required for lateral (i.e. side to side) movement
    private double[] turnMap = {-1, -1, 1, 1}; // base wheel powers required for turning
    private double wheelDiameter = 96.0; // millimeters
    private double wheelDistancePerEncoderTick = (wheelDiameter * Math.PI)/encoderTicksPerRevolution;

    // movement stuff
    public VectorF movementVector = new VectorF(0, 0, 0, 1); // movement vector is in the bot's local space
    public VectorF displacement = new VectorF(0, 0, 0, 0);
    private OpenGLMatrix displacementMatrix = OpenGLMatrix.identityMatrix();
    private double masterPower = 0.9; // master multiplier of wheel powers
    private int[] lastWheelEncoders = {0, 0, 0, 0};
    private int[] currentWheelEncoders = {0, 0, 0, 0};

    // rotation stuff
    public final double RIGHT_ANGLE = Math.PI/2.0;
    private BNO055IMU imu;
    private OpenGLMatrix rotationMatrix = OpenGLMatrix.identityMatrix();
    private OpenGLMatrix targetRotationMatrix = OpenGLMatrix.identityMatrix();
    private double referenceRotation = 0.0; // frame of reference rotation, used to ensure that right-angle increments are aligned with the field
    public double rotation = 0.0; // rotation of the bot as compared to the reference rotation
    public double targetRotation = 0.0; // target for "rotation" variable, achieved by turning the bot
    private double rotationDampeningThreshold = RIGHT_ANGLE * (60.0/90.0); // threshold before the motors begin to lessen their power
    private double rotationPower = 0.9; // multiplier for rotation speed
    private double turnVelocity = 0.0; //
    public double angleOfWall = 0.0;

    private double lastTick = 0.0;
    public double deltaTime = 0.0;

    public OpenCvCamera camera;

    // generic math functions
    public double modulo(double a, double b) {
        return  (a % b + b) % b;
    }

    public double map(double num, double oldMin, double oldMax, double newMin, double newMax, boolean clamp) {
        double newValue = (num - oldMin)/(oldMax - oldMin) * (newMax - newMin) + newMin;
        if (clamp) {
            newValue = clamp(newValue, newMin, newMax);
        }
        return newValue;
    }

    public double clamp(double num, double min, double max) {
        double max2 = Math.max(min, max);
        double min2 = Math.min(min, max);
        return Math.min(Math.max(num, min2), max2);
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

    public double round(double num, double interval) {
        return (Math.round(num/interval) * interval);
    }

    private void applyMovement() {
        float axialMovement = movementVector.get(1);
        float lateralMovement = movementVector.get(0);
        double maxPowerMagnitude = 1; //map(realArmEncoderValue, -1300, -3000, 1, 0.5, true);
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = (axialMovementMap[i] * axialMovement + lateralMovementMap[i] * lateralMovement + turnMap[i] * turnVelocity);
            maxPowerMagnitude = Math.max(maxPowerMagnitude, wheelPowers[i]);
        }
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = wheelPowers[i]/maxPowerMagnitude;
            wheelMap[i].setPower(wheelPowers[i] * masterPower);
        }
    }

    private void updateRotationData() {
        Orientation rawOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        rotation = normalizeAngle(rawOrientation.firstAngle - referenceRotation, AngleUnit.RADIANS);
        Orientation currentOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) rotation, 0, 0, 0);
        rotationMatrix = currentOrientation.getRotationMatrix();
    }

    public void setTurnVelocity(double vel) {
        turnVelocity = vel;
    }

    private void applyTargetRotation() {
        float turnDiff = (float) normalizeAngle(targetRotation - rotation, AngleUnit.RADIANS);
        telemetry.addData("turn diff", turnDiff);
        float tvel = ((float) (Math.max(Math.min(turnDiff, rotationDampeningThreshold), -rotationDampeningThreshold)/rotationDampeningThreshold * rotationPower));
        if (Math.abs(tvel) > 0.02) {
            setTurnVelocity(tvel);
        } else {
            setTurnVelocity(0);
        }
    }

    public void setReferenceRotation(double val) {
        targetRotation = targetRotation - referenceRotation;
        referenceRotation = referenceRotation + val;
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
    }

    public void setMovementVectorRelativeToTargetOrientation(VectorF vector) {
        vector = targetRotationMatrix.inverted().multiplied(vector);
        setWorldMovementVector(vector);
    }

    public void setLocalMovementVector(VectorF vector) {
        movementVector = vector;
        //applyMovement();
    }

    public void setCurrentDisplacementAs(VectorF reference) {
        displacementMatrix = OpenGLMatrix.identityMatrix();
        displacementMatrix.translate(
                reference.get(0),
                reference.get(1),
                reference.get(2)
        );
        displacementMatrix.multiply(rotationMatrix.transposed());
        VectorF tempDisplacement = displacementMatrix.getTranslation();
        displacement = new VectorF(tempDisplacement.get(0), tempDisplacement.get(1), tempDisplacement.get(2), 0);
        //telemetry.addData("Displacement", displacementMatrix.formatAsTransform(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
    }

    public VectorF getWorldMovementVector() {
        return new VectorF(0, 0, 0, 1);
    }

    public VectorF getLocalMovementVector() {
        return movementVector;
    }

    public void baseInitialize() {

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
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    }

    public void baseUpdate() {

        deltaTime = runtime.seconds() - lastTick;
        lastTick = runtime.seconds();

        // ROTATION CALCULATIONS
        updateRotationData();

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

            lateral = (tr + tl)/2 = (br + bl))/2
            strafe = (tl - bl)/2 = (br - tr)/2
            turn = (tl - br)/2 = (bl - tr)/2
             */

            double deltaTR = (double) currentWheelEncoders[0] - lastWheelEncoders[0];
            double deltaBR = (double) currentWheelEncoders[1] - lastWheelEncoders[1];
            double deltaTL = (double) currentWheelEncoders[2] - lastWheelEncoders[2];
            double deltaBL = (double) currentWheelEncoders[3] - lastWheelEncoders[3];

            double forwardTicks = (deltaTR + deltaTL + deltaBR + deltaBL) / 4.0; // avg of the two formulas
            double strafeTicks = (deltaTL - deltaBL + deltaBR - deltaTR) / 4.0; // avg of the two formulas
            double turnTicks = (deltaTL - deltaBR + deltaBL - deltaTR) / 4.0; // avg of the two formulas

            double forwardDisplacement = forwardTicks * wheelDistancePerEncoderTick;
            double sidewaysDisplacement = strafeTicks * wheelDistancePerEncoderTick;
            VectorF displacementDelta = new VectorF((float) -sidewaysDisplacement, (float) forwardDisplacement, 0, 0);
            displacementMatrix = OpenGLMatrix.identityMatrix();
            displacementMatrix.translate(
                    displacement.get(0),
                    displacement.get(1),
                    displacement.get(2)
            );
            displacementMatrix.multiply(rotationMatrix.transposed());
            displacementMatrix.translate(
                    displacementDelta.get(0),
                    displacementDelta.get(1),
                    displacementDelta.get(2)
            );
            VectorF tempDisplacement = displacementMatrix.getTranslation();
            displacement = new VectorF(tempDisplacement.get(0), tempDisplacement.get(1), tempDisplacement.get(2), 0);
            telemetry.addData("Displacement", displacementMatrix.formatAsTransform(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
        }


        // OTHER TELEMETRY AND POST-CALCULATION STUFF
        {
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Local Movement Vector", movementVector);
            //telemetry.addData("Local Displacement from Motor Encoders", displacement);
            telemetry.addData("Rotation", Math.toDegrees(rotation));
            telemetry.addData("Target Rotation", Math.toDegrees(targetRotation));

            applyMovement();
            applyTargetRotation();
        }
    }

    @Override
    public void runOpMode() {

    }
}
