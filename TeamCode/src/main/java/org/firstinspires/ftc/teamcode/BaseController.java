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

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.openftc.easyopencv.OpenCvCamera;

import java.io.File;
import java.io.IOException;

public class BaseController extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public double encoderTicksPerRevolution = 537.7;

    public final double IN_TO_MM = 25.4;
    public final double FIELD_SIZE = 141.345 * IN_TO_MM;
    public final double TILE_SIZE = FIELD_SIZE/6.0;

    // movement stuff
    public SampleMecanumDrive drive;
    public StandardTrackingWheelLocalizer localizer;

    // rotation stuff
    public final double RIGHT_ANGLE = Math.PI/2.0;

    private double lastTick = 0.0;
    public double deltaTime = 0.0;

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

    public void baseInitialize() {

        drive = new SampleMecanumDrive(hardwareMap);
        localizer = drive.getLocalizer();

        // INITIALIZATION TELEMETRY
        {
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



        // OTHER TELEMETRY AND POST-CALCULATION STUFF
        {
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Local Movement Vector", movementVector);
            //telemetry.addData("Local Displacement from Motor Encoders", displacement);
            telemetry.addData("Rotation", Math.toDegrees(rotation));
            telemetry.addData("Target Rotation", Math.toDegrees(targetRotation));
        }
    }

    @Override
    public void runOpMode() {

    }
}
