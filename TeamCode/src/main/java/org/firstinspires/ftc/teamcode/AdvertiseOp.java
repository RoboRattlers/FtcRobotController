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

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.function.Supplier;

@TeleOp(name = "Addy", group = "Linear Opmode")
public class AdvertiseOp extends BaseAutoOp {

    // rotation stuff
    private int phase = 0;
    private double INITIAL_MAX_SPEED_MULT = 0.6;
    private double MAX_SPEED_MULT = INITIAL_MAX_SPEED_MULT;
    private double INITIAL_MAX_ACCEL_TIME = 0.5;
    private double MAX_ACCEL_TIME = INITIAL_MAX_ACCEL_TIME;
    private final double IN_TO_MM = 25.4;
    private final double FIELD_SIZE = 141.345 * IN_TO_MM;
    private final double TILE_SIZE = FIELD_SIZE/6.0;
    private final double SLOW_BEGIN_THRESHOLD = 3 * IN_TO_MM;
    private final double PHASE_CHANGE_THRESHOLD = 0.35 * IN_TO_MM;
    private final double ROTATION_PHASE_CHANGE_THRESHOLD = Math.toRadians(1.0);
    private VectorF desiredDisplacement = displacement;

    Supplier<Boolean> MovementPhaseCheck = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        return diff.magnitude() < PHASE_CHANGE_THRESHOLD;
    };

    Supplier<Boolean> RotationPhaseCheck = () -> {
        return Math.abs(targetRotation - rotation) < ROTATION_PHASE_CHANGE_THRESHOLD;
    };

    Runnable MovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                    * Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD // ending deceleration;
                    * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) Math.max(speedMult, 0.075));
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable InitialMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                    //* Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD // ending deceleration;
                    * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable IntermediateMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable EndMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };


    // opencv
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    ColorDetectionPipeline colorDetectionPipeline;
    final float DECIMATION_LOW = 2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    public double[] coneColor = new double[]{0, 0, 255};
    //int zone = -1;

    // variable params
    float leftDst = (float) (-TILE_SIZE * 1.0);
    float fwdDst = (float) (-TILE_SIZE * 2.0);
    boolean placeFirstConeImmediately = false;
    int reps = 2;

    // dist: 62.5 inches

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void autoOpInitialize() {

        // detect zone
        addPhase(() -> {
        }, () -> {
            if (runtime.seconds() % 2 > 1) {
                setTargetRotation(Math.toRadians(0));
            } else {
                setTargetRotation(Math.toRadians(30));
            }
            if (runtime.seconds() % 2 > 0.5 && runtime.seconds() % 2 < 1.5) {
                setArmStage(1);
            } else {
                setArmStage(2);
            }
        }, () -> zone != -1);

        float initialOffset = (float) (-2.5 * IN_TO_MM);
        float midLeftDst = (float) ((TILE_SIZE/2.0) * Math.signum(leftDst) * 1.0f);

    }

}

