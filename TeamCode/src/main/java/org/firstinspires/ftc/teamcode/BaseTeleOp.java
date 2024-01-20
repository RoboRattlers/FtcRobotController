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

import static org.firstinspires.ftc.teamcode.util.MoreMath.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Size;

@TeleOp(name="Base TeleOp", group="Linear Opmode")
public class BaseTeleOp extends BaseController {

    public Gamepad lastGamepadState = new Gamepad();
    public Gamepad currentGamepadState = new Gamepad();
    public Gamepad lastGamepad2State = new Gamepad();
    public Gamepad currentGamepad2State = new Gamepad();
    private String CONTROL_STRING = "!!!! MOVEMENT CONTROLS: !!!!"
            + "\nLeft stick to move/strafe"
            + "\nBumpers to rotate 90 degrees"
            + "\nA to enter free movement mode"
            + "\nRight stick to rotate in free movement mode"
            + "\nY to calibrate orientation";

    // movement stuff
    public boolean freeMovement = true;
    private double freeMoveSpeed = 0.8; // multiplier for strafing speeds in "free movement" mode
    private double freeTurnSpeed = 0.8; // multiplier for turning speeds in "free movement" mode

    public void teleOpStep() {
    }

    public void teleOpInitialize() {
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Controls", CONTROL_STRING);
        baseInitialize();
        teleOpInitialize();

        waitForStart();
        runtime.reset();

        // MAIN LOOP
        while (opModeIsActive()) {

            lastGamepadState.copy(currentGamepadState);
            currentGamepadState.copy(gamepad1);
            lastGamepad2State.copy(currentGamepad2State);
            currentGamepad2State.copy(gamepad2);

            telemetry.addData("Controls", CONTROL_STRING);

            // MOVEMENT HANDLING
            {
                Vector2d rawMoveVector;
                double xMoveFactor = -currentGamepadState.left_stick_y
                        + (currentGamepadState.dpad_right ? 1 : 0)
                        - (currentGamepadState.dpad_left ? 1 : 0);
                double yMoveFactor = -currentGamepadState.left_stick_x
                        + (currentGamepadState.dpad_down ? 1 : 0)
                        - (currentGamepadState.dpad_up ? 1 : 0);
                if (Math.abs(xMoveFactor) + Math.abs(yMoveFactor) > 0.05) {
                    double magnitude = Math.sqrt(Math.pow(currentGamepadState.left_stick_y, 2) + Math.pow(currentGamepadState.left_stick_x, 2));
                    magnitude = Math.min(magnitude, 1.0);
                    double angle = Math.atan2(yMoveFactor, xMoveFactor);
                    double newAngle = round(angle, RIGHT_ANGLE/2.0);
                    rawMoveVector = new Vector2d(Math.cos(newAngle) * magnitude, Math.sin(newAngle) * magnitude);
                } else {
                    rawMoveVector = new Vector2d();
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    freeMovement = !freeMovement;
                }
                if (!freeMovement) {
                    // turning
                    double targetHeading = getTargetHeading();
                    double nearestRightAngle = round(targetHeading, RIGHT_ANGLE);
                    double targetRotationDiff = targetHeading - nearestRightAngle;
                    if (Math.abs(targetRotationDiff) < 0.01) {
                        if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                            setTargetHeading(round(targetHeading, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                            setTargetHeading(targetHeading + RIGHT_ANGLE);
                        }
                        if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                            setTargetHeading(round(targetHeading, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                            setTargetHeading(targetHeading - RIGHT_ANGLE);
                        }
                    } else {
                        if (normalizeAngle(targetHeading - nearestRightAngle, AngleUnit.RADIANS) < 0.0) {
                            if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                                setTargetHeading(nearestRightAngle);
                            }
                            if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                                setTargetHeading(nearestRightAngle - RIGHT_ANGLE);
                            }
                        } else {
                            if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                                setTargetHeading(nearestRightAngle + RIGHT_ANGLE);
                            }
                            if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                                setTargetHeading(nearestRightAngle);
                            }
                        }
                    }
                    // application
                    if (rawMoveVector.distTo(new Vector2d(0, 0)) < 0.01) {
                        setMoveDir(new Vector2d(), CoordinateSystem.WORLD);
                    } else {
                        setMoveDir(rawMoveVector, CoordinateSystem.TARGET_HEADING);
                    }
                    applyTargetHeading();
                } else { // free movement
                    setMoveDir(rawMoveVector, CoordinateSystem.ROBOT);
                    setTargetHeading(localizer.getPoseEstimate().getHeading());
                    setTurnVelocity(freeTurnSpeed * -currentGamepadState.right_stick_x);
                }
            }

            teleOpStep();
            baseUpdate();

            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                telemetry.addData("Current Gamepad", currentGamepadState.toString());
                telemetry.addData("Last Gamepad", lastGamepadState.toString());
                telemetry.update();
            }
        }
    }}
