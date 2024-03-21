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

import static androidx.core.math.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.util.MoreMath.map;
import static org.firstinspires.ftc.teamcode.util.MoreMath.modulo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MoreMath;

@TeleOp(name="CenterStage TeleOp", group="Linear Opmode")
@Config
public class CenterStageTeleOp extends BaseTeleOp {

    public enum DriveState {
        DRIVING,
        PRE_PLACING_PIXELS,
        PLACING_PIXELS,
        PRE_SUSPEND,
        SUSPENDING,
        UNSUSPENDING,
        LOADING_PIXELS,
        PIXEL_PUSHBOT,
        PUSH_ONTO_BACKDROP, PRE_DRONE_LAUNCH, LAUNCHING_DRONE, GRABBING_PIXELS
    }
    private CenterStageHardware gameHardware;

    private DriveState state = DriveState.DRIVING;
    private boolean switchedState = true;
    private double timeInState = 0;
    private double lastTimeInState = 0;
    private double lastStateSwitchTime = runtime.seconds();

    private int clawHeight = 0;

    public static double BOARD_ALIGNMENT_MAX_POWER_DIST = 300; //mm
    public static double BOARD_ALIGNMENT_TARGET_DIST = 194; // mm
    private double boardAlignmentTargetDistOffset = 0;
    public static double BOARD_ALIGNMENT_MAX_POWER = 0.35;
    public static double BOARD_ALIGNMENT_EXPONENT = 2;

    private double boardAlignmentMotorPowerCurve(double num) {
        return BOARD_ALIGNMENT_MAX_POWER * Math.pow(Math.abs(MoreMath.clamp(num, -1, 1)), 1/BOARD_ALIGNMENT_EXPONENT) * Math.signum(num);
    }

    public void setState(DriveState state) {
        this.state = state;
        this.switchedState = true;
        lastStateSwitchTime = runtime.seconds();
    }

    private boolean hasTurnedWhilePlacing = false;

    public void teleOpStep() {
        lastTimeInState = timeInState;
        timeInState = runtime.seconds() - lastStateSwitchTime;
        telemetry.addData("State", state.toString());

        boolean switchedStateBeforeStep = switchedState;
        telemetry.addData("Arm Extension", gameHardware.armExtender.getCurrentPosition());
        switch (state) {
            case DRIVING: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(0);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED;
                    gameHardware.setTargetPokerRotation(0);

                    gameHardware.clawPose = ClawPose.DRIVE_LOW;
                    gameHardware.clawOpen = true;
                    gameHardware.clawShelf.setPosition(0.2);

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = true;
                }
                if (timeInState > 0.5) {
                    gameHardware.clawShelf.setPosition(0.9);
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    slowMovement = !slowMovement;
                }
                telemetry.addData("Arm Extension", gameHardware.armExtender.getCurrentPosition());
                if (gameHardware.armExtender.getCurrentPosition() > -500) {
                    gameHardware.setTargetArmRotation(100);
                } else {
                    //slowMovement = true;
                }
                if (gameHardware.armRotator.getCurrentPosition() < 100) {
                    gameHardware.clawPose = ClawPose.DRIVE;
                }
                if (currentGamepadState.left_trigger > 0.5 && lastGamepadState.left_trigger <= 0.5) {
                    setState(DriveState.PRE_PLACING_PIXELS);
                } else if (currentGamepadState.left_stick_button && !lastGamepadState.left_stick_button) {
                    setState(DriveState.PRE_SUSPEND);
                } else if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                    setState(DriveState.PRE_DRONE_LAUNCH);
                } else if (currentGamepadState.dpad_down) {
                    setState(DriveState.GRABBING_PIXELS);
                }
                break;
            }
            case PRE_DRONE_LAUNCH: {
                if (switchedState) {
                    slowMovement = true;
                }
                gameHardware.setTargetArmExtension(0);
                gameHardware.setTargetArmRotation(630);
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    slowMovement = !slowMovement;
                }
                if (currentGamepadState.right_stick_button && !lastGamepadState.right_stick_button) {
                    setState(DriveState.LAUNCHING_DRONE);
                }
                if (currentGamepadState.right_trigger > 0.5) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case LAUNCHING_DRONE: {
                slowMovement = false;
                gameHardware.droneLauncher.setPosition(0.375);
                if (timeInState > 0.75) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case PRE_PLACING_PIXELS: {
                if (switchedState) {
                    gameHardware.setTargetArmRotation(1000);
                    gameHardware.setTargetArmExtension(-2000);
                    gameHardware.smoothArmExtension = false;
                    boardAlignmentTargetDistOffset = 0;

                    gameHardware.pusherPos = PusherPosition.ZERO_PIXEL;
                    gameHardware.setTargetPokerRotation(0.175);

                    gameHardware.clawPose = ClawPose.DRIVE;
                    gameHardware.clawShelf.setPosition(0.2);
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                }
                if (currentGamepadState.y) {
                    setCurrentHeadingAs(0);
                }
                if (currentGamepadState.left_trigger > 0.5 && lastGamepadState.left_trigger <= 0.5) {
                    setState(DriveState.PLACING_PIXELS);
                }
                if (currentGamepadState.right_trigger > 0.5) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case PLACING_PIXELS: {
                if (switchedState) {
                    gameHardware.setTargetArmRotation(1600);
                    gameHardware.smoothArmExtension = false;

                    if (gameHardware.pusherPos == PusherPosition.FULLY_RETRACTED) {
                        gameHardware.pusherPos = PusherPosition.ZERO_PIXEL;
                        boardAlignmentTargetDistOffset = 0;
                    } else {
                        boardAlignmentTargetDistOffset = 20;
                    }

                    gameHardware.setTargetPokerRotation(0.175);

                    gameHardware.clawPose = ClawPose.DRIVE;
                    gameHardware.clawShelf.setPosition(0.2);
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = false;
                    useSmoothMovement = false;
                    hasTurnedWhilePlacing = false;
                }
                double distanceToBackboard = gameHardware.backboardDistanceSensor.getDistance(DistanceUnit.MM);
                double targetDistDiff = BOARD_ALIGNMENT_TARGET_DIST + boardAlignmentTargetDistOffset - distanceToBackboard;
                double alignmentPower = boardAlignmentMotorPowerCurve(
                        targetDistDiff/BOARD_ALIGNMENT_MAX_POWER_DIST
                );
                if (currentGamepadState.y) {
                    setCurrentHeadingAs(0);
                    hasTurnedWhilePlacing = false;
                }
                hasTurnedWhilePlacing = Math.abs(currentGamepadState.right_stick_x) > 0.1 || hasTurnedWhilePlacing;
                if (!hasTurnedWhilePlacing) {
                    setTargetHeading(0);
                    applyTargetHeading();
                } else {
                    setTurnVelocity(map(Math.abs(currentGamepadState.right_stick_x), 0.1, 1, 0, 0.4, true)
                    * -Math.signum(currentGamepadState.right_stick_x));
                }
                double BACKBOARD_OFFSET_MAX_POWER_THRESHOLD = 20;
                double xMoveFactor = clamp(
                        (currentGamepadState.dpad_left ? -1 : currentGamepadState.dpad_right ? 1 : 0)
                        + map(Math.abs(currentGamepadState.left_stick_x), 0.1, 1, 0, 1, true)
                        * Math.signum(currentGamepadState.left_stick_x),
                        -1, 1);
                double yMoveFactor = clamp(
                        (currentGamepadState.dpad_down ? -1 : currentGamepadState.dpad_up ? 1 : 0)
                                + map(Math.abs(currentGamepadState.left_stick_y), 0.1, 1, 0, 1, true)
                        * -Math.signum(currentGamepadState.left_stick_y),
                        -1, 1);
                if (Math.abs(yMoveFactor) > 0) {
                    if (currentGamepadState.left_trigger > 0.5) {
                        boardAlignmentTargetDistOffset += 200 * deltaTime * -yMoveFactor;
                        if (targetDistDiff * yMoveFactor > 0) {
                            alignmentPower += map(targetDistDiff, -BACKBOARD_OFFSET_MAX_POWER_THRESHOLD, BACKBOARD_OFFSET_MAX_POWER_THRESHOLD, -0.2, 0.2, true);
                        }
                        boardAlignmentTargetDistOffset = clamp(boardAlignmentTargetDistOffset, -80, 30);
                    } else {
                        gameHardware.setTargetArmExtension(yMoveFactor > 0 ? -7000 : 0);
                        gameHardware.armExtender.setPower(Math.abs(yMoveFactor));
                    }
                } else {
                    gameHardware.setTargetArmExtension(
                            gameHardware.armExtender.getCurrentPosition()
                    );
                }
                setMoveDir(
                        new Vector2d(
                                alignmentPower,
                                xMoveFactor * 0.5
                        )
                        , CoordinateSystem.ROBOT
                );
                if (currentGamepadState.right_stick_button && !lastGamepadState.right_stick_button && gameHardware.pusherPos != PusherPosition.TWO_PIXEL) {
                    setState(DriveState.PUSH_ONTO_BACKDROP);
                }
                if (currentGamepadState.right_trigger > 0.5) {
                    slowMovement = true;
                    gameHardware.armExtender.setPower(1);
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case PUSH_ONTO_BACKDROP: {
                setMoveDir(new Vector2d(-0.2, 0), CoordinateSystem.ROBOT);
                if (currentGamepadState.right_stick_button && !lastGamepadState.right_stick_button) {
                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED; // tricks the thing into moving it to two
                }
                if (timeInState > 0.2 && lastTimeInState <= 0.2) {
                    gameHardware.pusherPos = gameHardware.pusherPos == PusherPosition.ZERO_PIXEL
                            ? PusherPosition.ONE_PIXEL
                            : PusherPosition.TWO_PIXEL;
                }
                if (timeInState > 0.835) {
                    gameHardware.pusherPos = PusherPosition.SAFE_ONE_PIXEL;
                    setState(DriveState.PLACING_PIXELS);
                }
                break;
            }
            case PRE_SUSPEND: {
                if (switchedState) {
                    gameHardware.setTargetArmRotation(1062);
                    gameHardware.setTargetArmExtension(-5000);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.setTargetPokerRotation(0.5);
                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED;

                    gameHardware.clawPose = ClawPose.DRIVE_LOW;
                    gameHardware.clawOpen = true;
                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                    slowMovement = true;
                }
                if (currentGamepadState.left_stick_button  && !lastGamepadState.left_stick_button) {
                    setState(DriveState.SUSPENDING);
                }
                break;
            }
            case SUSPENDING: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(-1000);
                    gameHardware.armExtender.setPower(0.5);
                    gameHardware.clawPose = ClawPose.DRIVE_LOW;
                    useBaseTeleOpMovement = false;
                }
                if (currentGamepadState.left_stick_button  && !lastGamepadState.left_stick_button) {
                    setState(DriveState.UNSUSPENDING);
                }
                break;
            }
            case UNSUSPENDING: {
                if (switchedState) {
                    gameHardware.armExtender.setPower(1);
                    gameHardware.setTargetArmExtension(-5000);
                    gameHardware.setTargetArmRotation(1062);
                    useBaseTeleOpMovement = true;
                    slowMovement = true;
                }
                if (currentGamepadState.left_stick_button  && !lastGamepadState.left_stick_button) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case GRABBING_PIXELS: {
                if (switchedState) {
                    clawHeight = 0;
                    gameHardware.setIKPokerEndpoint(new Pose2d(350, 100, -Math.PI));
                    gameHardware.smoothArmExtension = true;

                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED;

                    gameHardware.clawOpen = true;
                    gameHardware.clawShelf.setPosition(0);

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                    slowMovement = true;
                }
                clawHeight += currentGamepadState.right_bumper && !lastGamepadState.right_bumper ? 1
                        : currentGamepadState.left_bumper && !lastGamepadState.left_bumper ? -1
                        : 0;
                clawHeight = clamp(clawHeight, 0, 4);
                gameHardware.clawPose = clawHeight == 0 ? ClawPose.GRAB
                        : CenterStageHardware.getClawPoseForHeight(clawHeight * 25.4/2 + 4);
                if (currentGamepadState.x && !lastGamepadState.x) {
                    gameHardware.clawOpen = !gameHardware.clawOpen;
                }
                if (currentGamepadState.right_trigger > 0.5 && lastGamepadState.right_trigger <= 0.5 && !gameHardware.clawOpen) {
                    setState(DriveState.LOADING_PIXELS);
                }
                if (currentGamepadState.left_trigger > 0.5 && lastGamepadState.left_trigger <= 0.5) {
                    setState(DriveState.DRIVING);
                }
                if (currentGamepadState.dpad_up && !lastGamepadState.dpad_up && !gameHardware.clawOpen) {
                    setState(DriveState.PIXEL_PUSHBOT);
                }
                break;
            }
            case PIXEL_PUSHBOT: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(0);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED;
                    gameHardware.setTargetPokerRotation(0.05);

                    gameHardware.clawPose = ClawPose.DRIVE_LOW;
                    gameHardware.clawOpen = false;
                    gameHardware.clawShelf.setPosition(0.9);

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = true;
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    slowMovement = !slowMovement;
                }
                if (currentGamepadState.dpad_down && !lastGamepadState.dpad_down && !gameHardware.clawOpen) {
                    setState(DriveState.GRABBING_PIXELS);
                }
                if (currentGamepadState.right_trigger > 0.5 && lastGamepadState.right_trigger <= 0.5 && !gameHardware.clawOpen) {
                    setState(DriveState.LOADING_PIXELS);
                }
                break;
            }
            case LOADING_PIXELS: {
                if (switchedState) {

                    gameHardware.pusherPos = PusherPosition.FULLY_RETRACTED;
                    slowMovement = true;
                    gameHardware.clawPose = ClawPose.LOAD;
                    gameHardware.clawShelf.setPosition(0.8);
                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                    slowMovement = true;
                }
                double plungeTime = 0.7;
                double plungeLength = 0.7;
                double plungeAftertime = 0.8;
                if (timeInState > plungeTime + plungeLength + plungeAftertime) {
                    gameHardware.clawOpen = true;
                    setState(DriveState.DRIVING);
                }

                if (timeInState > plungeTime) {
                    if (currentGamepadState.right_trigger > 0.5) {
                        gameHardware.clawOpen = true;
                        setState(DriveState.DRIVING);
                    }
                    gameHardware.clawShelf.setPosition(0.9);
                    gameHardware.setIKPokerEndpoint(
                            new Pose2d(
                                    map(timeInState, plungeTime, plungeTime + plungeLength, 375, 400, true),
                            map(timeInState, plungeTime, plungeTime + plungeLength, 75, -100, true),
                            -Math.PI)
                    );
                } else if (timeInState > 0.35) {
                    gameHardware.clawShelf.setPosition(0.9);
                } else {
                    gameHardware.setIKPokerEndpoint(new Pose2d(375, 75, -Math.PI));
                }
                break;
            }
        }
        if (switchedStateBeforeStep) {
            switchedState = false;
        }
        gameHardware.update(false);
    }

    public void teleOpInitialize() {
        gameHardware = new CenterStageHardware(hardwareMap, telemetry);
        gameHardware.initPose();
    }

}
