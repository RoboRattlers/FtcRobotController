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

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CenterStage TeleOp", group="Linear Opmode")
public class CenterStageTeleOp extends BaseTeleOp {

    public enum DriveState {
        DRIVING,
        PLACING_PIXELS,
        PRE_SUSPEND,
        SUSPENDING,
        UNSUSPENDING,
        LOADING_PIXELS,
        GRABBING_PIXELS
    }
    private CenterStageHardware gameHardware;

    private DriveState state = DriveState.DRIVING;
    private boolean switchedState = true;
    private double timeInState = 0;
    private double lastStateSwitchTime = runtime.seconds();

    public void setState(DriveState state) {
        this.state = state;
        switchedState = true;
        lastStateSwitchTime = runtime.seconds();
    }

    public void teleOpStep() {
        timeInState = runtime.seconds() - lastStateSwitchTime;
        switch (state) {
            case DRIVING: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(0);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.pusherPos = 0;
                    gameHardware.setTargetPokerRotation(0.1);

                    gameHardware.clawPose = 0;
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = true;
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    slowMovement = !slowMovement;
                }
                if (gameHardware.armExtender.getCurrentPosition() > -100) {
                    gameHardware.setTargetArmRotation(100);
                } else {
                    slowMovement = true;
                }
                if (currentGamepadState.left_trigger > 0.5) {
                    setState(DriveState.PLACING_PIXELS);
                } else if (currentGamepadState.left_stick_button) {
                    setState(DriveState.PRE_SUSPEND);
                } else if (currentGamepadState.dpad_down) {
                    setState(DriveState.GRABBING_PIXELS);
                }
            }
            case PLACING_PIXELS: {
                if (switchedState) {
                    gameHardware.setTargetArmRotation(1100);
                    gameHardware.setTargetArmExtension(0);
                    gameHardware.smoothArmExtension = false;

                    gameHardware.pusherPos = 0;
                    gameHardware.setTargetPokerRotation(0.3);

                    gameHardware.clawPose = 0;
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = false;
                    useSmoothMovement = false;
                }
                setMoveDir(
                        new Vector2d(
                                currentGamepadState.left_stick_y,
                                (currentGamepadState.dpad_right ? 1 : 0) - (currentGamepadState.dpad_left ? 1 : 0)
                        ).times(0.2)
                        , CoordinateSystem.ROBOT
                );
                gameHardware.setTargetArmExtension(
                        gameHardware.getTargetArmExtension()
                                - 3000 * deltaTime
                                * ((currentGamepadState.dpad_up ? 1 : 0) - (currentGamepadState.dpad_down ? 1 : 0)
                        )
                );
                if (currentGamepadState.right_stick_button && !lastGamepadState.right_stick_button) {
                    gameHardware.pusherPos = clamp(gameHardware.pusherPos + 1, 0, 2);
                }
                if (currentGamepadState.right_trigger > 0.5) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case PRE_SUSPEND: {
                if (switchedState) {
                    gameHardware.setTargetArmRotation(800);
                    gameHardware.setTargetArmExtension(-5000);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.setTargetPokerRotation(0.5);
                    gameHardware.pusherPos = 0;

                    gameHardware.clawPose = 0;
                    gameHardware.clawOpen = true;
                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                    slowMovement = true;
                }
                if (currentGamepadState.left_stick_button) {
                    setState(DriveState.SUSPENDING);
                }
                break;
            }
            case SUSPENDING: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(-1000);
                    useBaseTeleOpMovement = false;
                }
                if (currentGamepadState.left_stick_button) {
                    setState(DriveState.UNSUSPENDING);
                }
                break;
            }
            case UNSUSPENDING: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(-5000);
                    useBaseTeleOpMovement = true;
                    slowMovement = true;
                }
                if (currentGamepadState.left_stick_button) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
            case GRABBING_PIXELS: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(-400);
                    gameHardware.setTargetArmRotation(370);
                    gameHardware.smoothArmExtension = true;

                    gameHardware.pusherPos = 0;
                    gameHardware.setTargetPokerRotation(0.8);

                    gameHardware.clawPose = 1;
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = false;
                    slowMovement = true;
                }
                if (currentGamepadState.x && !lastGamepadState.x) {
                    gameHardware.clawOpen = !gameHardware.clawOpen;
                }
                if (currentGamepadState.right_trigger > 0.5 && lastGamepadState.right_trigger <= 0.5 && !gameHardware.clawOpen) {
                    setState(DriveState.LOADING_PIXELS);
                }
                break;
            }
            case LOADING_PIXELS: {
                if (switchedState) {
                    gameHardware.setTargetArmExtension(-400);
                    gameHardware.setTargetArmRotation(370);
                    gameHardware.smoothArmExtension = false;

                    gameHardware.pusherPos = 0;
                    gameHardware.setTargetPokerRotation(0.8);

                    gameHardware.clawPose = 2;
                    gameHardware.clawOpen = true;

                    useBaseTeleOpMovement = true;
                    useSmoothMovement = true;
                    slowMovement = true;
                }
                if (timeInState > 1) {
                    gameHardware.clawPose = 3;
                }
                if (timeInState > 2) {
                    setState(DriveState.DRIVING);
                }
                break;
            }
        }
        telemetry.addData("State", state.toString());
        switchedState = false;
        gameHardware.update(false);
    }

    public void teleOpInitialize() {
        gameHardware = new CenterStageHardware(hardwareMap);
        gameHardware.initPose();
    }

}
