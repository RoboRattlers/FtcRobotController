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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

@TeleOp
public class CenterStagePropDetectionTest extends BaseController {

    private int phase = 0;
    protected double phaseStartTime = 0;
    protected double timeInPhase = 0;
    private boolean phaseEndReached = false;
    private double phaseEndReachedTime = 0;
    private final boolean goToNextPhase = true;

    OpenCvCamera camera;

    public ArrayList<Phase> phases = new ArrayList<>();
    public HashMap<String, Integer> phaseIndicesById = new HashMap<>();

    public void addPhase(Runnable init, Runnable step, Supplier<Boolean> check) {
        phases.add(new Phase(init, step, check));
    }

    public void addPhase(Runnable init, Runnable step, Supplier<Boolean> check, String id) {
        Phase newPhase = new Phase(init, step, check);
        phases.add(newPhase);
        phaseIndicesById.put(id, phases.size() - 1);
    }

    private void goToPhaseNum(int phaseNum) {
        phaseEndReached = true;
        phaseStartTime = runtime.seconds();
        phase = phaseNum;
        try {
            phases.get(phase).Init.run();
        } catch (Exception err) {
            telemetry.addData("what", "???");
        }
        phaseEndReachedTime = runtime.seconds();
    }

    public void goToPhase(String id) {
        if (phaseIndicesById.get(id) == null) {
            return;
        }
        goToPhaseNum(phaseIndicesById.get(id));
    }

    public void autoOpInitialize () {
    }

    // dist: 62.5 inches
    @Override
    public void runOpMode() {

        baseInitialize();
        this.autoOpInitialize();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry.addData("Camera", camera);

        waitForStart();
        runtime.reset();
        CenterStagePropDetectionPipeline propDetectionPipeline = new CenterStagePropDetectionPipeline(new Size( (int) (240 * 16/9), 240), new double[]{255, 0, 0});

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        camera.setPipeline(propDetectionPipeline);
        // MAIN LOOP
        while (opModeIsActive()) {
            telemetry.addData("Zone", propDetectionPipeline.avgColor1);

            telemetry.update();

        }
    }
}

