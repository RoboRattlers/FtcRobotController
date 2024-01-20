package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ViperSlide336mm extends LinearSlide {


    // all lengths are in inches
    public ViperSlide336mm(DcMotorEx myMotor) {
        motor = myMotor;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);
        fullLength = 38;
        cappedLength = 30;
        encoderTicksPerRevolution = 537.7;
        encoderTicksInFullLength = -3150;
    }

}
