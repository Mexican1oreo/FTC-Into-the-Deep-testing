package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Util.IDs.ARM_MOTOR_ID;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotorEx armMotor;

    public void init(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, ARM_MOTOR_ID);
    }

    public void moveArm(double power) {
        this.armMotor.setPower(power);
    }
}
