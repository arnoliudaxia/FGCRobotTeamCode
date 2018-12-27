package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

/**
 * Created by arno on 18-5-21.
 */
@Disabled
public class CollectSystem extends BasicOpMode_Iterative
{
    public DcMotor motorCollector;
    double Power=.5;

    public void CollectSystemConfigue()
    {
        motorCollector = hardwareMap.get(DcMotor.class,"motorCollector");
        motorCollector.setDirection(DcMotor.Direction.FORWARD);
    }

    //region 控球系统
    public void Collecting()
    {
        motorCollector.setPower(1);
    }

    public void StopCollecting()
    {
        motorCollector.setPower(0);
    }

    //endregion
    public void ChangeSpeed(double deltaV)
    {
        Power=deltaV;
        motorCollector.setPower(Power);

    }
    public void Loop()
    {
        if(gamepad1.left_stick_button)
            this.Collecting();
        if(gamepad1.right_stick_button)
            this.StopCollecting();
        if(this.motorCollector.getPower()!=0)
        {
            double v=this.motorCollector.getPower();
            if(gamepad1.right_stick_y!=0)
            {
                 ChangeSpeed(Range.clip(-gamepad1.right_stick_y+v,0,1));
            }

        }
    }
}
