package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FGCCollect", group = "Iterative Opmode")
//@Disabled
public class FGCCollect extends OpMode{
    public DcMotor motorCollector;
    double Power=.5;
    private ElapsedTime runtime = new ElapsedTime();//计时
    boolean isRunning=false;


    public void init()
    {
        CollectSystemConfigue();
        telemetry.addData("Inited","Done");
        telemetry.addData("Ammo(Useful)\t",getBatteryVoltage());
        telemetry.update();
    }
    @Override
    public void init_loop()
    {
    }
    @Override
    public void start()
    {
        telemetry.clearAll();
        runtime.reset();
    }



    public void CollectSystemConfigue()
    {
        motorCollector = hardwareMap.get(DcMotor.class,"motorCollector");
        motorCollector.setDirection(DcMotor.Direction.FORWARD);
    }
    public void loop()
    {
        if (gamepad1.left_stick_button)
        {
            if (!isRunning)Collecting();
            if (isRunning)StopCollecting();
        }
        if (gamepad1.left_bumper)
        {
            while (gamepad1.left_bumper){}
            Power= Range.clip( motorCollector.getPower()+.1,-1,1);

        }
        if (gamepad1.right_bumper)
        {
            while (gamepad1.right_bumper){}
            Power= Range.clip( motorCollector.getPower()-.1,-1,1);
        }
        telemetry.addData("CollectPower",Power);
        motorCollector.setPower(Power);
        telemetry.update();
    }
    public double getBatteryVoltage()//获取电量
    {
        double result = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if(voltage > 0)
            {
                result = Math.min(result,voltage);
            }
        }
        return result;
    }
    public void Collecting()
    {
        motorCollector.setPower(1);
    }

    public void StopCollecting()
    {
        motorCollector.setPower(0);
    }
}
