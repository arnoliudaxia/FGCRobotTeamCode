package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

import static android.os.SystemClock.sleep;

@TeleOp(name="FGCDrop", group="Linear Opmode")
//@Disabled
public class FGCDrop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();//计时
    //region 定义闸门舵机
    Servo servoGate;
    Servo servoBlue;
    Servo servoOrange;
    //endregion
    //region 闸门角度
    double BluePosition=0;
    double OrangePosition=0;
    double GatePosition=0;
    //endregion
    //region 颜色传感器和角度定义
    public ColorSensor colorSensor;
    int Blue=12;
    int Orange=15;
    //endregion
    //region 舵机开关角度
    double BlueOpen=.08;
    double BlueClose=0;
    double OrangeOpen= 0.28;
    double OrangeClose= .5;
    double GateUp= .36;
    double GateDown= .1;
    //endregion
    public void init()
    {ControlSystemConfigue();ColourConfigue();}
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
    @Override
    public void loop()
    {
         SetDegree();
//        telemetry.addData("Blue",servoBlue.getPosition());
//        telemetry.addData("Orange",servoOrange.getPosition());
//        telemetry.addData("Gate",servoGate.getPosition());
//        telemetry.addData("GateV",GatePosition);

    }
    public void ControlSystemConfigue()
    {
        //region 实例化闸门舵机
        servoGate = hardwareMap.get(Servo.class,"servoGate");
        servoBlue = hardwareMap.get(Servo.class,"servoBlue");
        servoOrange = hardwareMap.get(Servo.class,"servoOrange");
        //endregion
        //region 初始化闸门角度
        BluePosition = servoBlue.getPosition();
        OrangePosition = servoOrange.getPosition();
        GatePosition=servoGate.getPosition();
        //endregion
    }
    public void SetDegree()
    {
        if(gamepad1.dpad_down==true||gamepad1.dpad_up==true||gamepad1.dpad_left==true||gamepad1.dpad_right==true||gamepad1.y||gamepad1.a)
        {

            //region 根据手柄1的方向轮盘调整红蓝球门的角度
            if(gamepad1.dpad_up)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_up){}
                //sleep(50);
                BluePosition=BlueOpen;
            }
            if(gamepad1.dpad_down)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_down){}
                //sleep(50);
                BluePosition=BlueClose;
            }
            if(gamepad1.dpad_left)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_left){}
                //sleep(50);
                OrangePosition=OrangeOpen;
            }

            if(gamepad1.dpad_right)
            {
                //noinspection StatementWithEmptyBody
                while(gamepad1.dpad_right){}
                //sleep(50);
                OrangePosition=OrangeClose;
            }
//            if (gamepad1.y)
//            {
//                //noinspection StatementWithEmptyBody
//                while (gamepad1.y){}
//                GatePosition=Range.clip(GatePosition+.02,0,1);
//            }
//            if (gamepad1.a)
//            {
//                //noinspection StatementWithEmptyBody
//                while (gamepad1.a){}
//                GatePosition=Range.clip(GatePosition-.02,0,1);
//
//            }

            //endregion
            servoOrange.setPosition(OrangePosition);
            servoBlue.setPosition(BluePosition);

        }
        if(colorSensor.red()>Orange)
        {
            servoGate.setPosition(GateUp);
        }
        else if(colorSensor.blue()>Blue)
        {
            servoGate.setPosition(GateDown);
        }
    }
    private void ColourConfigue()
    {
        colorSensor = hardwareMap.get(ColorSensor.class,"colour");
    }
}
