package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz AUTO RED", group = "BioBuzz", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzPedroAutoRed extends BioBuzzPedroAutoBase {
    @Override
    protected AutoField.Alliance alliance() {
        return AutoField.Alliance.RED;
    }
}