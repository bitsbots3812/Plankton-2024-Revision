package frc.robot;

//Class to allow using a POV like a button
public class POVBind {
    int angle;
    int index = 0;

    //Angle increases to the right
    POVBind (int Angle) {
        angle = Angle;
    }

    POVBind (int Angle, int Index) {
        angle = Angle;
        index = Index;
    }
}
