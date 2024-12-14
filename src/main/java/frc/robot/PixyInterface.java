package frc.robot;
import java.util.ArrayList;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import java.util.ArrayList;
import java.util.Arrays;
import org.apache.commons.lang3.ArrayUtils;

//TODO: Optimize

public class PixyInterface {
  static enum BallColor {
    RED_BALL(1),
    BLUE_BALL(2);

    final int value;

    BallColor(int Value) {
      this.value = Value;
    }

    int getValue() {
      return this.value;
    }
  };

    Pixy2 pixy;   
    NetworkTable table;
  
    double[] dBlocks;

    double pixyMinTargetAspectRatio;
    double pixyMaxTargetAspectRatio;
    int pixyMinBlockHeight;
    int pixyMinBlockWidth;

    private int checksWithoutTarget = 0;
    private int state;
    private NetworkTable pixyTable;
    PixyInterface(LinkType type, double minTargetAspectRatio, double maxTargetAspectRatio, int minBlockHeight, int minBlockWidth) {
        SmartDashboard.putString("Pixy State:", "Initializing..."); 
        pixy = Pixy2.createInstance(type);
        state = pixy.init();
        SmartDashboard.putNumber("Pixy State Raw:", state);

        //double[] coords = new double[500];
        pixyTable = NetworkTableInstance.getDefault().getTable("pixy");
        
        pixyMinTargetAspectRatio = minTargetAspectRatio;
        pixyMaxTargetAspectRatio = maxTargetAspectRatio;
        pixyMinBlockHeight = minBlockHeight;
        pixyMinBlockWidth = minBlockWidth;

        /*switch (state){
          case 0: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_OK"); break;
          case -1: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_ERROR"); break;
          case -2: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_BUSY"); break;
          case -3: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_CHECKSUM_ERROR"); break;
          case -4: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_TIMEOUT"); break;
          case -5: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_BUTTON_OVERRIDE"); break;
          case -6: SmartDashboard.putString("Pixy State:", "PIXY_RESULT_PROG_CHANGING"); break;
          default: SmartDashboard.putString("Pixy State:", "UNKNOWN: " + String.valueOf(state)); break;

        }*/
    }

    public void updateTargets(){
      if (state != 0) return;
    
      double result = -2;
  
      int worked = pixy.getCCC().getBlocks(false, 3, 5); //have pixy acquire data //Param 2 is "sigmap" What is sigmap?
      
      //int worked =pixy.getCCC().getBlocks();
      SmartDashboard.putNumber("GetBlocks Return Code:", worked);
      ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
      int blockCount = blocks.size();
      SmartDashboard.putNumber("Target Number (blocks size)", blockCount);

      dBlocks = BlocksToArray(blocks);
      pixyTable.getEntry("blocks").setDoubleArray(dBlocks);
      double[] targetBlock = {-1, -1, -1, -1, -1};
      pixyTable.getEntry("targetBlock").setDoubleArray(targetBlock);
    }

    double steeringSuggestion(BallColor color){
        double result = -2;

        int ci = -1;
        if (dBlocks != null && dBlocks.length > 0) { //if blocks were found
          //will eventually contain data of optimal target

          double xcoord = 0;
          double ycoord = 0;
          int blockWidth = 0;
          int blockHeight = 0;
          int sig = 0;
          //String data = "ND";
          boolean valid = false;

          //Find the best target. Largest approximately square, propery colored signature.
          for (int i = 0; i < dBlocks.length; i += 5) {
            
            sig = (int)dBlocks[i];
            xcoord = dBlocks[i+1]; 
            ycoord = dBlocks[i+2]; 
            blockWidth = (int)dBlocks[i+3]; 
            blockHeight = (int)dBlocks[i+4]; 
            

            if (sig == color.value && blockWidth / blockHeight >= pixyMinTargetAspectRatio
              && blockWidth / blockHeight <= pixyMaxTargetAspectRatio && blockHeight >= pixyMinBlockHeight
              && blockWidth >= pixyMinBlockWidth) {
               //proper signature. approx. square
              //data = block.toString();
              valid = true; //mark block data as a valid target
              break; //exit loop, with variables properly set
            }
          }

          if (valid) {
            //compute target position from center -1.0 to 1.0
            result = -1*((xcoord-157.5)/157.5);
            //print target info
            SmartDashboard.putNumber("Block Width", blockWidth);
            SmartDashboard.putNumber("Block Height", blockHeight);
            SmartDashboard.putNumber("Block Signature", sig);  
            SmartDashboard.putBoolean("Targets:", true); //send to smartdashboard
            SmartDashboard.putNumber("X Coordinate:", xcoord);
            SmartDashboard.putNumber("Y Coordinate:", ycoord);
            //SmartDashboard.putString("Data String:", data);

            //set netTable value for pixy blocks
            double[] targetBlock = {sig, xcoord, ycoord, blockWidth, blockHeight};
            pixyTable.getEntry("targetBlock").setDoubleArray(targetBlock);
           
            checksWithoutTarget = 0;
          }
          else {
            checksWithoutTarget ++;

          }
        }
        else {
          SmartDashboard.putBoolean("Targets:", false);
        }
    
        return result;
    }

    int getChecksWithoutTarget() {
      return checksWithoutTarget;
    }

    int getPixyState() {
        return state;
    }

    double[] BlocksToArray(ArrayList<Block> blocks){
      ArrayList<Double> arrlist = new ArrayList<Double>(500);
      for (int i = 0; i < blocks.size(); i++) {
        Block b = blocks.get(i);
        arrlist.add(Double.valueOf(b.getSignature()));
        arrlist.add(Double.valueOf(b.getX()));
        arrlist.add(Double.valueOf(b.getY()));
        arrlist.add(Double.valueOf(b.getWidth()));
        arrlist.add(Double.valueOf(b.getHeight()));
      }

      Double[] array = arrlist.toArray(new Double[arrlist.size()]);
      double[] d = ArrayUtils.toPrimitive(array);
      //pixyTable.getEntry("blocks").forceSetDoubleArray(d);
      return d;

    }
}
