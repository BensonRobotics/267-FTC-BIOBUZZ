package org.firstinspires.ftc.teamcode.util;

public class MenuOption {
    public final String name;
    public String[] options = null;
    public int optionIdx = 0;
    public double increment;
    public double min;
    public double max;
    public double selectedNumber;

    public void Change(int amount){
        if(options != null){
            optionIdx+=amount;
            if(optionIdx >= options.length){
                optionIdx = 0;
            }
            else if(optionIdx <= 0){
                optionIdx = options.length - 1;
            }
        }
        else{
            selectedNumber += amount * increment;
            if(selectedNumber >= max){
                selectedNumber = min;
            }
            else if(selectedNumber <= min){
                selectedNumber = max;
            }
        }
    }

    public MenuOptionExport Export(){
        if(options != null){
            return new MenuOptionExport(name, options[optionIdx]);
        }
        else{
            return new MenuOptionExport(name, selectedNumber);
        }
    }

    public MenuOption(String iname, String[] ioptions){
        name = iname;
        options = ioptions;

    }

    public MenuOption(String iname, double incrementIn, double imin, double imax){
        name = iname;
        increment = incrementIn;
        min = imin;
        max = imax;
        selectedNumber = min;
    }
    public MenuOption(String iname, double incrementIn, double imin, double imax, double defaultNum){
        name = iname;
        increment = incrementIn;
        min = imin;
        max = imax;
        selectedNumber = defaultNum;
    }
}
