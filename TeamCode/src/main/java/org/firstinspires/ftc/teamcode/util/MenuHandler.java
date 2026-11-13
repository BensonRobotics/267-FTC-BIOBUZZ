package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class MenuHandler {
    private final Telemetry telemetry;
    private int currentMenuIdx = 0;
    private boolean unlocked = true;
    private final MenuOption[] options = new MenuOption[] {
      new MenuOption("Alliance", new String[] {"Blue", "Red"}),
      new MenuOption("Starting position", new String[]{"Left Of Park", "Right Of Park"}),
      new MenuOption("Auto Mode", new String[]{"Just Shoot", "Shoot Then Intake", "Shoot, Intake, and Shoot"})
    };

    public MenuHandler(Telemetry tel){
        this.telemetry = tel;
    }

    public void Redraw(){
        telemetry.addLine("Realign turret with forward direction marking.");
        if(!unlocked){
            telemetry.addLine("Configuration confirmed. Menu is locked.");
        }
        for(int i = 0; i < options.length; i++){
            if(currentMenuIdx == i){
                telemetry.addLine("> " + options[i].name + " : " + options[i].options[options[i].optionIdx]);
            }
            else{
                telemetry.addLine("  " + options[i].name + " : " + options[i].options[options[i].optionIdx]);
            }
        }
    }
    public void ChangeSelection(int amount){
        if(!unlocked){
            return;
        }
        currentMenuIdx += amount;
        if(currentMenuIdx > options.length){
            currentMenuIdx = 0;
        }
        else if(currentMenuIdx < 0){
            currentMenuIdx = options.length;
        }
    }

    public void EditCurrentSelection(){
        if(!unlocked){
            return;
        }
        options[currentMenuIdx].optionIdx++;
        if(options[currentMenuIdx].optionIdx > options[currentMenuIdx].options.length){
            options[currentMenuIdx].optionIdx = 0;
        }
    }

    public MenuOptionExport[] ExportSelections(){
        List<MenuOptionExport> exports = new ArrayList<MenuOptionExport>();
        for(int i = 0; i < options.length; i++){
            exports.add(new MenuOptionExport(options[i].name, options[i].options[options[i].optionIdx]));
        }
        return exports.toArray(new MenuOptionExport[0]);
    }

    public void LockOptions(){
        unlocked = false;
    }
}
