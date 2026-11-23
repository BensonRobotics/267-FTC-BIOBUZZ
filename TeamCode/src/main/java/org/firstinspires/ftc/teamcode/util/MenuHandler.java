package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class MenuHandler {
    private final Telemetry telemetry;
    private int currentMenuIdx = 0;
    private boolean unlocked = true;
    private final MenuOption[] options;

    public MenuHandler(Telemetry tel, MenuOption[] optionsIn){
        this.telemetry = tel;
        options = optionsIn;
    }

    public void Redraw(){
        telemetry.addLine("Realign turret with forward direction marking.");
        if(!unlocked){
            telemetry.addLine("Configuration confirmed. Menu is locked.");
        }
        for(int i = 0; i < options.length; i++){
            if(currentMenuIdx == i){
                if(options[i].options != null){
                    telemetry.addLine("> " + options[i].name + " : " + options[i].options[options[i].optionIdx]);
                }
                else{
                    telemetry.addLine("> " + options[i].name + " : " + options[i].selectedNumber);
                }
            }
            else{
                if(options[i].options != null){
                    telemetry.addLine("  " + options[i].name + " : " + options[i].options[options[i].optionIdx]);
                }
                else{
                    telemetry.addLine("  " + options[i].name + " : " + options[i].selectedNumber);
                }
            }
        }
    }
    public void ChangeSelection(int amount){
        if(!unlocked){
            return;
        }
        currentMenuIdx += amount;
        if(currentMenuIdx >= options.length){
            currentMenuIdx = 0;
        }
        else if(currentMenuIdx < 0){
            currentMenuIdx = options.length - 1;
        }
    }

    public void EditCurrentSelection(int amount){
        if(!unlocked){
            return;
        }
        options[currentMenuIdx].Change(amount);
    }

    public MenuOptionExport[] ExportSelections(){
        List<MenuOptionExport> exports = new ArrayList<MenuOptionExport>();
        for(int i = 0; i < options.length; i++){
            exports.add(options[i].Export());
        }
        return exports.toArray(new MenuOptionExport[0]);
    }

    public void LockOptions(){
        unlocked = false;
    }
}
