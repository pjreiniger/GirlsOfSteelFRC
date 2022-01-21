/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package com.gos.aerial_assist.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import com.gos.aerial_assist.subsystems.Collector;

/**
 * @author Sylvie and Heather
 */
public class CollectorDownAndWheelIn extends CommandGroup {
    public CollectorDownAndWheelIn(Collector collector) {
        addParallel(new CollectorWheelForward(collector));
        addParallel(new EngageCollector(collector));
    }
}