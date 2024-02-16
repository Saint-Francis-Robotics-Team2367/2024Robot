#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include <iostream>

// for use in the ShuffleUI class
class UIWidget {
  private:
    //pointer to widget's entry
    nt::GenericEntry *entry;
    //widget's name
    std::string name;
    //widget's tab
    std::string tab;

  public:
    /**
     * UIWidget constructor
     *
     * @param entry pointer to widget's entry
     * @param tab Widget's tab
     * @param name Widget's name
     */
    UIWidget(nt::GenericEntry *entry, std::string tab, std::string name) 
    {
        this->name = name;
        this->tab = tab;
        this->entry = entry;
    }
    
    /**
     * Get's a UIWidget's entry
     * @return pointer to the widget's entry
     */
    nt::GenericEntry * GetEntry() 
    {
        return this->entry;
    }

    /**
     * Get's a UIWidget's name
     * @return widget's name
     */
    std::string GetName() {
        return this->name;
    }

    /**
     * Get's a UIWidget's tab
     * @return widget's tab
     */
    std::string GetTab() {
        return this->tab;
    }
};

// A static class for making Shuffleboard easier to use
class ShuffleUI {
  private:
    // static vector to store info on created widgets, using the UIWidget class.
    static std::vector<UIWidget *> widgetList;

    /**
     * Adds a UIWidget to a list of created widgets
     *
     * @param widget pointer to a UIWidget
     */
    static void AddEntry(UIWidget *widget);


  public:
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, int value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param row Widget row
     * @param col Widget column
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value, int row, int col);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param widget Widget Type
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value, frc::BuiltInWidgets widgetType);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param widget Widget Type
     * @param row Widget row
     * @param col Widget column
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value, frc::BuiltInWidgets widgetType, int row, int col);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, float value);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, bool value);
    
    /**
     * Creates a new widget with a slider.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param min Slider lower bound
     * @param max Slider upper bound
     * @param defaultVal Slider's default value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal);

    /**
     * Finds an existing widget.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @return pointer to the widget's entry, or NULL if widget does not exist
     */
    static nt::GenericEntry *GetEntry(std::string name, std::string tab);

    /**
     * Creates a new widget with a graph, or updates an existing graph's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeGraph(std::string name, std::string tab, double value);

    /**
     * Creates a new widget with a button.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultState Widget's default state
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeButton(std::string name, std::string tab, bool defaultState);

    /**
     * Prints the names of all widgets.
     */
    static void PrintWidgetList();
};
