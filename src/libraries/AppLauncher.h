#ifndef APPLAUNCHER_H
#define APPLAUNCHER_H

#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "StateMachine.h"

class AppLauncher
{
    public:
        AppLauncher( std::string config_file, int positive_matches, int negative_matches);
        AppLauncher( std::string config_file, std::vector<int> positive_matches, std::vector<int> negative_matches);

        void update( int current_value);

        float getPercentageMatches(int i);

        bool getFound(int i);
        int getValueToTrackAt(int i);

        int getNumberOfCommands();

    private:
        void configFileParser( std::string config_file);

        std::vector<StateMachine> _state_machines;
        std::vector<std::string> _commands_to_launch;
        std::vector<int> _values_to_track;

        std::vector<bool> _found;

};

#endif
