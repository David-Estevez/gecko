#include "AppLauncher.h"

AppLauncher::AppLauncher(std::string config_file, int positive_matches, int negative_matches)
{
    configFileParser( config_file);

    for (int i = 0; i < _values_to_track.size(); i++ )
    {
        StateMachine newMachine( _values_to_track[i], positive_matches, negative_matches );
        _state_machines.push_back( newMachine);
    }

    _found = std::vector<bool>( _state_machines.size() );
}

AppLauncher::AppLauncher(std::string config_file, std::vector<int> positive_matches, std::vector<int> negative_matches)
{
    configFileParser( config_file);

    if ( positive_matches.size() == _values_to_track.size() && negative_matches.size() == _values_to_track.size() )
        for (int i = 0; i < _values_to_track.size(); i++ )
        {
            StateMachine newMachine( _values_to_track[i], positive_matches[i], negative_matches[i] );
            _state_machines.push_back( newMachine);
        }
    else
        std::cerr << "[AppLauncher] Error: Indices of vectors do not match" << std::endl;

    _found = std::vector<bool>( _state_machines.size() );

}

void AppLauncher::update(int current_value)
{
    for (int i = 0; i < _state_machines.size(); i++)
    {
        _state_machines[i].update( current_value );
        _found[i] = _state_machines[i].getFound();

        //-- Check state vector here and launch commands
        if ( _found[i] )
        {
            if( system( _commands_to_launch[i].c_str() ) == -1)
            {
                std::cerr << "[AppLauncher] Error: could not run \"" << _commands_to_launch[i] << "\"" << std::endl;
            }

            _state_machines[i].reset();
        }
    }

}

float AppLauncher::getPercentageMatches(int i)
{
    return _state_machines[i].getPercentageMatches();
}

bool AppLauncher::getFound(int i)
{
    return _found[i];
}

int AppLauncher::getValueToTrackAt(int i)
{
    return _state_machines[i].getValue_to_track();
}

int AppLauncher::getNumberOfCommands()
{
    return _commands_to_launch.size();
}

void AppLauncher::configFileParser(std::string config_file)
{
    //-- Clear data:
    _commands_to_launch.clear();
    _values_to_track.clear();

    //-- Open config file
    std::ifstream file(config_file.c_str() );

    if ( !file.is_open() )
    {
        std::cerr << "[AppLauncher] Error opening file: " << config_file << std::endl;
        return;
    }

    while( !file.eof() )
    {
        //-- Read command:
        std::string newCommand;
        file >> newCommand;

        //-- Read value:
        int newValue;
        file >> newValue;

        //-- Save those values:
        _values_to_track.push_back( newValue );

        for( int i = 0; i < newCommand.length(); i++)
            if ( newCommand[i] == '$' )
                newCommand.erase( i, 1);

        _commands_to_launch.push_back(newCommand);

        std::cout << "[AppLauncher][Debug] Read: " << newCommand << " " << newValue << std::endl;
    }

    file.close();
}
