#ifndef APPLAUNCHER_H
#define APPLAUNCHER_H

#include <string>

class AppLauncher : StateMachine
{
    public:

        AppLaucher( std::string config_file );

        AppLauncher( std::string config_file, int positive_matches, int negative_matches )
};

#endif
