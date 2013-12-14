//------------------------------------------------------------------------------
//-- StateMachine
//------------------------------------------------------------------------------
//--
//-- Simple state machine for value tracking and event recognition
//--
//------------------------------------------------------------------------------
//--
//-- This file belongs to the "Gecko - Gesture Recognition" project
//-- (https://github.com/David-Estevez/gecko)
//--
//------------------------------------------------------------------------------
//-- Authors: David Estevez Fernandez
//--          Irene Sanz Nieto
//--
//-- Released under the GPL license (more info on LICENSE.txt file)
//------------------------------------------------------------------------------

/*! \file StateMachine.cpp
 *  \brief Simple state machine for value tracking and event recognition
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */


#include "StateMachine.h"


StateMachine::StateMachine(int value_to_track, unsigned int positive_matches, unsigned int negative_matches)
{
    this->value_to_track = value_to_track;
    this->min_positive_matches = positive_matches;
    this->max_negative_matches = negative_matches;

    current_positive_matches = 0;
    current_negative_matches = 0;
    found = false;
}


void StateMachine::update(int current_value)
{
    if ( current_value == value_to_track )
    {
        if ( current_positive_matches < min_positive_matches)
            current_positive_matches++;

        current_negative_matches = 0;
    }
    else
    {
        if ( current_negative_matches < max_negative_matches)
            current_negative_matches++;
    }

    if ( current_positive_matches == min_positive_matches )
        found = true;

    if ( current_negative_matches == max_negative_matches )
    {
        reset();
    }
}


void StateMachine::reset()
{
    found = false;
    current_negative_matches = 0;
    current_positive_matches = 0;
}


int StateMachine::getValue_to_track() const
{
    return value_to_track;
}

void StateMachine::setValue_to_track(const int &value)
{
    value_to_track = value;
    reset();
}

float StateMachine::getPercentageMatches()
{
    return current_positive_matches / (float) min_positive_matches;
}

bool StateMachine::getFound()
{
    return found;
}


