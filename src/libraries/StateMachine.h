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

/*! \file StateMachine.h
 *  \brief Simple state machine for value tracking and event recognition
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */

#ifndef STATEMACHINE_H
#define STATEMACHINE_H


class StateMachine
{
    public:
        StateMachine( int value_to_track, unsigned int positive_matches, unsigned int negative_matches );

        void update(int current_value );

        void reset( );

        int getValue_to_track() const;
        void setValue_to_track(const int &value);

        float getPercentageMatches();

        bool getFound();

private:
        int value_to_track;

        unsigned int positive_matches;
        unsigned int current_positive_matches;

        unsigned int negative_matches;
        unsigned int current_negative_matches;

        bool found;
};

#endif
