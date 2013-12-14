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

/*! \class StateMachine
 * \brief Simple state machine for value tracking and event recognition
 *
 * It has internally two counters, one counts the number of positive matches with the
 * value to track, and the other the number of consecutive negative matches.
 *
 * If the number of consecutive negative matches is equal to the maximum value allowed, it
 * resets the positive matches counter. If the the number of positive matches equals the
 * minimum value of positive matches needed, it sets the found variable true, until it is
 * reseted (either by the user or by the arrival of negative matches).
 */
class StateMachine
{
    public:
        /*! \brief StateMachine constructor
         *  \param value_to_track Value to track for the positive matches
         *  \param min_positive_matches Minimum number of coincidences with the value to track before it considers it has been found
         *  \param max_negative_matches Maximum number of negative matches allowed between two positive matches
         */
        StateMachine( int value_to_track, unsigned int min_positive_matches, unsigned int max_negative_matches );

        /*! \brief Update the state of the state machine according to the current value passed to it
         *  \param current_value Current value to compare with the value to track
         */
        void update(int current_value );

        //! \brief Reset the state of the state machine ( 0 positive matches and 0 negative matches )
        void reset( );


        int getValue_to_track() const;              //!< \brief Returns the current value set to track
        void setValue_to_track(const int &value);   //!< \brief Changes the current value to track

        float getPercentageMatches();               //!< \brief Returns the current number of positive matches as a percentage

        bool getFound();                            //!< \brief Gets the current state of the state machine

private:
        int value_to_track;                         //!< \brief Value to track

        unsigned int min_positive_matches;          //!< \brief Minimum number of positive matches needed to consider a value found
        unsigned int current_positive_matches;      //!< \brief Current number of positive matches

        unsigned int max_negative_matches;          //!< \brief Maximum number of negative matches needed to reset the state/counter
        unsigned int current_negative_matches;      //!< \brief Current number of negative matches

        bool found;                                 //!< \brief State of the state machine, true if current number of positive matches
                                                    //!<        is equal that the minimum needed
};

#endif
