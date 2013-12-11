#include "StateMachine.h"


StateMachine::StateMachine(int value_to_track, unsigned int positive_matches, unsigned int negative_matches)
{
    this->value_to_track = value_to_track;
    this->positive_matches = positive_matches;
    this->negative_matches = negative_matches;

    current_positive_matches = 0;
    current_negative_matches = 0;
    found = false;
}


void StateMachine::update(int current_value)
{
    if ( current_value == value_to_track )
    {
        if ( current_positive_matches < positive_matches)
            current_positive_matches++;

        current_negative_matches = 0;
    }
    else
    {
        if ( current_negative_matches < negative_matches)
            current_negative_matches++;
    }

    if ( current_positive_matches == positive_matches )
        found = true;

    if ( current_negative_matches == negative_matches )
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
}

float StateMachine::getPercentageMatches()
{
    return current_positive_matches / (float) positive_matches;
}

bool StateMachine::getFound()
{
    return found;
}


