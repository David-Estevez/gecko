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
