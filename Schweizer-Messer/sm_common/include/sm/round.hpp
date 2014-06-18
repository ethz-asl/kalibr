#ifndef SM_ROUND_HPP
#define SM_ROUND_HPP

namespace sm {
    
    inline double round (double number)
    {
        return (number < 0.0 ? ceil (number - 0.5) : floor (number + 0.5));
    }

    inline float round (float number)
    {
        return (number < 0.0f ? ceilf (number - 0.5f) : floorf (number + 0.5f));
    }

    inline int rint(double number)
    {
        return static_cast<int>( round(number) );
    }

    inline int rint(float number)
    {
        return static_cast<long int>( round(number) );
    }

    inline long int lrint(double number)
    {
        return static_cast<int>( round(number) );
    }

    inline long int lrint(float number)
    {
        return static_cast<long int>( round(number) );
    }
    
} // namespace sm


#endif /* SM_ROUND_HPP */
