#ifndef MAINSENSORHUB_H
#define MAINSENSORHUB_H

/*
 * =========== timer variable ===========
 */
#define LOOP_RATE_1000Hz_ 1000

/*
 * COUNT is defined for timer 1ms
 */
#define COUNT_50_HZ_	20
#define COUNT_100_HZ_	10
#define COUNT_125_HZ_   8
struct TimerCountType{
    unsigned char Hz_100;
    unsigned char Hz_50;
    unsigned char Hz_125;
}Timer_Count;

struct FlagTimerType{
    unsigned char Hz_50:1;
    unsigned char Hz_100:1;
    unsigned char Hz_125:1;
}FlagTimer;
//======timer handler run in 1000Hz=========
void Timer_handler(){
    Timer_Count.Hz_50++;
    Timer_Count.Hz_100++;
    Timer_Count.Hz_125++;
    if(Timer_Count.Hz_50==COUNT_50_HZ_){
        Timer_Count.Hz_50=0;
        FlagTimer.Hz_50=1;
    }
    if(Timer_Count.Hz_100==COUNT_100_HZ_)
    {
        Timer_Count.Hz_100=0;
        FlagTimer.Hz_100=1;
    }
    if(Timer_Count.Hz_125==COUNT_125_HZ_)
    {
        Timer_Count.Hz_125=0;
        FlagTimer.Hz_125=1;
    }
}


/*
 * =========== sensor variable ===========
 */


#endif // MAINSENSORHUB_H
