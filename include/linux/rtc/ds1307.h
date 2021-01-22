#ifndef __LINUX_RTC_DS1307_H
#define __LINUX_RTC_DS1307_H

/*
 * External clock input configuration for ds1341/42 RTC chips.
 */
struct ds1342_platform_data {
    /* Disable Oscillator Stop Flag */
    bool dosf;
    /* Enable External Clock Input */
    bool eclk;
    /* Select Clock Source */
    int clksel;
};

#endif /* __LINUX_RTC_DS1307_H */
