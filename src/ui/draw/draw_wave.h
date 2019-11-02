#ifndef DRAW_WAVE_H
#define DRAW_WAVE_H

#include <FL/Fl.H>

class WavePlot : public Fl_Widget
{
public:
    WavePlot(int X, int Y, int W, int H, const char *);

    void start(void);

    void stop(void);

    int robot_to_display; // robot index, to display the sensor reading
private:
    void draw(void);

    static void Timer_CB(void *data);
};

#endif
