#ifndef SimUI_H
#define SimUI_H

#include "FL/Fl_Widget.H"

struct SimUI_Widgets
{
    void *toolbar;
};

class SimUI
{
public:
    SimUI(int width, int height, const char *title);

    SimUI_Widgets ws;
private:
    static void cb_close(Fl_Widget *w, void *data);
};

void update_mox_conc_disp(float conc);
void update_pid_conc_disp(float conc);
void update_tdlas_conc_disp(float conc);

#endif
