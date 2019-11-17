/*
 * User Interface of Simulator
 *         using FLTK
 *
 * Author: Roice (LUO Bing)
 * Date: 2016-02-01 create this file
 */

/* FLTK */
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Light_Button.H>
#include <FL/Fl_Pixmap.H>
#include <FL/Fl_Tabs.H>
#include <FL/fl_ask.H>
#include <FL/Fl_Shared_Image.H>
#include <FL/Fl_PNG_Image.H>
#include <FL/Fl_Value_Input.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Output.H>
/* OpenGL */
#include <FL/Fl_Gl_Window.H>
#include <GL/glew.h>
#include GL_HEADER
#include GLUT_HEADER
/* RAOS */
#include "ui/SimUI.h"
#include "ui/icons/icons.h" // pixmap icons used in Tool bar
#include "ui/SimView.h" // 3D RAO view
#include "SimThread.h"
#include "SimConfig.h" // runtime RAOS configs
#include <sstream>
#include <string>

/*------- Configuration Dialog -------*/
struct ConfigDlg_widgets
{ // for parameter saving
    // arena widith/length/height
    Fl_Value_Input *arena_w;
    Fl_Value_Input *arena_l;
    Fl_Value_Input *arena_h;
    // source position xyz
    Fl_Value_Slider *source_x;
    Fl_Value_Slider *source_y;
    Fl_Value_Slider *source_z;
    // plume params

};

class ConfigDlg : public Fl_Window
{
public:
    ConfigDlg(int xpos, int ypos, int width, int height, const char *title);

private:
    // widgets
    struct ConfigDlg_widgets dlg_w;

    // callback funcs
    static void cb_close(Fl_Widget *, void *);

    static void cb_switch_tabs(Fl_Widget *, void *);

    static void cb_change_src_pos_bounds(Fl_Widget *, void *);

    // function to save current value of widgets to runtime configs
    static void save_value_to_configs(ConfigDlg_widgets *);

    // function to get runtime configs to set value of widgets
    static void set_value_from_configs(ConfigDlg_widgets *);
};

void ConfigDlg::cb_close(Fl_Widget *w, void *data)
{
    if (Fl::event() == FL_CLOSE)
    {
        struct ConfigDlg_widgets *ws = (struct ConfigDlg_widgets *) data;
        // save widget values to RAOS runtime configs when closing the dialog window
        save_value_to_configs(ws);
        ((Fl_Window *) w)->hide();
    }
}

void ConfigDlg::cb_switch_tabs(Fl_Widget *w, void *data)
{
    Fl_Tabs *tabs = (Fl_Tabs *) w;
    // When tab changed, make sure it has same color as its group
    tabs->selection_color((tabs->value())->color());
}

// change source position bounds according to arena size
void ConfigDlg::cb_change_src_pos_bounds(Fl_Widget *arena_wlh, void *src_xyz)
{
    ((Fl_Value_Slider *) src_xyz)->minimum(-((Fl_Valuator *) arena_wlh)->value() / 2.0);
    ((Fl_Value_Slider *) src_xyz)->maximum(((Fl_Valuator *) arena_wlh)->value() / 2.0);
}

void ConfigDlg::save_value_to_configs(ConfigDlg_widgets *ws)
{
    SimConfig_t *configs = SimConfig_get_configs(); // get runtime configs
    // save arena size
    configs->arena.w = ws->arena_w->value();
    configs->arena.l = ws->arena_l->value();
    configs->arena.h = ws->arena_h->value();
    // save source pos
    configs->source.x = ws->source_x->value();
    configs->source.y = ws->source_y->value();
    configs->source.z = ws->source_z->value();
}

void ConfigDlg::set_value_from_configs(ConfigDlg_widgets *ws)
{
    SimConfig_t *configs = SimConfig_get_configs(); // get runtime configs
    // set arena size
    ws->arena_w->value(configs->arena.w);
    ws->arena_l->value(configs->arena.l);
    ws->arena_h->value(configs->arena.h);
    // set source pos & maximum
    ws->source_x->value(configs->source.x);
    ws->source_y->value(configs->source.y);
    ws->source_z->value(configs->source.z);
    ws->source_x->minimum(-configs->arena.w / 2.0);
    ws->source_y->minimum(-configs->arena.l / 2.0);
    ws->source_z->minimum(-configs->arena.h / 2.0);
    ws->source_x->maximum(configs->arena.w / 2.0);
    ws->source_y->maximum(configs->arena.l / 2.0);
    ws->source_z->maximum(configs->arena.h / 2.0);
}

ConfigDlg::ConfigDlg(int xpos, int ypos, int width, int height,
                     const char *title = 0) : Fl_Window(xpos, ypos, width, height, title)
{
    // add event handle to dialog window
    callback(cb_close, (void *) &dlg_w);
    // begin adding children
    begin();
    // Tabs
    int t_x = 5, t_y = 5, t_w = w() - 10, t_h = h() - 10;
    Fl_Tabs *tabs = new Fl_Tabs(t_x, t_y, t_w, t_h);
    {
        tabs->callback(cb_switch_tabs); // callback func when switch tabs
        // Tab Scenario
        Fl_Group *scenario = new Fl_Group(t_x, t_y + 25, t_w, t_h - 25, "Scenario");
        {
            // color of this tab
            scenario->color(0xe8e8e800); // light milk tea
            scenario->selection_color(0xe8e8e800); // light milk tea
            // Arena
            Fl_Box *arena = new Fl_Box(t_x + 10, t_y + 25 + 10, 180, 130, "Arena");
            arena->box(FL_PLASTIC_UP_FRAME);
            arena->labelsize(16);
            arena->labelfont(FL_COURIER_BOLD_ITALIC);
            arena->align(Fl_Align(FL_ALIGN_TOP | FL_ALIGN_INSIDE));
            // Arena width/length/height
            dlg_w.arena_w = new Fl_Value_Input(t_x + 10 + 60, t_y + 25 + 10 + 30, 80, 25, "Width");
            dlg_w.arena_l = new Fl_Value_Input(t_x + 10 + 60, t_y + 25 + 10 + 60, 80, 25, "Length");
            dlg_w.arena_h = new Fl_Value_Input(t_x + 10 + 60, t_y + 25 + 10 + 90, 80, 25, "Height");
            new Fl_Box(t_x + 10 + 60 + 80, t_y + 25 + 10 + 30, 20, 25, "m");
            new Fl_Box(t_x + 10 + 60 + 80, t_y + 25 + 10 + 60, 20, 25, "m");
            new Fl_Box(t_x + 10 + 60 + 80, t_y + 25 + 10 + 90, 20, 25, "m");
            // Source
            Fl_Box *source = new Fl_Box(t_x + 10 + 190, t_y + 25 + 10, 180, 130, "Source Position");
            source->box(FL_PLASTIC_UP_FRAME);
            source->labelsize(16);
            source->labelfont(FL_COURIER_BOLD_ITALIC);
            source->align(Fl_Align(FL_ALIGN_TOP | FL_ALIGN_INSIDE));
            // Source pos
            dlg_w.source_x = new Fl_Value_Slider(t_x + 10 + 210, t_y + 25 + 10 + 30, 140, 25, "X");
            dlg_w.source_y = new Fl_Value_Slider(t_x + 10 + 210, t_y + 25 + 10 + 60, 140, 25, "Y");
            dlg_w.source_z = new Fl_Value_Slider(t_x + 10 + 210, t_y + 25 + 10 + 90, 140, 25, "Z");
            dlg_w.source_x->labelsize(16);
            dlg_w.source_y->labelsize(16);
            dlg_w.source_z->labelsize(16);
            dlg_w.source_x->type(FL_HOR_NICE_SLIDER);
            dlg_w.source_y->type(FL_HOR_NICE_SLIDER);
            dlg_w.source_z->type(FL_HOR_NICE_SLIDER);
            dlg_w.source_x->align(Fl_Align(FL_ALIGN_LEFT));
            dlg_w.source_y->align(Fl_Align(FL_ALIGN_LEFT));
            dlg_w.source_z->align(Fl_Align(FL_ALIGN_LEFT));
            new Fl_Box(t_x + 10 + 210 + 140, t_y + 25 + 10 + 30, 20, 25, "m");
            new Fl_Box(t_x + 10 + 210 + 140, t_y + 25 + 10 + 60, 20, 25, "m");
            new Fl_Box(t_x + 10 + 210 + 140, t_y + 25 + 10 + 90, 20, 25, "m");
            dlg_w.arena_w->callback(cb_change_src_pos_bounds, (void *) dlg_w.source_x);
            dlg_w.arena_l->callback(cb_change_src_pos_bounds, (void *) dlg_w.source_y);
            dlg_w.arena_h->callback(cb_change_src_pos_bounds, (void *) dlg_w.source_z);
        }
        scenario->end();
        // Tab Flow
        Fl_Group *flow = new Fl_Group(t_x, t_y + 25, t_w, t_h - 25, "Flow");
        {
            // color of this tab
            flow->color(0xe0ffff00); // light blue
            flow->selection_color(0xe0ffff00); // light blue

            // Mean wind velocity
            Fl_Box *m_wind = new Fl_Box(t_x + 10, t_y + 25 + 10, 370, 130, "Mean Wind Vel");
            m_wind->box(FL_PLASTIC_UP_FRAME);
            m_wind->labelsize(16);
            m_wind->labelfont(FL_COURIER_BOLD_ITALIC);
            m_wind->align(Fl_Align(FL_ALIGN_TOP | FL_ALIGN_INSIDE));
            // Mean wind velocity x/y/z components
            Fl_Value_Slider *m_wind_x = new Fl_Value_Slider(t_x + 10 + 30, t_y + 25 + 10 + 30, 300, 25, "X");
            Fl_Value_Slider *m_wind_y = new Fl_Value_Slider(t_x + 10 + 30, t_y + 25 + 10 + 60, 300, 25, "Y");
            Fl_Value_Slider *m_wind_z = new Fl_Value_Slider(t_x + 10 + 30, t_y + 25 + 10 + 90, 300, 25, "Z");
            m_wind_x->labelsize(16);
            m_wind_y->labelsize(16);
            m_wind_z->labelsize(16);
            m_wind_x->type(FL_HOR_NICE_SLIDER);
            m_wind_y->type(FL_HOR_NICE_SLIDER);
            m_wind_z->type(FL_HOR_NICE_SLIDER);
            m_wind_x->align(Fl_Align(FL_ALIGN_LEFT));
            m_wind_y->align(Fl_Align(FL_ALIGN_LEFT));
            m_wind_z->align(Fl_Align(FL_ALIGN_LEFT));
            m_wind_x->bounds(0, 10); // 0~10 m/s
            m_wind_y->bounds(0, 10);
            m_wind_z->bounds(0, 10);
            new Fl_Box(t_x + 10 + 30 + 300, t_y + 25 + 10 + 30, 30, 25, "m/s");
            new Fl_Box(t_x + 10 + 30 + 300, t_y + 25 + 10 + 60, 30, 25, "m/s");
            new Fl_Box(t_x + 10 + 30 + 300, t_y + 25 + 10 + 90, 30, 25, "m/s");
        }
        flow->end();
        // Tab Plume
        Fl_Group *plume = new Fl_Group(t_x, t_y + 25, t_w, t_h - 25, "Plume");
        {
            // color of this tab
            plume->color(0xeeee0000); // light yellow+green (chlorine)
            plume->selection_color(0xeeee0000); // light yellow+green
            // Plume Params
            Fl_Box *m_plume = new Fl_Box(t_x + 10, t_y + 25 + 10, 370, 130, "Plume Param");
            m_plume->box(FL_PLASTIC_UP_FRAME);
            m_plume->labelsize(16);
            m_plume->labelfont(FL_COURIER_BOLD_ITALIC);
            m_plume->align(Fl_Align(FL_ALIGN_TOP | FL_ALIGN_INSIDE));
        }
        plume->end();
        // Tab Robot
        Fl_Group *robot = new Fl_Group(t_x, t_y + 25, t_w, t_h - 25, "Robot");
        {
            // color of this tab
            robot->color(0xa8a8a800); // light yellow+green (chlorine)
            robot->selection_color(0xa8a8a800); // light yellow+green
        }
        robot->end();
    }
    // Make sure default tab has same color as its group
    tabs->selection_color((tabs->value())->color());
    tabs->end();

    end();
    // set value according to runtime configs
    set_value_from_configs(&dlg_w);
    show();
}

/*------- ToolBar -------*/
struct ToolBar_Widgets
{
    Fl_Button *start; // start button
    Fl_Button *pause; // pause button
    Fl_Button *stop; // stop button
    Fl_Button *config; // config button
    Fl_Light_Button *record; // record button
};
struct ToolBar_Handles // handles of dialogs/panels opened by corresponding buttons
{
    ConfigDlg *config_dlg; // handle of config dialog opened by config button
};

class ToolBar : public Fl_Group
{
public:
    ToolBar(int Xpos, int Ypos, int Width, int Height, void *win);

    struct ToolBar_Widgets ws;
    static struct ToolBar_Handles hs;
private:
    static void cb_button_start(Fl_Widget *, void *);

    static void cb_button_pause(Fl_Widget *, void *);

    static void cb_button_stop(Fl_Widget *, void *);

    static void cb_button_config(Fl_Widget *, void *);
};

struct ToolBar_Handles ToolBar::hs = {NULL};

void ToolBar::cb_button_start(Fl_Widget *w, void *data)
{
    ToolBar_Widgets *widgets = (ToolBar_Widgets *) data;

    // if pause button is pressed, meaning that the initialization has been carried out, so just restore and continue
    if (widgets->pause->value())
    {
        // release pause button
        widgets->pause->activate();
        widgets->pause->clear();
        // continue running
        *(sim_get_pauss_control_addr()) = false;
    } else
    {
        // if pause button is not pressed, then check start button state
        if (((Fl_Button *) w)->value()) // if start button is pressed down
        {
            // lock config button
            widgets->config->deactivate();

            sim_start();
            data_start();
        } else
        {
            // user is trying to release start button when pause is not pressed
            ((Fl_Button *) w)->value(1);
        }
    }
}

void ToolBar::cb_button_pause(Fl_Widget *w, void *data)
{
    ToolBar_Widgets *widgets = (ToolBar_Widgets *) data;
    // if start button pressed, release it, and pause simulation
    if (widgets->start->value())
    {
        widgets->start->value(0); // release start button
        widgets->pause->deactivate(); // make pause button unclickable
        // pause simulation...
        *(sim_get_pauss_control_addr()) = true;
    } else
    {
        // if start button not pressed, pause button will not toggle and no code action will be took
        widgets->pause->clear();
    }
}

void ToolBar::cb_button_stop(Fl_Widget *w, void *data)
{
    // release start and pause buttons
    struct ToolBar_Widgets *widgets = (struct ToolBar_Widgets *) data;
    widgets->start->clear();
    widgets->pause->activate();
    widgets->pause->clear();

    /* stop simulation */
    sim_stop();
    data_stop();

    // unlock config button
    widgets->config->activate();
}

void ToolBar::cb_button_config(Fl_Widget *w, void *data)
{
    if (hs.config_dlg != NULL)
    {
        if (hs.config_dlg->shown()) // if shown, do not open again
        {}
        else
        {
            hs.config_dlg->show();
        }
    } else // first press this button
    {// create config dialog
        Fl_Window *window = (Fl_Window *) data;
        hs.config_dlg = new ConfigDlg(window->x() + 20, window->y() + 20,
                                      400, 400, "Settings");
    }
}

ToolBar::ToolBar(int Xpos, int Ypos, int Width, int Height, void *win) :
    Fl_Group(Xpos, Ypos, Width, Height)
{
    begin();
    Fl_Box *bar = new Fl_Box(FL_UP_BOX, Xpos, Ypos, Width, Height, "");
    Ypos += 2;
    Height -= 4;
    Xpos += 3;
    Width = Height;
    // widgets of this toolbar
    // instances of buttons belong to tool bar
    ws.start = new Fl_Button(Xpos, Ypos, Width, Height);
    Xpos += Width + 5;
    ws.pause = new Fl_Button(Xpos, Ypos, Width, Height);
    Xpos += Width + 5;
    ws.stop = new Fl_Button(Xpos, Ypos, Width, Height);
    Xpos += Width + 5;
    ws.config = new Fl_Button(Xpos, Ypos, Width, Height);
    Xpos += Width + 5;
    ws.record = new Fl_Light_Button(Xpos, Ypos, Width + 22, Height);
    Xpos += Width + 22 + 5;
    Fl_Box *bar_rest = new Fl_Box(FL_DOWN_BOX, Xpos, Ypos, bar->w() - Xpos, Height, "");
    resizable(bar_rest); // protect buttons from resizing
    // icons
    Fl_Pixmap *icon_start = new Fl_Pixmap(pixmap_icon_play);
    Fl_Pixmap *icon_pause = new Fl_Pixmap(pixmap_icon_pause);
    Fl_Pixmap *icon_stop = new Fl_Pixmap(pixmap_icon_stop);
    Fl_Pixmap *icon_config = new Fl_Pixmap(pixmap_icon_config);
    Fl_Pixmap *icon_record = new Fl_Pixmap(pixmap_icon_record);
    // link icons to buttons
    ws.start->image(icon_start);
    ws.pause->image(icon_pause);
    ws.stop->image(icon_stop);
    ws.config->image(icon_config);
    ws.record->image(icon_record);
    // tips for buttons
    ws.start->tooltip("Start Simulation");
    ws.pause->tooltip("Pause Simulation");
    ws.stop->tooltip("Stop Simulation");
    ws.config->tooltip("Settings");
    ws.record->tooltip("Recording");
    // types of buttons
    ws.start->type(FL_TOGGLE_BUTTON); // start & pause are mutually exclusive
    ws.pause->type(FL_TOGGLE_BUTTON);
    // colors
    ws.record->selection_color(FL_RED);
    // link call backs to buttons
    ws.start->callback(cb_button_start, (void *) &ws);
    ws.pause->callback(cb_button_pause, (void *) &ws);
    //  start & pause buttons will be released when stop button is pressed
    ws.stop->callback(cb_button_stop, (void *) &ws);
    ws.config->callback(cb_button_config, (void *) win);
    end();
}

/*------- StateBar -------*/
struct StateBar_Widgets
{
    Fl_Output* mox_conc; // mox conc disp
    Fl_Output* pid_conc; // pid conc disp
    Fl_Output* tdlas_conc; // tdlas conc disp
};

class StateBar : public Fl_Group
{
public:
    StateBar(int Xpos, int Ypos, int Width, int Height, void *win);

    struct StateBar_Widgets ws;
private:

};

StateBar::StateBar(int Xpos, int Ypos, int Width, int Height, void *win) :
    Fl_Group(Xpos, Ypos, Width, Height)
{
    begin();
    Fl_Box *bar = new Fl_Box(FL_UP_BOX, Xpos, Ypos, Width, Height, "");

    Ypos += 2;
    Height -= 4;
    // widgets of this statebar
    // MOX Concentration Disp
    Xpos += 3;
    Width = 200;
    ws.mox_conc = new Fl_Output(Xpos, Ypos, Width, Height, "Mox Conc: ");
    Xpos += Width + 5;

    // PID Concentration Disp
    Xpos += 3;
    Width = 200;
    ws.pid_conc = new Fl_Output(Xpos, Ypos, Width, Height, "PID Conc: ");
    Xpos += Width + 5;

    // TDLAS Concentration Disp
    Xpos += 3;
    Width = 250;
    ws.tdlas_conc = new Fl_Output(Xpos, Ypos, Width, Height, "TDLAS Conc: ");
    Xpos += Width + 5;

    Fl_Box *bar_rest = new Fl_Box(FL_DOWN_BOX, Xpos, Ypos, bar->w() - Xpos, Height, "");
    resizable(bar_rest); // protect buttons from resizing

    ws.mox_conc->value("MOX Conc: ");
    ws.pid_conc->value("PID Conc: ");
    ws.tdlas_conc->value("TDLAS Conc: ");
    end();
}

/*------- UI update handler -------*/
class UI_update_handler
{
public:
    StateBar* state_bar; // start button
};

UI_update_handler* g_ui_update_handler;

/* =============================================
 * ================  UI  =======================
 * ============================================= */
void SimUI::cb_close(Fl_Widget *w, void *data)
{
    // close RAOS
    if (Fl::event() == FL_CLOSE)
    {
        sim_stop();

        // close main window
        ((Fl_Window *) w)->hide();
    }
}

/*------- Creation function of User Interface  -------*/
SimUI::SimUI(int width, int height, const char *title = 0)
{
    /* Main Window, control panel */
    Fl_Double_Window *panel = new Fl_Double_Window(width, height, title);
    panel->resizable(panel);

    // Add tool bar, it's width is equal to panel's
    const unsigned int TOOL_BAR_HEIGHT = 34;
    ToolBar *tool = new ToolBar(0, 0, width, TOOL_BAR_HEIGHT, (void *) panel);
    tool->clear_visible_focus(); //just use mouse, no TABs

    // Add state bar, it's width is equal to panel's
    const unsigned int STATE_BAR_HEIGHT = 30;
    StateBar *state = new StateBar(0, height - STATE_BAR_HEIGHT - 10, width, STATE_BAR_HEIGHT, (void *) panel);
    state->clear_visible_focus(); //just use mouse, no TABs

    g_ui_update_handler = new UI_update_handler();
    g_ui_update_handler->state_bar = state;

    // protect buttons from resizing
    Fl_Box *rt = new Fl_Box(FL_NO_BOX, 0, tool->h(), width, height - tool->h() - state->h() - 15, "right_border");
    rt->hide();
    panel->resizable(rt);

    /* Add simulator view */
    panel->show(); // glut will die unless parent window visible
    /* begin adding children */
    panel->begin();
    //Set 3D Area's Size and Position

    glutInitWindowSize(width - 10, height - tool->h() - state->h() - 15);// be consistent with SimView_init
    glutInitWindowPosition(panel->x() + 5, tool->h() + 5); // place it inside parent window
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutCreateWindow("Simulation view");
    /* end adding children */
    panel->end();
    panel->resizable(glut_window);
    panel->callback(cb_close, &ws);

    // init Sim view
    SimView_init(width - 10, height - tool->h() - 10);// pass gl window size
};

void update_mox_conc_disp(float conc)
{
    std::stringstream sstr;
    sstr << "MOX Conc: ";
    sstr << conc;
    g_ui_update_handler->state_bar->ws.mox_conc->value(sstr.str().c_str());
}

void update_pid_conc_disp(float conc)
{
    std::stringstream sstr;
    sstr << "PID Conc: ";
    sstr << conc;
    g_ui_update_handler->state_bar->ws.pid_conc->value(sstr.str().c_str());
}

void update_tdlas_conc_disp(float conc)
{
    std::stringstream sstr;
    sstr << "TDLAS Conc: ";
    sstr << conc;
    g_ui_update_handler->state_bar->ws.tdlas_conc->value(sstr.str().c_str());
}

/* End of SimUI.cxx */
