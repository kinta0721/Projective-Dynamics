#ifndef _ANTTWEAKBAR_WRAPPER_H_
#define _ANTTWEAKBAR_WRAPPER_H_

#include "AntTweakBar.h"
#include "global_headers.h"

typedef enum
{
    ATB_RESHAPE_WINDOW   = 0x1,
    ATB_CHANGE_TIME_STEP = 0x2,
    ATB_REPREFACTORIZE = 0x4,
    ATB_DEFAULT = 0x0

} ATBFeedBack;

class AntTweakBarWrapper
{
public:
    AntTweakBarWrapper();
    virtual ~AntTweakBarWrapper();

    // init / cleanup / reset / save / load / default / hide / show / update / Draw / change window size
    void init();
    void cleanup();
    void reset();
    void saveSettings();
    void loadSettings();
    void defaultSettings();
    void hide();
    void show();
    int  update();
    inline void draw() {TwDraw();}
    inline void changeTwBarWindowSize(int width, int height) {TwWindowSize(width, height);}

    static void TW_CALL setDefaultSettings(void*);
    static void TW_CALL saveSettings(void*);
    static void TW_CALL loadSettings(void*);

protected:

    // bars
    TwBar *m_control_panel_bar; // Control Panel
    TwBar *m_mesh_bar;   // Mesh Settings
    TwBar *m_sim_bar;    // Simulation Settings
};

#endif