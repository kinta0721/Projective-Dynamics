#pragma warning( disable : 4996)//‚È‚ñ‚Ì”Žš‚¾‚ë‚¤

//‚±‚ê‚Æmesh‚Æsimulation‚ÌŒÄ‚Ñž‚Ý
#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "simulation.h"

//----------Events Related Variables--------------------//

static int g_old_screen_width;
static int g_old_screen_height;
static ScalarType g_old_timestep;
static ScalarType g_old_attachment_stiffness;


//extern ‚ÌˆÓ–¡‚È‚ñ‚¾‚ë‚È
//----------Global rs----------------//
extern int g_screen_width;
extern int g_screen_height;

//----------State Control--------------------//
extern bool g_only_show_sim;
extern bool g_record;
extern bool g_pause;
extern bool g_show_wireframe;
extern bool g_show_texture;

//----------anttweakbar handlers----------//
extern void TW_CALL reset_simulation(void*);
extern void TW_CALL step_through(void*);

//----------key components--------------//
extern Mesh* g_mesh;
extern Simulation* g_simulation;

AntTweakBarWrapper::AntTweakBarWrapper()
{
}

AntTweakBarWrapper::~AntTweakBarWrapper()
{
    saveSettings(NULL);
    cleanup();
}

void AntTweakBarWrapper::init()
{
    TwInit(TW_OPENGL, NULL);

	// ----------control panel bar----------
    m_control_panel_bar = TwNewBar("Control Panel");
    TwDefine(" 'Control Panel' size='200 710' position='814 10' color='255 255 255' text=dark ");
    char control_bar_pos_string [255];
    sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
    TwDefine(control_bar_pos_string);

    // state control
    TwAddVarRW(m_control_panel_bar,  "Pause", TwType(sizeof(bool)), &(g_pause), "group='State Control'");
    TwAddButton(m_control_panel_bar, "Step Once", step_through, NULL, "group='State Control' ");
    TwAddVarRW(m_control_panel_bar,  "Record", TwType(sizeof(bool)), &(g_record), "group='State Control'");
    TwAddSeparator(m_control_panel_bar, NULL, "");

    // visualization
    TwAddVarRW(m_control_panel_bar, "Wireframe", TwType(sizeof(bool)), &(g_show_wireframe), "group='Visualization'");
    TwAddVarRW(m_control_panel_bar, "Texture", TwType(sizeof(bool)), &(g_show_texture), "group='Visualization'");
    TwAddVarRW(m_control_panel_bar, "Width", TW_TYPE_INT32, &(g_screen_width), "min=640 group='Screen Resolution'");
    TwAddVarRW(m_control_panel_bar, "Height", TW_TYPE_INT32, &(g_screen_height), "min=480 group='Screen Resolution'");
    TwAddSeparator(m_control_panel_bar, NULL, "");

    // buttons
    TwAddButton(m_control_panel_bar, "Save Settings", saveSettings, this, " ");
    TwAddButton(m_control_panel_bar, "Load Settings", loadSettings, this, " ");
    TwAddButton(m_control_panel_bar, "Default Settings", setDefaultSettings, this, " ");
    TwAddSeparator(m_control_panel_bar, NULL, "");
    TwAddButton(m_control_panel_bar, "Reset Simulation", reset_simulation, NULL, " ");
    //!Control Panel bar

	// ----------mesh settings bar----------
    m_mesh_bar = TwNewBar("Mesh Settings");
    TwDefine(" 'Mesh Settings' size='200 250' position='10 10' color='210 240 255' text=dark ");

    // mesh type
    TwEnumVal meshTypeStyleEV[1] =  
	{
        {MESH_TYPE_TET, "Tet Mesh"}
		// Please add another mesh type here !!
	};
    TwType meshTypeStyle = TwDefineEnum("MeshType", meshTypeStyleEV, 1);
    TwAddVarRW(m_mesh_bar, "Mesh Type", meshTypeStyle, &g_mesh->m_mesh_type, " ");
    TwAddVarRW(m_mesh_bar, "Total Mass", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_total_mass), " ");

    // tet settings
    TwAddVarRW(m_mesh_bar, "File", TW_TYPE_CSSTRING(sizeof(g_mesh->m_mesh_file_path)), &(g_mesh->m_mesh_file_path), " group='Tet Settings' ");
    TwAddVarRW(m_mesh_bar, "Tet Scaling", TW_TYPE_SCALAR_TYPE, &(g_mesh->m_tet_scaling), " min=0.01 group='Tet Settings' ");
    // !mesh settings bar

	// ----------simulation settings bar----------
    m_sim_bar = TwNewBar("Simulation Settings");
    TwDefine(" 'Simulation Settings' size='200 450' position='10 270' color='255 216 224' text=dark ");

    // integration
    TwAddVarRW(m_sim_bar, "Time Step", TW_TYPE_SCALAR_TYPE, &g_simulation->m_h, " min=0.0001 step=0.0001 ");
    TwEnumVal integrationStyleEV[INTEGRATION_TOTAL_NUM] =  
	{
		{INTEGRATION_LOCAL_GLOBAL, "Local Global"}
		// Please add another integration type here !!
	};
    TwType integrationStyle = TwDefineEnum("Integration Method", integrationStyleEV, INTEGRATION_TOTAL_NUM);
    TwAddVarRW(m_sim_bar, "Method", integrationStyle, &g_simulation->m_integration_method, " group='Integration' ");
    TwAddVarRW(m_sim_bar, "Iterations/Frame", TW_TYPE_INT32, &g_simulation->m_iterations_per_frame, " group='Integration' ");

	// material property
	TwEnumVal materialStyleEV[MATERIAL_TOTAL_NUM] = 
	{
		{ MATERIAL_COROTATIONAL_LINEAR, "Corotational-Linear" }
	};
	TwType materialStyle = TwDefineEnum("Elastic Material", materialStyleEV, MATERIAL_TOTAL_NUM);
	TwAddVarRW(m_sim_bar, "Material", materialStyle, &g_simulation->m_material_type, "group='Elasticity'");
	TwAddVarRW(m_sim_bar, "Young", TW_TYPE_SCALAR_TYPE, &g_simulation->m_young, "min=0.1 step=0.1 group='Elasticity'");
	TwAddVarRW(m_sim_bar, "Poisson", TW_TYPE_SCALAR_TYPE, &g_simulation->m_poisson, "min=0.0 max=1.0 step=0.001 group='Elasticity'");

    // constants
    TwAddVarRW(m_sim_bar, "Attachment Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_attachment, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Gravity", TW_TYPE_SCALAR_TYPE, &g_simulation->m_gravity_constant, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Damping Coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_damping_coefficient, " min=0 step=0.001 group='Constants' ");
    // !simulation settings bar

    TwDefine(" TW_HELP visible=false ");
}

void AntTweakBarWrapper::cleanup()
{
    m_control_panel_bar = NULL;
    m_mesh_bar = NULL;
    m_sim_bar = NULL;

    TwTerminate();
}

void AntTweakBarWrapper::reset()
{
    cleanup(); 
    init();
}

void AntTweakBarWrapper::hide()
{
    TwDefine(" 'Control Panel' visible=false ");
    TwDefine(" 'Mesh Settings' visible=false ");
    TwDefine(" 'Simulation Settings' visible=false ");
}

void AntTweakBarWrapper::show()
{
    TwDefine(" 'Control Panel' visible=true ");
    TwDefine(" 'Mesh Settings' visible=true ");
    TwDefine(" 'Simulation Settings' visible=true ");
}

int AntTweakBarWrapper::update()
{
    // update

    // control panel pos
    char control_bar_pos_string [255];
    sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
    TwDefine(control_bar_pos_string);

    // mesh settings display
    switch (g_mesh->m_mesh_type)
    {
    case MESH_TYPE_TET:
        TwDefine(" 'Mesh Settings'/'Tet Settings' visible=true");
        break;
    }

    // simulation settings display
    switch(g_simulation->m_integration_method)
    {
    case INTEGRATION_LOCAL_GLOBAL:
        TwDefine(" 'Simulation Settings'/'Iterations/Frame' visible=true");
		TwDefine(" 'Simulation Settings'/'Material' visible=false");
        break;
    }

    // give feed back
    int atb_feedback = ATB_DEFAULT;
    if (g_old_screen_width!=g_screen_width || g_old_screen_height!=g_screen_height)
    {
        g_old_screen_width = g_screen_width;
        g_old_screen_height = g_screen_height;
        atb_feedback |= ATB_RESHAPE_WINDOW;
    }
	// change timestep (feed back)
    if (g_simulation->m_h != g_old_timestep)
    {
        g_old_timestep = g_simulation->m_h;

        atb_feedback |= ATB_CHANGE_TIME_STEP;
    }
	// change stiffness (feed back)
    if (g_old_attachment_stiffness != g_simulation->m_stiffness_attachment)
    {
        g_old_attachment_stiffness = g_simulation->m_stiffness_attachment;

        atb_feedback |= ATB_REPREFACTORIZE;
    }
	// change Lame constant (feed back)
	if (g_simulation->m_young_old != g_simulation->m_young || \
		g_simulation->m_poisson_old != g_simulation->m_poisson)
	{
		g_simulation->m_young_old = g_simulation->m_young;
		g_simulation->m_poisson_old = g_simulation->m_poisson;
		g_simulation->convertLameConstant();

		atb_feedback = ATB_REPREFACTORIZE;
	}
    
    return atb_feedback;
}

void AntTweakBarWrapper::saveSettings()
{
    std::ofstream outfile;
    outfile.open(DEFAULT_CONFIG_FILE, std::ifstream::out);
    if (outfile.is_open())
    {
        // TODO: change it to memory dump.
        // global settings:
        outfile << "Wireframe           " << g_show_wireframe << std::endl;
        outfile << "Texture             " << g_show_texture << std::endl;
        outfile << "ScreenWidth         " << g_screen_width << std::endl;
        outfile << "ScreenHeight        " << g_screen_height << std::endl;
        outfile << std::endl;

        // mesh settings:
        outfile << "MeshType            " << g_mesh->m_mesh_type << std::endl;
        outfile << "MeshMass            " << g_mesh->m_total_mass << std::endl;
        outfile << "TetFilePath         " << g_mesh->m_mesh_file_path<< std::endl;
        outfile << "TetScaling          " << g_mesh->m_tet_scaling << std::endl;
        outfile << std::endl;

        // simulation settings:
        outfile << "SimMethod           " << g_simulation->m_integration_method << std::endl;
        outfile << "Timestep            " << g_simulation->m_h << std::endl;

		outfile << "Mattype             " << g_simulation->m_material_type << std::endl;
		outfile << "YoungModule         " << g_simulation->m_young << std::endl;
		outfile << "PoissonRatio        " << g_simulation->m_poisson << std::endl;

        outfile << "AttachmentStiffness " << g_simulation->m_stiffness_attachment << std::endl;
        outfile << "GravityConstant     " << g_simulation->m_gravity_constant << std::endl;
        outfile << "DampingCoefficient  " << g_simulation->m_damping_coefficient << std::endl;

        outfile << "IterationsPerFrame  " << g_simulation->m_iterations_per_frame << std::endl;

        outfile.close();
    }
    else
    {
        std::cerr << "Warning: Can not write config file. Settings not saved." << std::endl; 
    }
}

void AntTweakBarWrapper::loadSettings()
{
    bool successfulRead = false;

    //read file
    std::ifstream infile;
    infile.open(DEFAULT_CONFIG_FILE, std::ifstream::in);
    if (successfulRead = infile.is_open())
    {
        int tempEnum;
        char ignoreToken[256];

        // global settings:
        infile >> ignoreToken >> g_show_wireframe;
        infile >> ignoreToken >> g_show_texture;
        infile >> ignoreToken >> g_screen_width;
        infile >> ignoreToken >> g_screen_height;

        // mesh settings:
        infile >> ignoreToken >> tempEnum; g_mesh->m_mesh_type = MeshType(tempEnum);
        infile >> ignoreToken >> g_mesh->m_total_mass;
        infile >> ignoreToken >> g_mesh->m_mesh_file_path;
        infile >> ignoreToken >> g_mesh->m_tet_scaling;                         

        // simulation settings:
        infile >> ignoreToken >> tempEnum; g_simulation->m_integration_method = IntegrationMethod(tempEnum);
        infile >> ignoreToken >> g_simulation->m_h;

		infile >> ignoreToken >> tempEnum; g_simulation->m_material_type = MaterialType(tempEnum);
		infile >> ignoreToken >> g_simulation->m_young;
		infile >> ignoreToken >> g_simulation->m_poisson;

        infile >> ignoreToken >> g_simulation->m_stiffness_attachment;
        infile >> ignoreToken >> g_simulation->m_gravity_constant;
        infile >> ignoreToken >> g_simulation->m_damping_coefficient;

        infile >> ignoreToken >> g_simulation->m_iterations_per_frame;

        infile.close();
    }

    // setup default values
    if (!successfulRead)
    {
        std::cerr << "Waning: failed loading settings, set to defaults." << std::endl;
        defaultSettings();
    }

    // init event related variables
    g_old_screen_width = g_screen_width;
    g_old_screen_height = g_screen_height;
}

void AntTweakBarWrapper::defaultSettings()
{
    // global settings
    g_show_wireframe = false;
    g_show_texture = false;
    g_screen_width = 1024;
    g_screen_height = 768;

    // mesh settings
    g_mesh->m_mesh_type = MESH_TYPE_TET;
    g_mesh->m_total_mass = 1.0;
    // tet
    strcpy(g_mesh->m_mesh_file_path, DEFAULT_MODEL);
    g_mesh->m_tet_scaling = 1.0;

    //simulation settings
    g_simulation->m_integration_method = INTEGRATION_LOCAL_GLOBAL;
    g_simulation->m_h = 0.0333;

	g_simulation->m_material_type = MATERIAL_COROTATIONAL_LINEAR;
	g_simulation->m_young = 10.0;
	g_simulation->m_poisson = 0.0;

    g_simulation->m_stiffness_attachment = 120;
    g_simulation->m_gravity_constant = 100;
    g_simulation->m_damping_coefficient = 0.001;

    g_simulation->m_iterations_per_frame = 10;
}

void TW_CALL AntTweakBarWrapper::saveSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->saveSettings();
}

void TW_CALL AntTweakBarWrapper::loadSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->loadSettings();
    //resetSimulation(NULL);
}

void TW_CALL AntTweakBarWrapper::setDefaultSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->defaultSettings();
    //resetSimulation(NULL);
}