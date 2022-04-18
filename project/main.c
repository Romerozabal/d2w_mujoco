#include<stdbool.h> //for bool
#include <iostream>
//#include<unistd.h> //for usleep
#include <math.h>

#include "mujoco.h"
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// Joint numbers according to MJMODEL.TXT
#define WINCH_0_JOINT 0
#define WINCH_1_JOINT 3
#define WINCH_2_JOINT 6
#define WINCH_3_JOINT 9
#define RIGHT_ELBOW_N 29
#define LEFT_ELBOW_N 32

#define WINCH_0_BODY 30
#define WINCH_1_BODY 57
#define WINCH_2_BODY 84
#define WINCH_3_BODY 111

#define MASTIL_0_BODY 31
#define MASTIL_1_BODY 58
#define MASTIL_2_BODY 85
#define MASTIL_3_BODY 112

const float A0[3] = { 0.32 , 0.32, 0.94};
const float A1[3] = {-0.32 , 0.32, 0.94};
const float A2[3] = {-0.32 , -0.32, 0.94};
const float A3[3] = { 0.32 , -0.32, 0.94};

bool winches_positioned = 0;
bool winches_positioned_x = 0;
bool winches_positioned_y = 0;
bool winches_positioned_z = 0;

char filename[] = "../project/models/d2w_cables.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

int calibrated = 0;
int rotate = 1;
int threshold = 300;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void tensionCables() {
    
    if (calibrated == 0) {
        d->ctrl[WINCH_0_JOINT]= d->ctrl[WINCH_0_JOINT] + 0.1;
        d->ctrl[WINCH_1_JOINT]= d->ctrl[WINCH_1_JOINT] - 0.1;
        d->ctrl[WINCH_2_JOINT]= d->ctrl[WINCH_2_JOINT] - 0.1;
        d->ctrl[WINCH_3_JOINT]= d->ctrl[WINCH_3_JOINT] + 0.1;
        if ((abs(d->ctrl[WINCH_0_JOINT]) > threshold))
            calibrated = 1;

    }
    
}

void rotatePelvis() {
    if (calibrated == 1) {
        
        int sign = 1;
        if (rotate) {
            sign = 1;
            if (abs(d->ctrl[WINCH_0_JOINT]) > threshold + 20)
                rotate = 0;
        } else {
            sign = -1;
            if (abs(d->ctrl[WINCH_0_JOINT]) < threshold - 20)
                rotate = 1;
        }
        d->ctrl[WINCH_0_JOINT]= d->ctrl[WINCH_0_JOINT] + 0.1 * sign;
        d->ctrl[WINCH_1_JOINT]= d->ctrl[WINCH_1_JOINT] - 0.1 * sign;
        d->ctrl[WINCH_2_JOINT]= d->ctrl[WINCH_2_JOINT] - 0.1 * sign;
        d->ctrl[WINCH_3_JOINT]= d->ctrl[WINCH_3_JOINT] + 0.1 * sign;
        

    }


}

void mycontroller(const mjModel* m, mjData* d) {

    
    d->ctrl[RIGHT_ELBOW_N] = 0.25;    
    d->ctrl[LEFT_ELBOW_N] = 0.25;

    // Positioning Winches routine:
    if (!winches_positioned) {
        printf("{winch (x,y,z): [%.4f , %.4f , %.4f], A0: [%.4f , %.4f , %.4f]};\n",d->xpos[WINCH_0_BODY * 3], d->xpos[WINCH_0_BODY * 3 + 1], d->xpos[WINCH_0_BODY * 3 + 2], A0[0],A0[1], A0[2]);
        printf("{winch (x,y,z): [%.4f , %.4f , %.4f], A1: [%.4f , %.4f , %.4f]};\n",d->xpos[WINCH_1_BODY * 3], d->xpos[WINCH_1_BODY * 3 + 1], d->xpos[WINCH_1_BODY * 3 + 2], A1[0],A1[1], A1[2]);
        printf("{winch (x,y,z): [%.4f , %.4f , %.4f], A2: [%.4f , %.4f , %.4f]};\n",d->xpos[WINCH_2_BODY * 3], d->xpos[WINCH_2_BODY * 3 + 1], d->xpos[WINCH_2_BODY * 3 + 2], A2[0],A2[1], A2[2]);
        printf("{winch (x,y,z): [%.4f , %.4f , %.4f], A3: [%.4f , %.4f , %.4f]};\n",d->xpos[WINCH_3_BODY * 3], d->xpos[WINCH_3_BODY * 3 + 1], d->xpos[WINCH_3_BODY * 3 + 2], A3[0],A3[1], A3[2]);

        
        if (!winches_positioned_x) {
            if (d->xpos[WINCH_0_BODY * 3] < A0[0]) {
                m->body_pos[MASTIL_0_BODY * 3] =  m->body_pos[MASTIL_0_BODY * 3] - 0.0003;
                m->body_pos[MASTIL_0_BODY * 3 + 2] =  m->body_pos[MASTIL_0_BODY * 3 + 2] - 0.00015;
                m->body_pos[MASTIL_3_BODY * 3] =  m->body_pos[MASTIL_3_BODY * 3] - 0.0003;
                m->body_pos[MASTIL_3_BODY * 3 + 2] =  m->body_pos[MASTIL_3_BODY * 3 + 2] + 0.00015;
            }
            if (d->xpos[WINCH_1_BODY * 3] > A1[0]) {
                m->body_pos[MASTIL_1_BODY * 3] =  m->body_pos[MASTIL_1_BODY * 3] - 0.0003;
                m->body_pos[MASTIL_1_BODY * 3 + 2] =  m->body_pos[MASTIL_1_BODY * 3 + 2] - 0.00015;
                m->body_pos[MASTIL_2_BODY * 3] =  m->body_pos[MASTIL_2_BODY * 3] - 0.0003;
                m->body_pos[MASTIL_2_BODY * 3 + 2] =  m->body_pos[MASTIL_2_BODY * 3 + 2] + 0.00015;
            }
            if (d->xpos[WINCH_0_BODY * 3] > A0[0]) {
                m->body_pos[MASTIL_0_BODY * 3] =  m->body_pos[MASTIL_0_BODY * 3] + 0.0003;
                m->body_pos[MASTIL_0_BODY * 3 + 2] =  m->body_pos[MASTIL_0_BODY * 3 + 2] + 0.00015;
                m->body_pos[MASTIL_3_BODY * 3] =  m->body_pos[MASTIL_3_BODY * 3] + 0.0003;
                m->body_pos[MASTIL_3_BODY * 3 + 2] =  m->body_pos[MASTIL_3_BODY * 3 + 2] - 0.00015;
            }  
            if (d->xpos[WINCH_1_BODY * 3] < A1[0]) { 
                m->body_pos[MASTIL_1_BODY * 3] =  m->body_pos[MASTIL_1_BODY * 3] + 0.0003;
                m->body_pos[MASTIL_1_BODY * 3 + 2] =  m->body_pos[MASTIL_1_BODY * 3 + 2] + 0.00015;
                m->body_pos[MASTIL_2_BODY * 3] =  m->body_pos[MASTIL_2_BODY * 3] + 0.0003;
                m->body_pos[MASTIL_2_BODY * 3 + 2] =  m->body_pos[MASTIL_2_BODY * 3 + 2] - 0.00015;
            }
        }        
        
        if (!winches_positioned_y) {
            if (d->xpos[WINCH_0_BODY * 3 + 1] < A0[1]) {
                m->body_pos[MASTIL_0_BODY * 3 + 2] =  m->body_pos[MASTIL_0_BODY * 3 + 2] + 0.0003;
                m->body_pos[MASTIL_0_BODY * 3] =  m->body_pos[MASTIL_0_BODY * 3] - 0.00015;
                m->body_pos[MASTIL_1_BODY * 3 + 2] =  m->body_pos[MASTIL_1_BODY * 3 + 2] + 0.0003;
                m->body_pos[MASTIL_1_BODY * 3] =  m->body_pos[MASTIL_1_BODY * 3] - 0.00015;
            }

            if (d->xpos[WINCH_2_BODY * 3 + 1] > A2[1]) {
                std::cout << "MENOR y" << std::endl;
                m->body_pos[MASTIL_2_BODY * 3 + 2] =  m->body_pos[MASTIL_2_BODY * 3 + 2] - 0.0003;
                m->body_pos[MASTIL_2_BODY * 3] =  m->body_pos[MASTIL_2_BODY * 3] - 0.00015;
                m->body_pos[MASTIL_3_BODY * 3 + 2] =  m->body_pos[MASTIL_3_BODY * 3 + 2] - 0.0003;
                m->body_pos[MASTIL_3_BODY * 3] =  m->body_pos[MASTIL_3_BODY * 3] - 0.00015;
            }

            if (d->xpos[WINCH_0_BODY * 3 + 1] > A0[1]) {
                m->body_pos[MASTIL_0_BODY * 3 + 2] =  m->body_pos[MASTIL_0_BODY * 3 + 2] - 0.0003;
                m->body_pos[MASTIL_0_BODY * 3] =  m->body_pos[MASTIL_0_BODY * 3] + 0.00015;
                m->body_pos[MASTIL_1_BODY * 3 + 2] =  m->body_pos[MASTIL_1_BODY * 3 + 2] - 0.0003;
                m->body_pos[MASTIL_1_BODY * 3] =  m->body_pos[MASTIL_1_BODY * 3] + 0.00015;
            }

            if (d->xpos[WINCH_2_BODY * 3 + 1] < A2[1]) {
                std::cout << "MAYOR y" << std::endl;

                m->body_pos[MASTIL_2_BODY * 3 + 2] =  m->body_pos[MASTIL_2_BODY * 3 + 2] + 0.0003;
                m->body_pos[MASTIL_2_BODY * 3] =  m->body_pos[MASTIL_2_BODY * 3] + 0.00015;
                m->body_pos[MASTIL_3_BODY * 3 + 2] =  m->body_pos[MASTIL_3_BODY * 3 + 2] + 0.0003;
                m->body_pos[MASTIL_3_BODY * 3] =  m->body_pos[MASTIL_3_BODY * 3] + 0.00015;
            }          
        }   
        
        
        // Positioning in z axis:
        if (!winches_positioned_z) {
            if (d->xpos[WINCH_0_BODY * 3 + 2] < A0[2]) {
                m->body_pos[MASTIL_0_BODY * 3 + 1] =  m->body_pos[MASTIL_0_BODY * 3 + 1] - 0.0003;
                m->body_pos[MASTIL_1_BODY * 3 + 1] =  m->body_pos[MASTIL_1_BODY * 3 + 1] + 0.0003;
                m->body_pos[MASTIL_2_BODY * 3 + 1] =  m->body_pos[MASTIL_2_BODY * 3 + 1] + 0.0003;
                m->body_pos[MASTIL_3_BODY * 3 + 1] =  m->body_pos[MASTIL_3_BODY * 3 + 1] - 0.0003;
            }
            if (d->xpos[WINCH_0_BODY * 3 + 2] > A0[2]) {
                m->body_pos[MASTIL_0_BODY * 3 + 1] =  m->body_pos[MASTIL_0_BODY * 3 + 1] + 0.0003;
                m->body_pos[MASTIL_1_BODY * 3 + 1] =  m->body_pos[MASTIL_1_BODY * 3 + 1] - 0.0003;
                m->body_pos[MASTIL_2_BODY * 3 + 1] =  m->body_pos[MASTIL_2_BODY * 3 + 1] - 0.0003;
                m->body_pos[MASTIL_3_BODY * 3 + 1] =  m->body_pos[MASTIL_3_BODY * 3 + 1] + 0.0003;
            }         
        }
        
        if ((d->xpos[WINCH_0_BODY * 3] > A0[0] - 0.003) && (d->xpos[WINCH_0_BODY * 3] < A0[0] + 0.003) && (d->xpos[WINCH_1_BODY * 3] > A1[0] - 0.003) && (d->xpos[WINCH_1_BODY * 3] < A1[0] + 0.003)) {
            winches_positioned_x = 1;
        } else {
            winches_positioned_x = 0;
        }
        if ((d->xpos[WINCH_0_BODY * 3 + 1] > A0[1] - 0.003) && (d->xpos[WINCH_0_BODY * 3 + 1] < A0[1] + 0.003) && (d->xpos[WINCH_2_BODY * 3 + 1] > A2[1] - 0.003) && (d->xpos[WINCH_2_BODY * 3 + 1] < A2[1] + 0.003)) {
            winches_positioned_y = 1;
        } else {
            winches_positioned_y = 0;
        }
        
        if ((d->xpos[WINCH_0_BODY * 3 + 2] > A0[2] - 0.003) && (d->xpos[WINCH_0_BODY * 3 + 2] < A0[2] + 0.003)) {
            winches_positioned_z = 1;
        } else {
            winches_positioned_z = 0;
        }
   
        if (winches_positioned_x == 1 && winches_positioned_y == 1 && winches_positioned_z == 1) {
            winches_positioned = 1;
            printf("[MuJoCo] Winches positioned!\n");
        }
        
    } else {
        // Control motors
        tensionCables();
        rotatePelvis();
    }

    // Orientation

    
}
    //d->ctrl[1] = (-10 * (d->sensordata[1] - 1) - 1 * d->sensordata[1]);// PD control with noisy feedback
    //printf("{position: %f,force : %f, control: %f};\n",d->qpos[0],d->act_dot[0], d->ctrl[0]);


// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {113.400000, -30.857143, 4.265978, -0.083944, 0.692119, 0.645915};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    //d->qpos[1] = 200;
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjcb_control = mycontroller;
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
            // drag force = -c*v
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Display the world axes
        opt.frame = mjFRAME_WORLD;

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
