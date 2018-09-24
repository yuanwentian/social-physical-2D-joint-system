//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <iostream>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>


#include "mujoco.h"
#include "glfw3.h"

#include "SocialPhysical/dynamic.hpp"


#define random(x)(rand()%x)

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
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
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}




/////TRIAL: test wall contact


void InitializeForce(const mjModel* m, mjData* d)
{
d -> ctrl[0] = {0.00000001};
		d -> ctrl[1] = {0};

}




//main functon
int main(int argc, const char **argv)
{
    // check command-line arguments
    if (argc != 2)
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    mj_activate("mjkey.txt");
    // load and compile model
    char error[1000] = "Could not load binary model";
    if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
       { m = mj_loadModel(argv[1], 0);
        std::cout<<argv[1][0]<<std::endl;}
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                 // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100); // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // print some arguments
    // m -> nu = 2;    
    // run main loop, target real-time simulation and 60 fps rendering
int timeCounter=50;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
/*         while ((d->time - simstart < 1.0 / 60.0) && (d->time < 11))
        {
            mj_step1(m, d);
            applyForceOnModel(m, d);
            std::cout << m->nu << "  " << d->ctrl[0] << std::endl;
            std::cout << d->time << std::endl;
            mj_step2(m, d);
        } */



         while ((d->time - simstart < 1.0 / 200.0) && (d->time < 1000))
        {
     
            mj_step1(m, d);
            std::cout << "-------------------------------------" << std::endl; 
  	    std::cout << "simulation time: " << d->time << std::endl; 
	    std::cout << "qpos: " << d->qpos[0] <<  " " << d->qpos[1] <<" " << d->qpos[2] <<" " << d->qpos[3] << " " << d->qpos[4] << " " << d->qpos[5] << std::endl; 
	    std::cout << "qvel: " << d->qvel[0] <<  " " << d->qvel[1] <<" " << d->qvel[2] <<" " << d->qvel[3] << " " << d->qvel[4] << " " << d->qvel[5] << std::endl; 
            std::cout << "d->ctrl:" << d->ctrl[0] << " " << d->ctrl[1] <<" " << d->ctrl[2] <<" " << d->ctrl[3] <<" " << d->ctrl[4] <<" " << d->ctrl[5] <<std::endl;
     	 
       

  	    //apply random force
	
  	    //apply follow force


   InitializeForce(m, d);

 	if (d -> time >30)
      {

	
		const double PredatorForce = 10;
		double PreyForceX = (random(100)-50.0)/0.25;
		double PreyForceY = (random(100)-50.0)/0.25;
		double DraggerForceX = -(random(100)-50.0)/0.25;
		double DraggerForceY = -(random(100)-50.0)/0.25;

		timeCounter= timeCounter + 1;
		const int Counterthreshold = 50;
		double xpredatorstart = 3.5;
		double ypredatorstart = 3.5;
		double  xDistance; // X distance between predator and prey
		double  yDistance; // Y distance between predator and prey
		double  l; // length between predator and prey
		double  cosPP; // length between predator and prey
		double  sinPP; // length between predator and prey
		double  PredatorForceX; // X distance between predator and prey
		double  PredatorForceY; // Y distance between predator and prey
		xDistance=((d->qpos[0]+xpredatorstart)-d->qpos[2]);
		yDistance=((d->qpos[1]+ypredatorstart)-d->qpos[3]);
		l=sqrt(pow(xDistance,2)+pow(yDistance,2));
		cosPP=xDistance/l;
		sinPP=yDistance/l;

		PredatorForceX=-PredatorForce*cosPP;
		PredatorForceY=-PredatorForce*sinPP;
		std::cout << "xDistance:" << xDistance << " " << "yDistance:"<< yDistance <<" " << "l:"<< l <<std::endl;
		std::cout << "xForce:" << PredatorForceX << " " << "yForce:"<< PredatorForceY <<std::endl;
	        std::cout << "timeCounter:" << timeCounter <<std::endl;

   	     
		d -> ctrl[0] = {PredatorForceX};
		d -> ctrl[1] = {PredatorForceY};

			  d -> ctrl[2] = {PreyForceX};
			  d -> ctrl[3] = {PreyForceY};
			if (timeCounter % Counterthreshold ==0)
			{
			  d -> ctrl[4] = {DraggerForceX};
			  d -> ctrl[5] = {DraggerForceX};
			}
			else
			{
 			  d -> ctrl[4] = {0};
			  d -> ctrl[5] = {0};
			}
            mj_step2(m, d);

            if (xDistance < 0.4 || yDistance < 4)
			{
			  d -> time = d -> time - 2;
			}
	}			
            std::cout << "-------------------------------------" << std::endl;
           

        } 
           


        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}
