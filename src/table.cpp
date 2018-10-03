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

#include "SocialPhysical/policy.hpp"



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
d -> ctrl[0] = {0.000000000001};
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

    int timeCounter=50; // to decide when to start to make demo convenient
    const int Counterthreshold = 10;  //to modify dragger frames
    const int catchThreshold = 0.25;
    int startTime=timeCounter/10;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    int positionIndex=-6;
    int velocityIndex=-6;
    int accelerationIndex=-6;
    int rejectionSamplingFrame=50;
    int rejectionSamplingCounter = 0;
    int renderIndex=0;
    const int NumberOfPosition = m -> nq;
    const int NumberOfVelocity = m -> nv;
    const int NumberOfControl = m -> nu;  // nq ; nv; nu all depends on number of joints
    const int NumberOfJointForEachBody = 2;


    const char* positionFilename = "position.txt";
    std::ofstream positionFile(positionFilename, std::ios::app);
    const char* velocityFilename = "velocity.txt";
    std::ofstream velocityFile(velocityFilename, std::ios::app);
    const char* accelerationFilename = "acceleration.txt";
    std::ofstream accelerationFile(accelerationFilename, std::ios::app);
    const char* trajectoryFilename = "trajectory.txt";
    std::ofstream trajectoryFile(trajectoryFilename, std::ios::app);

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
            
            if((d->time < 500))
            {
                mj_step1(m, d);
                    
                    positionIndex = positionIndex + NumberOfPosition * NumberOfJointForEachBody;   
                    velocityIndex = velocityIndex + NumberOfVelocity * NumberOfJointForEachBody;
                    accelerationIndex = accelerationIndex + NumberOfPosition * NumberOfJointForEachBody;
                    std::cout << "-------------------------------------" << std::endl; 
                    std::cout << "simulation time: " << d->time << std::endl; 
                    std::cout << "nq: " << NumberOfPosition << "nv: " << NumberOfVelocity <<"nu: " << NumberOfControl <<std::endl; 
                    std::cout << "qpos: " << d->qpos[0] <<  " " << d->qpos[1] <<" " << d->qpos[2] <<" " << d->qpos[3] << " " << d->qpos[4] << " " << d->qpos[5] << std::endl; 
                    std::cout << "qvel: " << d->qvel[0] <<  " " << d->qvel[1] <<" " << d->qvel[2] <<" " << d->qvel[3] << " " << d->qvel[4] << " " << d->qvel[5] << std::endl; 
                    std::cout << "d->ctrl:" << d->ctrl[0] << " " << d->ctrl[1] <<" " << d->ctrl[2] <<" " << d->ctrl[3] <<" " << d->ctrl[4] <<" " << d->ctrl[5] <<std::endl;
                    
                    for( int i = 0; i < 6; i = i + 1 )
                    {
                        position.push_back(d->qpos[i]);
                        velocity.push_back(d->qvel[i]);
                        acceleration.push_back(d->qacc[i]);
                    }

                    std::cout<<"position.size:"<<position.size()<<"     "<<std::endl;
                    ////print position
                //  for(int i=0;i<position.size();i++)  
                //  {  
                //   std::cout<<"position:"<<position[i]<<"     ";  
                //   }  
                //   std::cout<<std::endl; 

                    //apply random force
                
                    //apply follow force


                    InitializeForce(m, d);

                if (d -> time > startTime)
                    {

                
                    //	const double PredatorForce = 10; 
                    //	double PreyForceX = (random(100)-50.0)/0.25;
                    //	double PreyForceY = (random(100)-50.0)/0.25;
                    //	double DraggerForceX = -(random(100)-50.0)/0.25;
                    //	double DraggerForceY = -(random(100)-50.0)/0.25;

                    const int ForceRatio = 10;
                    const double PredatorForce = 1.5 * ForceRatio;
                    const double range_from  = 0;
                    const double range_to    = 100;
                    std::random_device                  rand_dev;
                    std::mt19937                        generator(rand_dev());
                    std::uniform_int_distribution<int>  distr(range_from, range_to);

                    std::cout << distr(generator) << '\n';
        
          
                    double PreyForceX = (distr(generator)-50.0)/5 * ForceRatio;
                    double PreyForceY = (distr(generator)-50.0)/5 * ForceRatio;
                    double DraggerForceX = -(distr(generator)-50.0)/25 * ForceRatio;
                    double DraggerForceY = -(distr(generator)-50.0)/25 * ForceRatio;

                    timeCounter= timeCounter + 1;
            
                    double xpredatorstart = 3.5;
                    double ypredatorstart = 3.5;


                    double  PPDistanceX; // X distance between predator and prey
                    double  PPDistanceY; // Y distance between predator and prey
                    double  PPl; // length between predator and prey
                    double  cosPP; // cos between predator and prey
                    double  sinPP; // sin between predator and prey
                    double  PredatorForceX; // X distance between predator and prey
                    double  PredatorForceY; // Y distance between predator and prey
                    PPDistanceX=((d->qpos[0]+xpredatorstart)-d->qpos[2]);
                    PPDistanceY=((d->qpos[1]+ypredatorstart)-d->qpos[3]);
                    PPl=sqrt(pow(PPDistanceX,2)+pow(PPDistanceY,2));
                    cosPP=PPDistanceX/PPl;
                    sinPP=PPDistanceY/PPl;

                    double xdraggerstart = 4.5;
                    double ydraggerstart = 4.5;
                    double  DPDistanceX; // X distance between dragger and prey
                    double  DPDistanceY; // Y distance between dragger and prey
                    double  DPl; // length between dragger and prey
                    double  cosDP; // cos between dragger and prey
                    double  sinDP; // sin between dragger and prey
                    DPDistanceX=((d->qpos[4]+xdraggerstart)-d->qpos[2]);
                    DPDistanceY=((d->qpos[5]+ydraggerstart)-d->qpos[3]);
                    DPl=sqrt(pow(DPDistanceX,2)+pow(DPDistanceY,2));

                    PredatorForceX=-PredatorForce*cosPP;
                    PredatorForceY=-PredatorForce*sinPP;
                    std::cout << "PPDistanceX:" << PPDistanceX << " " <<  "PPDistanceY:"<< PPDistanceY <<" " << "PPl:"<< PPl <<std::endl;
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
                    std::cout << "position.size"<<position.size()<<std::endl;
                    std::cout << "positionIndex"<<positionIndex<<std::endl;
                    std::cout << "velocity.size"<<velocity.size()<<std::endl;
                    std::cout << "velocityIndex"<<velocityIndex<<std::endl;
                    
                    //rejection sampling frames

                    
                    //if ((abs(PPDistanceX) < 0.25) || (abs PPDistanceY) < 0.25))
                    if ((PPl < 1) || (DPl< 1))
                    {
                                rejectionSamplingCounter = rejectionSamplingCounter + 1;
                                position.erase(position.end()-6*rejectionSamplingFrame,position.end());
                                velocity.erase(velocity.end()-6*rejectionSamplingFrame,velocity.end());
                                acceleration.erase(acceleration.end()-6*rejectionSamplingFrame,acceleration.end());   
                                positionIndex=positionIndex-6*rejectionSamplingFrame;
                                velocityIndex=velocityIndex-6*rejectionSamplingFrame;
                                accelerationIndex=accelerationIndex-6*rejectionSamplingFrame;         
                                //d -> time = d -> time - 0.01*rejectionSamplingFrame;
                                std::cout << "rejection sampling"<<std::endl;
                                std::cout << "position.size"<<position.size()<<std::endl;
                                std::cout << "positionIndex"<<positionIndex<<std::endl; 

                                for(int i=0;i<6;i++){
                                    d->qpos[i] = position[positionIndex+i];
                                    d->qvel[i] = velocity[velocityIndex+i];
                                    d->qacc[i] = acceleration[accelerationIndex+i];
                                    std::cout << "d->qpos" << i << ":" << d->qpos[i] << std::endl;
                                    std::cout << "position"<< i << ":" << position[positionIndex+i] << std::endl;
                                }

                            //  if (rejectionSamplingCounter % 50 == 0){                                    //rejectionSampilingCounter is used to double RejectionSampling if it is not working very well
                            //      rejectionSamplingFrame = rejectionSamplingFrame*2;
                            //    }
                            //      std::cout << "rejectionSamplingFrame"<< rejectionSamplingFrame << std::endl;
                            //      std::cout << "rejectionSamplingCounter"<< rejectionSamplingCounter << std::endl; 
                    }


                    }			
                    std::cout << "-------------------------------------" << std::endl;
            mj_step2(m, d);       
            }
            
            else{
           
                //std::cout << "simulation time: " << d->time << std::endl; 
                for ( renderIndex ; renderIndex < position.size(); renderIndex = renderIndex + 6){
                    mj_step1(m,d);
                    for (int i = 0; i <6; i++) {
                    std::cout << "-------------------------------------" << std::endl; 
                    std::cout << "simulation time: " << d->time << std::endl; 
                    std::cout << "renderIndex:" << renderIndex << std::endl;
                    std::cout << "position.size():" << position.size() << std::endl;
                    std::cout << "position:" << position[renderIndex+i] << std::endl;
                    positionFile << position[renderIndex+i] << " ";
                    velocityFile << velocity[renderIndex+i] << " ";
                    accelerationFile << acceleration[renderIndex+i] << " ";
                    
                    d->qpos[i] = position[renderIndex+i];
                    d->qvel[i] = velocity[renderIndex+i];
                    d->qacc[i] = acceleration[renderIndex+i];
                    
                    }
                    mj_step2(m,d);
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

                    positionFile << std::endl;
                    velocityFile << std::endl;
                    accelerationFile << std::endl;
                     
                }
                positionFile.close(); 
                velocityFile.close();  
                accelerationFile.close(); 
               
            }
            
           
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
