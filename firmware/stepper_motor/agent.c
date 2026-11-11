#include <math.h>
#include "agent.h"
#include "stepper.h" // TH_LIMIT, VMAX
#include "weights.h"

#define A_DOT_DENOM 20.0f // env.py: a_dot_denom

float controller(float a, float a_dot, float th, float th_dot){


    float state[OBS_DIM] = { sinf(a), cosf(a), th/TH_LIMIT, a_dot/A_DOT_DENOM, th_dot/VMAX };


    static float hidden1[HID_DIM];
    static float hidden2[HID_DIM];
    
    // 1. First Hidden Layer (latent_pi.0) + ReLU (latent_pi.1)
    for (int i = 0; i < HID_DIM; ++i) {
        hidden1[i] = latent_pi_0_bias[i];
        for (int j = 0; j < OBS_DIM; ++j) {
            hidden1[i] += state[j] * latent_pi_0_weight[i * OBS_DIM + j];
        }
        hidden1[i] = hidden1[i] > 0.0f ? hidden1[i] : 0.0f; // ReLU
    }
    
    // 2. Second Hidden Layer (latent_pi.2) + ReLU (latent_pi.3)
    for (int i = 0; i < HID_DIM; ++i) {
        hidden2[i] = latent_pi_2_bias[i];
        for (int j = 0; j < HID_DIM; ++j) {
            hidden2[i] += hidden1[j] * latent_pi_2_weight[i * HID_DIM + j];
        }
        hidden2[i] = hidden2[i] > 0.0f ? hidden2[i] : 0.0f; // ReLU
    }
    
    // 3. Output Mean Layer (mu)
    float mu = mu_bias[0];
    for (int j = 0; j < HID_DIM; ++j) {
        mu += hidden2[j] * mu_weight[j];
    }
    
    // 4. Squashed Gaussian 
    float action = tanhf(mu);
    return action;
}