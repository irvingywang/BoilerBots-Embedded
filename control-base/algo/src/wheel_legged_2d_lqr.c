#include "wheel_legged_2d_Lqr.h"
#include <stdint.h>
#include "robot.h"
#define LQR11 (-9.0381f)
#define LQR12 (-0.8002f)
#define LQR13 (-1.5454f)
#define LQR14 (-1.8952f)
#define LQR15 (7.0606f)
#define LQR16 (0.8181f)
#define LQR21 (29.8781f)
#define LQR22 (3.7447f)
#define LQR23 (11.3292f)
#define LQR24 (12.9867f)
#define LQR25 (25.9207f)
#define LQR26 (0.8019f)

#define OPENSOURCE_COORDINATE
//    Q=diag([0.1 0.1 100 50 5000 1]);
//    R=diag([150 5]);  
// float lqr_poly_fit_param[12][4] = 
// {{-146.8397,178.1973,-83.7881,2.8443},
// {8.5304,-3.7974,-3.5802,0.2173},
// {-40.5217,39.2950,-13.5266,0.9230},
// {-57.9757,56.6803,-19.9446,1.3387},
// {195.0357,-122.9509,9.7593,6.2244},
// {28.3143,-20.0813,3.1549,0.5661},
// {1473.2038,-1142.9067,247.5109,5.8574},
// {105.5269,-94.7226,27.2942,-0.5018},
// {135.5466,-83.0411,4.9135,4.6654},
// {206.4120,-129.2789,9.6747,6.7037},
// {1603.1755,-1567.6944,545.5435,-38.7507},
// {130.3261,-137.4586,52.2075,-5.0431}};

// Q=diag([0.1 0.1 100 50 5000 1]);
// R=[150 0;0 5];
// float lqr_poly_fit_param[12][4] = 
// {{-135.3453,158.9821,-75.0086,2.0930},
// {4.8803,-1.4456,-3.8973,0.1909},
// {-37.5561,35.2304,-11.7066,0.6412},
// {-54.4328,51.3769,-17.4431,0.9291},
// {138.9470,-77.9373,-2.4892,7.9823},
// {21.5725,-14.9521,2.0154,0.6933},
// {1117.0967,-875.1741,193.5456,7.1032},
// {85.3890,-76.2583,21.8655,-0.1857},
// {65.3828,-28.1241,-8.2274,5.3604},
// {103.0676,-47.8553,-10.0767,7.8005},
// {1622.2835,-1555.2473,533.1584,-34.9396},
// {129.6584,-132.4556,49.3328,-4.6592}};


//    Q=diag([0.1 0.1 100 500 5000 1]);
//    R=[150 0;0 5];
// float lqr_poly_fit_param[12][4] = 
// {{-75.0118,112.1647,-70.4753,1.8061},
// {12.9513,-8.7708,-3.6872,0.1924},
// {-26.5521,26.3310,-9.4944,0.5264},
// {-68.3177,68.2261,-25.1308,1.3668},
// {130.5731,-82.6278,3.4079,7.5875},
// {18.2980,-13.4825,2.1852,0.6954},
// {1191.5062,-1006.2760,259.2660,5.5796},
// {96.2733,-95.5380,33.4083,-0.3565},
// {65.3363,-35.3815,-3.4411,4.9918},
// {179.6229,-101.0996,-6.1644,13.0344},
// {1158.9919,-1180.8042,442.3189,-31.6758},
// {93.3741,-102.2150,41.9548,-4.6102}};


//    Q=diag([0.1 0.1 100 1600 5000 1]);
//    R=[100 0;0 5];

extern Robot_State_t g_robot_state;
float lqr_poly_fit_param[12][4] = 
{{19.9177,45.6310,-85.1880,2.5162},
{35.9334,-32.3786,-3.1289,0.2562},
{-181.5150,190.3658,-74.8042,5.0700},
{-123.9630,136.1380,-59.5809,3.8028},
{138.3874,-96.2568,9.6982,7.3913},
{18.1576,-13.6580,2.2540,0.7722},
{1455.6560,-1353.9214,419.4314,4.6622},
{99.7005,-120.7792,58.5563,-0.8040},
{552.5246,-372.2155,27.5695,31.9137},
{492.1016,-358.0164,47.7693,24.7283},
{836.5175,-886.2784,353.5117,-26.5192},
{87.4363,-95.2419,39.5887,-4.6017}};

// K gain for testing is 
// -35.3109   -5.1495  -20.2524  -16.1035   31.5393    2.3398
// 26.6050    4.2699   18.9568   14.4032  127.6940    4.3058
// float K[2][6] = {
//     {-35.3109f, -5.1495f, -20.2524f, -16.1035f, 31.5393f, 2.3398f},
//     {26.6050f, 4.2699f, 18.9568f, 14.4032f, 127.6940f, 4.3058f}
// };
/**
 * -6.9854   -0.2968   -0.0559   -0.2264    0.6045    0.2095
    5.7354    0.2646    0.0867    0.3415    1.2279    0.4110
 */
// float  K[2][6] = {
//     {-6.9854f, -0.2968f, -0.0559f, -0.2264f, 0.6045f, 0.2095f},
//     {5.7354f, 0.2646f, 0.0867f, 0.3415f, 1.2279f, 0.4110f}
// };
// -7.6324   -0.4336   -0.5741   -0.8191    1.6837    0.3748
//     5.6826    0.4060    0.8257    1.1351    4.2033    0.7740

// float K[2][6] = {
//     {-7.6324f, -0.4336f, -0.5741f, -0.8191f, 1.6837f, 0.3748f},
//     {5.6826f, 0.4060f, 0.8257f, 1.1351f, 4.2033f, 0.7740f}
// };

/**
 * -9.4858   -0.7030   -1.3617   -1.5593    1.6578    0.3881
    1.9850    0.1448    0.3816    0.4109    5.0654    0.9650
 */

//  float K[2][6] = {
//      {-9.4858f, -0.7030f, -1.3617f, -1.5593f, 1.6578f, 0.3881f},
//      {1.9850f, 0.1448f, 0.3816f, 0.4109f, 5.0654f, 0.9650f}
//  };

/**
 * -8.7832   -0.6367   -1.1696   -1.3449    2.9224    0.5229
    6.9396    0.6375    1.7778    1.9441    8.7630    1.1277
 */
// float K[2][6] = {
//     {-8.7832f, -0.6367f, -1.1696f, -1.3449f, 2.9224f, 0.5229f},
//     {6.9396f, 0.6375f, 1.7778f, 1.9441f, 8.7630f, 1.1277f}
// };

/**
 *   -9.1898   -0.7124   -1.3647   -1.5452    1.6226    0.3792
    1.9255    0.1499    0.3710    0.3949    5.0749    0.9670
 */
// float K[2][6] = {
//     {-9.1898f, -0.7124f, -1.3647f, -1.5452f, 1.6226f, 0.3792f},
//     {1.9255f, 0.1499f, 0.3710f, 0.3949f, 5.0749f, 0.9670f}
// };

/**
 * -9.1053   -0.6279   -0.9940   -1.2618    0.1912    0.0803
    0.4373   -0.0061   -0.0445   -0.0946    1.6421    0.5623

 */

float K[2][6] = {
    {-9.1053f, -0.6279f, -0.9940f, -1.2618f, 0.1912f, 0.0803f},
    {0.4373f, -0.0061f, -0.0445f, -0.0946f, 1.6421f, 0.5623f}
};

 void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
// float len = state->leg_len;
// float len_sqrt = len * len;
// float len_cub = len * len * len;
// if (g_robot_state.spintop_mode){
//     u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
//              (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
//              (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) * 0 +
//              (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
//              (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * (-state->phi) +
//              (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
//     u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
//              (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
//              (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) * 0+
//              (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
//              (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * (-state->phi) +
//              (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;

// }
// else
// {
    // u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
    //          (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
    //          (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) +
    //          (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
    //          (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * (-state->phi) +
    //          (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
    // u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
    //          (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
    //          (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) +
    //          (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
    //          (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * (-state->phi) +
    //          (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;
// }

u->T_A = -K[0][0] * state->theta 
        - K[0][1] * state->theta_dot
        - K[0][2] * (state->x)
        - K[0][3] * (state->x_dot)
        - K[0][4] * state->phi
        - K[0][5] * state->phi_dot;
u->T_B = K[1][0] * state->theta
        - K[1][1] * state->theta_dot
        - K[1][2] * (state->x)
        - K[1][3] * (state->x_dot)
        - K[1][4] * state->phi
        - K[1][5] * state->phi_dot;
}