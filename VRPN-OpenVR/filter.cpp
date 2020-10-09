#include "filter.h"
#include <math.h>

static void QuatSlerp(q_type from, q_type to, float t, q_type& res)
{
    q_type to1;
    double omega, cosom, sinom, scale0, scale1;

    // calc cosine
    cosom = from[0] * to[0] + from[1] * to[1] + from[2] * to[2] + from[3] * to[3];

    // adjust signs (if necessary)
    if (cosom < 0.0)
    {
        cosom = -cosom; 
        to1[0] = -to[0];
        to1[1] = -to[1];
        to1[2] = -to[2];
        to1[3] = -to[3];
    }
    else 
    {
        to1[0] = to[0];
        to1[1] = to[1];
        to1[2] = to[2];
        to1[3] = to[3];
    }

    // calculate coefficients
    if ((1.0 - cosom) > 0.0005)
    {
        // standard case (slerp)
        omega = acos(cosom);
        sinom = sin(omega);
        scale0 = sin((1.0 - t) * omega) / sinom;
        scale1 = sin(t * omega) / sinom;
    }
    else
    {
        // "from" and "to" quaternions are very close
        // ... so we can do a linear interpolation
        scale0 = 1.0 - t;
        scale1 = t;
    }

    // calculate final values
    res[0] = scale0 * from[0] + scale1 * to1[0];
    res[1] = scale0 * from[1] + scale1 * to1[1];
    res[2] = scale0 * from[2] + scale1 * to1[2];
    res[3] = scale0 * from[3] + scale1 * to1[3];
}


filter_median::filter_median(int sz)
{
    window = sz;
    samples = 0;
}

void filter_median::process_data(q_vec_type& pos, q_type& rot)
{

}

filter_avg::filter_avg(int sz)
{
    window = sz;
    samples = 0;
}

void filter_avg::process_data(q_vec_type& pos, q_type& rot)
{
    int i, j;
    q_vec_type tmp;

    if (samples < window)
    {
        q_vec_copy(poses[samples], pos);
        samples++;
        return;
    }

    for (i = 0; i < (window - 1); i++)
        q_vec_copy(poses[i], poses[i + 1]);
    q_vec_copy(poses[window - 1], pos);

    for (j = 0; j < 3; j++)
    {
        for (i = 0, tmp[j] = 0; i < window; i++)
            tmp[j] += poses[i][j];
        tmp[j] /= (double)window;
    }

    q_vec_copy(pos, tmp);
}

filter_exp1::filter_exp1(double a_pos, double a_rot)
{
    alpha_pos = a_pos;
    alpha_rot = a_rot;
    samples = 0;
}

void filter_exp1::process_data(q_vec_type& pos, q_type& rot_quat)
{
    int i;
    q_vec_type pos_tmp;
    q_type rot_tmp;

    if (!samples)
    {
        samples++;
        q_vec_copy(pos_prev, pos);
        q_copy(rot_prev, rot_quat);
        return;
    }

    for (i = 0; i < 3; i++)
        pos_tmp[i] = alpha_pos * pos[i] + (1.0 - alpha_pos) * pos_prev[i];
    q_vec_copy(pos, pos_tmp);
    q_vec_copy(pos_prev, pos_tmp);

    QuatSlerp(rot_prev, rot_quat, alpha_rot, rot_tmp);
    q_normalize(rot_quat, rot_tmp);
    q_copy(rot_prev, rot_quat);
}

filter_kalman::filter_kalman(double _pos_E_est, double _pos_E_mea)
{
    pos_E_est[0] = pos_E_est[1] = pos_E_est[2] = _pos_E_est;
    pos_E_mea[0] = pos_E_mea[1] = pos_E_mea[2] = _pos_E_mea;
    samples = 0;
}

void filter_kalman::process_data(q_vec_type& pos, q_type& rot_quat)
{
    int i;
    q_vec_type KG, pos_EST;

    if (samples == 0)
    {
        samples++;
        q_vec_copy(pos_EST_prev, pos);
        return;
    }

    for (i = 0; i < 3; i++)
    {
        KG[i] = pos_E_est[i] / (pos_E_est[i] + pos_E_mea[i]);

        pos_EST[i] = pos_EST_prev[i] + KG[i] * (pos[i] - pos_EST_prev[i]);

        pos_E_est[i] = (1 - KG[i]) * pos_E_est[i];
    };

    q_vec_copy(pos, pos_EST);
    q_vec_copy(pos_EST_prev, pos_EST);
}

filter_exp1dyn::filter_exp1dyn(double a_pos, double d_pos)
{
    k = -log(1.0 - a_pos) / d_pos;
    samples = 0;
}

void filter_exp1dyn::process_data(q_vec_type& pos, q_type& rot_quat)
{
    int i;
    double d, alpha_pos;
    q_vec_type pos_tmp;

    if (!samples)
    {
        samples++;
        q_vec_copy(pos_prev, pos);
        return;
    }

    for (d = 0.0, i = 0; i < 3; i++)
        d += (pos[i] - pos_prev[i]) * (pos[i] - pos_prev[i]);
    d = sqrt(d);

    alpha_pos = (1 - exp(-k * d));

    for (i = 0; i < 3; i++)
        pos_tmp[i] = alpha_pos * pos[i] + (1.0 - alpha_pos) * pos_prev[i];

    q_vec_copy(pos, pos_tmp);
    q_vec_copy(pos_prev, pos_tmp);
#if 0
    q_to_euler(rot, rot_quat);
    for (i = 0; i < 3; i++)
        rot_tmp[i] = alpha_rot * rot[i] + (1.0 - alpha_rot) * rot_prev[i];
    q_from_euler(rot_quat, rot_tmp[0], rot_tmp[1], rot_tmp[2]);
    q_copy(rot_prev, rot_tmp);
#endif
}

filter_exp1pasha::filter_exp1pasha(double a, double b)
{
    alpha[0] = alpha[1] = alpha[2] = a;
    betta = b;
    samples = 0;
}

void filter_exp1pasha::process_data(q_vec_type& pos, q_type& rot_quat)
{
    int i;
    double d, alpha_pos;
    q_vec_type pos_tmp;

    if (!samples)
    {
        samples++;
        q_vec_copy(pos_prev, pos);
        return;
    }

    for (i = 0; i < 3; i++)
    {
        double t;
        pos_tmp[i] = alpha[i] * pos[i] + (1.0 - alpha[i]) * pos_prev[i];

        t = fabs(pos[i] - pos_tmp[i]);

        alpha[i] = betta * t + (1 - betta) * alpha[i];
//        alpha[i] *= 0.01;
    };

    q_vec_copy(pos, pos_tmp);
    q_vec_copy(pos_prev, pos_tmp);
}

