#ifndef DOCKING_CONTROLLER_HPP_
#define DOCKING_CONTROLLER_HPP_


class DockingController
{
private:
    float k_ro_;
    float k_alpha_;
    float k_beta_;

public:
    DockingController(float k_ro, float k_alpha, float k_beta);
    ~DockingController();
    void updateGains(float k_ro, float k_alpha, float k_beta);
    float computeControlSignal(float ro, float alpha, float beta);
};


#endif