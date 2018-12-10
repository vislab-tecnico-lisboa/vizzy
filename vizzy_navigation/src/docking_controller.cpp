#include <docking_controller.hpp>

DockingController::DockingController(float k_ro, float k_alpha, float k_beta) : k_ro_(k_ro), 
        k_alpha_(k_alpha), k_beta_(k_beta)
{

}

DockingController::~DockingController(){}


void DockingController::updateGains(float k_ro, float k_alpha, float k_beta)
{

}


float DockingController::computeControlSignal(float ro, float alpha, float beta)
{


}


