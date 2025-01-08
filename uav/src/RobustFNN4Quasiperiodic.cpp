//  created:    2024/12/13  /   2022/11/20
//  filename:   RobustFNN4Quasiperiodic.cpp
//
//  author:     Guillaume Sanahuja / Alejandro TEVERA RUIZ
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: 1$
//
//  purpose:    Robust FNN for quasiperiodic disturbances
//
//
/*********************************************************************/

#include "RobustFNN4Quasiperiodic.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <DoubleSpinBox.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Label.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <MatrixDescriptor.h>
#include <cmath>
#include <Tab.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <ComboBox.h>
#include <GroupBox.h>
#include <TabWidget.h>
#include <Tab.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


RobustFNN4Quasiperiodic::RobustFNN4Quasiperiodic(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());

    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        //targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        //targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        //targetVrpn=new MetaVrpnObject("target");
    }

    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
        SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

    GroupBox *groupbox = new GroupBox(GetButtonsLayout()->NewRow(), "Controles");
    
    
    Tab *lawTab2 = new Tab(getFrameworkManager()->GetTabWidget(), "custom control laws");
    TabWidget *tabWidget2 = new TabWidget(lawTab2->NewRow(), "laws");
    
    setupLawTab2 = new Tab(tabWidget2, "SMC setup");
    setupLawTab3 = new Tab(tabWidget2, "AFNNC setup");
    graphLawTab2 = new Tab(tabWidget2, "SMC graphs");
    graphLawTab3 = new Tab(tabWidget2, "AFNNC graphs");
    adaptationGraphsTab = new Tab(tabWidget2, "Adaptation graphs");

    positionTab = new Tab(tabWidget2, "Reference position");
    positiongTab = new Tab(tabWidget2, "Desired orientation");

    GroupBox *posbox = new GroupBox(positionTab->NewRow(), "Setup reference");
    GroupBox *regbox = new GroupBox(positionTab->NewRow(), "Setup regulation");
    GroupBox *trackbox = new GroupBox(positionTab->NewRow(), "Setup tracking");
    GroupBox *trajbox = new GroupBox(positionTab->NewRow(), "Setup trajectory");

    position_behavior = new ComboBox(posbox->NewRow(),"Select behavior");
    position_behavior->AddItem("Regulation");
    position_behavior->AddItem("Tracking");
    position_behavior->AddItem("Trajectory");

    xd_behavior = new ComboBox(trackbox->NewRow(),"Select xd behavior");
    xd_behavior->AddItem("Regulation");
    xd_behavior->AddItem("Sin");
    xd_behavior->AddItem("Cos");

    yd_behavior = new ComboBox(trackbox->LastRowLastCol(),"Select yd behavior");
    yd_behavior->AddItem("Regulation");
    yd_behavior->AddItem("Sin");
    yd_behavior->AddItem("Cos");

    zd_behavior = new ComboBox(trackbox->LastRowLastCol(),"Select zd behavior");
    zd_behavior->AddItem("Regulation");
    zd_behavior->AddItem("Sin");
    zd_behavior->AddItem("Cos");

    GroupBox *xdkbox = new GroupBox(trackbox->NewRow(), "xd");
    GroupBox *ydkbox = new GroupBox(trackbox->LastRowLastCol(), "yd");
    GroupBox *zdkbox = new GroupBox(trackbox->LastRowLastCol(), "zd");
    
    xd = new DoubleSpinBox(regbox->NewRow(), "x", " m", -2, 2, 0.1, 2);
    yd = new DoubleSpinBox(regbox->LastRowLastCol(), "y", " m", -2, 2, 0.1, 2);
    zd = new DoubleSpinBox(regbox->LastRowLastCol(), "z", " m", -2, 2, 0.1, 2);
    
    lx = new Label(xdkbox->NewRow(), "funcion");
    lx->SetText("a*fnc(w*t) + b");
    ax = new DoubleSpinBox(xdkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wx = new DoubleSpinBox(xdkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bx = new DoubleSpinBox(xdkbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    ly = new Label(ydkbox->NewRow(), "funcion");
    ly->SetText("a*fnc(w*t) + b");
    ay = new DoubleSpinBox(ydkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wy = new DoubleSpinBox(ydkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    by = new DoubleSpinBox(ydkbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    lz = new Label(zdkbox->NewRow(), "funcion");
    lz->SetText("a*fnc(w*t) + b");
    az = new DoubleSpinBox(zdkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wz = new DoubleSpinBox(zdkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bz = new DoubleSpinBox(zdkbox->NewRow(), "Offset (b)", -3, 0, 0.1, 2);


    control_select=new ComboBox(groupbox->NewRow(),"select control");
    control_select->AddItem("Sliding");
    control_select->AddItem("Fourier Controller");
    control_select->AddItem("Sliding Force-Position");
    
    l2 = new Label(groupbox->LastRowLastCol(), "Control selec");
    l2->SetText("Control: off");
    
    start_prueba1=new PushButton(groupbox->NewRow(),"start control");
    stop_prueba1=new PushButton(groupbox->NewRow(),"stop control");
    
    
    u_sliding = new Sliding(setupLawTab2->At(0, 0), "smc");
    u_sliding->UseDefaultPlot(graphLawTab2->At(0, 0));
    u_sliding->UseDefaultPlot2(graphLawTab2->At(0, 1));
    u_sliding->UseDefaultPlot3(graphLawTab2->At(0, 2));
    u_sliding->UseDefaultPlot4(graphLawTab2->At(1, 2));
    u_sliding->UseDefaultPlot5(graphLawTab2->At(1, 0));

    afnnc = new AFNNC(setupLawTab3->At(0, 0), "afnnc");
    afnnc->UseDefaultPlot(graphLawTab3->At(0, 0));
    afnnc->UseDefaultPlot2(graphLawTab3->At(0, 1));
    afnnc->UseDefaultPlot3(graphLawTab3->At(0, 2));
    afnnc->UseDefaultPlot4(graphLawTab3->At(1, 2));

    afnnc->UseDefaultPlot8(graphLawTab3->At(1, 0));
    afnnc->UseDefaultPlot9(graphLawTab3->At(1, 1));

    afnnc->UseDefaultPlot5(positiongTab->At(0, 0));
    afnnc->UseDefaultPlot6(positiongTab->At(0, 1));
    afnnc->UseDefaultPlot7(positiongTab->At(0, 2));

    afnnc->plotFNN(adaptationGraphsTab->At(0, 0));
    afnnc->plotMonitor(adaptationGraphsTab->At(0, 1));
    afnnc->plotAdapGamma(adaptationGraphsTab->At(1, 0));
    afnnc->plotError(adaptationGraphsTab->At(1, 1));
    
    
    customOrientation=new AhrsData(this,"orientation");

    AddDeviceToControlLawLog(u_sliding);
    AddDeviceToControlLawLog(afnnc);

}

RobustFNN4Quasiperiodic::~RobustFNN4Quasiperiodic() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void RobustFNN4Quasiperiodic::ComputeCustomTorques(Euler &torques) {
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();
    switch(control_select->CurrentIndex()) {
        case 0:
            //Printf("Fx:%f Fy:%f Fz:%f\n",jr3->GetFx(),jr3->GetFy(),jr3->GetFz());
            sliding_ctrl(torques);
            break;
        
        case 1:
            if(vrpnLost==true){
                Thread::Err("Fourier NN control can't start: VRPN lost\n");
            } else {
                run_afnnc(torques);
            }
            break;

    }
    
}

float RobustFNN4Quasiperiodic::ComputeCustomThrust(void) {
    return thrust;
}

void RobustFNN4Quasiperiodic::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::position_control))) {
      /*
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
       */
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void RobustFNN4Quasiperiodic::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::Stopped:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        vrpnLost=false;
        behaviourMode=BehaviourMode_t::Default;
        break;
    case Event_t::EnteringFailSafeMode:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        vrpnLost=false;
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}


void RobustFNN4Quasiperiodic::ExtraCheckPushButton(void) {
    if(start_prueba1->Clicked() && (behaviourMode!=BehaviourMode_t::position_control)) {
        StartRobustFNN4Quasiperiodic();
    }

    if(stop_prueba1->Clicked() && (behaviourMode==BehaviourMode_t::position_control)) {
        StopRobustFNN4Quasiperiodic();
    }
}

void RobustFNN4Quasiperiodic::ExtraCheckJoystick(void) {
    //R1
    if(GetTargetController()->IsButtonPressed(9) && (behaviourMode!=BehaviourMode_t::position_control)) {
        StartRobustFNN4Quasiperiodic();
    }
    //L1
    if(GetTargetController()->IsButtonPressed(6) && (behaviourMode==BehaviourMode_t::position_control)) {
        StopRobustFNN4Quasiperiodic();
    }
    
}


void RobustFNN4Quasiperiodic::StartRobustFNN4Quasiperiodic(void) {
    control_select->setEnabled(false);
    //ask UavStateMachine to enter in custom torques
    if (SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("RobustFNN4Quasiperiodic: start\n");
        u_sliding->Reset();
        afnnc->Reset();
    } else {
        Thread::Warn("RobustFNN4Quasiperiodic: could not start\n");
        l2->SetText("Control: err");
        control_select->setEnabled(true);
        return;
    }
    switch(control_select->CurrentIndex()) {
        case 0:
            l2->SetText("Control: Sliding");
            Thread::Info("Sliding\n");
            behaviourMode=BehaviourMode_t::control;
            break;
        
        case 1:
            l2->SetText("Control: Fourier NN");
            Thread::Info("AFNNC\n");
            behaviourMode=BehaviourMode_t::position_control;
            break;
    }
}

void RobustFNN4Quasiperiodic::StopRobustFNN4Quasiperiodic(void) {
    control_select->setEnabled(true);
    //just ask to enter fail safe mode
    l2->SetText("Control: off");
    //first_update==true;
    SetTorqueMode(TorqueMode_t::Default);
    SetThrustMode(ThrustMode_t::Default);
    behaviourMode=BehaviourMode_t::Default;
    EnterFailSafeMode();
}

void RobustFNN4Quasiperiodic::pos_reference(Vector3Df &xid, Vector3Df &xidp, Vector3Df &xidpp, Vector3Df &xidppp, float tactual){
    
    switch(position_behavior->CurrentIndex()){
    case 0:
        // regulation
        xid = Vector3Df(xd->Value(),yd->Value(),zd->Value());
        xidp = Vector3Df(0,0,0);
        xidpp = Vector3Df(0,0,0);
        xidppp = Vector3Df(0,0,0);
        break;
    
    case 1:
        // tracking
        switch(xd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.x = xd->Value();
            xidp.x = 0;
            xidpp.x = 0;
            xidppp.x = 0;
            break;
        case 1:
            // sin
            xid.x = ax->Value()*sin(wx->Value()*tactual)+bx->Value();
            xidp.x = ax->Value()*wx->Value()*cos(wx->Value()*tactual);
            xidpp.x = -ax->Value()*wx->Value()*wx->Value()*sin(wx->Value()*tactual);
            xidppp.x = -ax->Value()*wx->Value()*wx->Value()*wx->Value()*cos(wx->Value()*tactual);
            break;
        case 2:
            // cos
            xid.x = ax->Value()*cos(wx->Value()*tactual)+bx->Value();
            xidp.x = -ax->Value()*wx->Value()*sin(wx->Value()*tactual);
            xidpp.x = -ax->Value()*wx->Value()*wx->Value()*cos(wx->Value()*tactual);
            xidppp.x = ax->Value()*wx->Value()*wx->Value()*wx->Value()*sin(wx->Value()*tactual);
            break;
        }

        switch(yd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.y = yd->Value();
            xidp.y = 0;
            xidpp.y = 0;
            xidppp.y = 0;
            break;
        case 1:
            // sin
            xid.y = ay->Value()*sin(wy->Value()*tactual)+by->Value();
            xidp.y = ay->Value()*wy->Value()*cos(wy->Value()*tactual);
            xidpp.y = -ay->Value()*wy->Value()*wy->Value()*sin(wy->Value()*tactual);
            xidppp.y = -ay->Value()*wy->Value()*wy->Value()*wy->Value()*cos(wy->Value()*tactual);
            break;
        case 2:
            // cos
            xid.y = ay->Value()*cos(wy->Value()*tactual)+by->Value();
            xidp.y = -ay->Value()*wy->Value()*sin(wy->Value()*tactual);
            xidpp.y = -ay->Value()*wy->Value()*wy->Value()*cos(wy->Value()*tactual);
            xidppp.y = ay->Value()*wy->Value()*wy->Value()*wy->Value()*sin(wy->Value()*tactual);
            break;
        }

        switch(zd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.z = zd->Value();
            xidp.z = 0;
            xidpp.z = 0;
            xidppp.z = 0;
            break;
        case 1:
            // sin
            xid.z = az->Value()*sin(wz->Value()*tactual)+bz->Value();
            xidp.z = az->Value()*wz->Value()*cos(wz->Value()*tactual);
            xidpp.z = -az->Value()*wz->Value()*wz->Value()*sin(wz->Value()*tactual);
            xidppp.z = -az->Value()*wz->Value()*wz->Value()*wz->Value()*cos(wz->Value()*tactual);
            break;
        case 2:
            // cos
            xid.z = az->Value()*cos(wz->Value()*tactual)+bz->Value();
            xidp.z = -az->Value()*wz->Value()*sin(wz->Value()*tactual);
            xidpp.z = -az->Value()*wz->Value()*wz->Value()*cos(wz->Value()*tactual);
            xidppp.z = az->Value()*wz->Value()*wz->Value()*wz->Value()*sin(wz->Value()*tactual);
            break;
        }
        break;
    
    case 2:
        // trajectory
        break;
    default:
        xid = Vector3Df(0,0,0);
        xidp = Vector3Df(0,0,0);
        xidpp = Vector3Df(0,0,0);
        xidppp = Vector3Df(0,0,0);
        break;
    }
}


void RobustFNN4Quasiperiodic::sliding_ctrl(Euler &torques){
    //flair::core::Time ti = GetTime();
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
    //flair::core::Time  tf = GetTime()-ti;

    //Printf("ref: %f ms\n", (float)tf/1000000);

    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Quaternion uav_quat;

    

    //ti = GetTime();
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    //tf = tf = GetTime()-ti;

    //Printf("cur: %f ms\n",  (float)tf/1000000);
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    u_sliding->SetValues(ze,zp,currentAngularRates,refAngularRates,currentQuaternion,refQuaternion);
    
    u_sliding->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    torques.roll = u_sliding->Output(0);
    torques.pitch = u_sliding->Output(1);
    torques.yaw = u_sliding->Output(2);
    //thrust = u_sliding->Output(3);
    thrust = ComputeDefaultThrust();
    

}

void RobustFNN4Quasiperiodic::run_afnnc(Euler &torques){
    float tactual=double(GetTime())/1000000000-afnnc->t0;
    //printf("t: %f\n",tactual);
    Vector3Df xid, xidp, xidpp, xidppp;

    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Quaternion uav_quat;

    //flair::core::Time ti = GetTime();
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(uav_quat);
    //flair::core::Time  tf = GetTime()-ti;

    //Printf("pos: %f ms\n",  (float)tf/1000000);

    //Thread::Info("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    //Printf("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    //Printf("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);
    //Thread::Info("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);

    //ti = GetTime();
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    //tf = GetTime()-ti;

    //Printf("ori: %f ms\n",  (float)tf/1000000);
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    

    pos_reference(xid, xidp, xidpp, xidppp, tactual);

    //printf("xid: %f\t %f\t %f\n",xid.x,xid.y, xid.z);
    
    afnnc->SetValues(uav_pos-xid,uav_vel-xidp,xid,xidpp,xidppp,currentAngularRates,currentQuaternion);
    
    afnnc->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    

    torques.roll = afnnc->Output(0);
    torques.pitch = afnnc->Output(1);
    torques.yaw = afnnc->Output(2);
    thrust = afnnc->Output(3);
    


}
