//  created:    2011/05/01
//  filename:   RobustFNN4Quasiperiodic.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#ifndef PROYECTO22_H
#define PROYECTO22_H

#include <UavStateMachine.h>
#include "Sliding.h"
#include "AFNNC.h"

namespace flair {
    namespace gui {
        class PushButton;
        class ComboBox;
        class Tab;
        class TabWidget;
        class DoubleSpinBox;
        class GroupBox;
        class Label;
    }
    namespace filter {
        class ControlLaw;
        class Sliding;
        class AFNNC;
        class Sliding_force;
    }
    namespace meta {
        class MetaVrpnObject;
    }
}

class RobustFNN4Quasiperiodic : public flair::meta::UavStateMachine {
    public:
        RobustFNN4Quasiperiodic(flair::sensor::TargetController *controller);
        ~RobustFNN4Quasiperiodic();

    private:

	enum class BehaviourMode_t {
            Default,
            control,
            position_control
        }clTabCtrl;

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);
        void SignalEvent(Event_t event);
        void StartRobustFNN4Quasiperiodic(void);
        void StopRobustFNN4Quasiperiodic(void);
        void ExtraSecurityCheck(void);
        void ComputeCustomTorques(flair::core::Euler &torques);
        float ComputeCustomThrust(void);
        void sliding_ctrl(flair::core::Euler &torques);
        void run_afnnc(flair::core::Euler &torques);
        //const flair::core::AhrsData *GetOrientation(void) const;
        void pos_reference(flair::core::Vector3Df &xid, flair::core::Vector3Df &xidp, flair::core::Vector3Df &xidpp, flair::core::Vector3Df &xidppp, float tactual);

        flair::filter::Sliding *u_sliding;
        flair::filter::AFNNC *afnnc;

        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        
        //bool first_update;

        float thrust;

        flair::gui::DoubleSpinBox *xd, *yd, *zd, *ax, *wx, *bx, *ay, *wy, *by, *az, *wz, *bz;

        flair::gui::PushButton *start_prueba1,*stop_prueba1;
        flair::gui::ComboBox *control_select, *position_behavior, *xd_behavior, *yd_behavior, *zd_behavior;   
        flair::gui::Tab *setupLawTab2, *graphLawTab2, *lawTab2, *setupLawTab3, *graphLawTab3, *positionTab, *positiongTab, *adaptationGraphsTab;
        flair::gui::TabWidget *tabWidget2, *Pos_tabWidget;
        flair::gui::GroupBox *seg;
        flair::gui::Label *l, *l2, *lx, *ly, *lz;

        flair::gui::ComboBox *force_behavior, *fx_behavior, *fy_behavior, *fz_behavior;
        flair::gui::DoubleSpinBox *fxd, *fyd, *fzd, *afx, *wfx, *bfx, *afy, *wfy, *bfy, *afz, *wfz, *bfz;
        flair::gui::Label *lfx, *lfy, *lfz;

        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        
};

#endif // PROYECTO22_H
