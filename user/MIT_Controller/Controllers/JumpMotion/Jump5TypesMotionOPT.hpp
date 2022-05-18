#ifndef OPT_FRONTJUMP_CTRL
#define OPT_FRONTJUMP_CTRL

#include "DataReader.hpp"
#include "DataReadCtrl.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>
#include <Utilities/Utilities_print.h>

template <typename T>
class Jump5TypesMotionOPT : public DataReadCtrl<T> {
 public:
  Jump5TypesMotionOPT(DataReader*, float _dt);
  virtual ~Jump5TypesMotionOPT();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);
    Vec3<float> xref_w;
    int curr_iter_for_plot;
    int get_plan_num()
    {
        return DataCtrl::_data_reader->plan_timesteps;//间隔读取，需要除以2
    }
 protected:
  void _update_joint_command();

//private:

};

#endif
