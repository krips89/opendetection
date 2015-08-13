//
// Created by sarkar on 17.03.15.
//

#ifndef _SNAPSHOT_SNAP_VTK_H_
#define _SNAPSHOT_SNAP_VTK_H_


#include "common/pipeline/ODTrainer.h"
#include "common/utils/utils.h"
#include "detectors/local2D/ODImageLocalMatching.h"


#define VIEW_ANGLE 30
#define NO_SNAPSHOTS 30





namespace od
{
  namespace l2d
  {
    /** \brief ODCADRecogTrainerSnapshotBased; One of the new algorithm; details will be explained later
   *
   * \author Kripasindhu Sarkar
   *
   */

    class ODCADRecogTrainerSnapshotBased : public ODImageLocalMatchingTrainer
    {

    public:
      ODCADRecogTrainerSnapshotBased(std::string const &training_input_location_ = "", std::string const &training_data_location_ = "") : ODImageLocalMatchingTrainer(
          training_input_location_, training_data_location_)
      { }

      int train();

      void init() {}

      void trainSingleModel(std::string objname);

    };

  }
}



#endif //_SNAPSHOT_SNAP_VTK_H_

