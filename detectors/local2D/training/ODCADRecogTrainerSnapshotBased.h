/*
Copyright (c) 2015, Kripasindhu Sarkar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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

    protected:
      int no_ring_;
      float view_angle_;
      int no_snapshot_per_ring_;


    };
    /** \example objectdetector/od_image_cadrecog_camera.cpp
    * \example objectdetector/od_image_cadrecog_files.cpp
    */

  }
}



#endif //_SNAPSHOT_SNAP_VTK_H_

