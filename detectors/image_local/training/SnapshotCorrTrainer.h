//
// Created by sarkar on 17.03.15.
//

#ifndef _SNAPSHOT_SNAP_VTK_H_
#define _SNAPSHOT_SNAP_VTK_H_


//simplecube/newcube.obj  simplecube/flower.jpeg
//Lion/Final.obj Lion/Texture.png
//BigDaddy/Param.obj BigDaddy/Parameterization.png

#include <string>
#include <stdlib.h>

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkRendererCollection.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOBJReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <string>
#include <vtkRendererCollection.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkPointPicker.h>

#include <vtkImageReader2Factory.h>
#include <vtkImageReader.h>
#include <vtkTexture.h>
#include <vtkAxesActor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <sstream>
#include <opencv2/core/types.hpp>
#include <map>

#include <boost/filesystem.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vtkJPEGWriter.h>
#include <detectors/image_local/ODImageLocalMatching.h>

//extra
//xml
#include "pugixml.hpp"

#include "common/pipeline/ODTrainer.h"
#include "common/utils/utils.h"


#define VIEW_ANGLE 30
#define NO_SNAPSHOTS 30



using namespace std;

namespace od
{
  /** \brief ODImageLocalMatchingTrainer; One of the new algorithm; details will be explained later
   *
   * \author Kripasindhu Sarkar
   *
   */

  class SnapshotCorrTrainer : public ODImageLocalMatchingTrainer
  {

  public:
    SnapshotCorrTrainer(string const &training_input_location_="", string const &training_data_location_="") : ODImageLocalMatchingTrainer(
        training_input_location_, training_data_location_)
    { }

    int train();

    void trainSingleModel(string objname);

  };


}



#endif //_SNAPSHOT_SNAP_VTK_H_

