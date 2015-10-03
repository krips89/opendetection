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
#include "ODCADRecogTrainerSnapshotBased.h"
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


//extra
//xml
#include "pugixml.hpp"


using namespace std;

namespace od
{
  namespace l2d
  {

    class vtkTimerCallbackSnapshot : public vtkCommand
    {
    public:
      static vtkTimerCallbackSnapshot *New()
      {
        vtkTimerCallbackSnapshot *cb = new vtkTimerCallbackSnapshot;
        cb->snap_count = 0;
        cb->snap_mode = true;
        cb->feature_type = "SIFT";
        return cb;
      }

      virtual void Execute(vtkObject *caller, unsigned long eventId, void * vtkNotUsed(callData))
      {
        vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);


        if(!this->snap_mode) return;

        if(vtkCommand::TimerEvent == eventId)
        {
          ++this->snap_count;
        }

        //no need to do extra processing if we have enough snaps
        if(this->snap_count >= NO_SNAPSHOTS)
        {
          this->snap_mode = false;
          //after getting all snaps, write everything together
          cout << "Processing finished... Writing the final descriptors" << endl;

          string filename = boost::filesystem::path(input_file).filename().replace_extension(feature_type + "." + output_extension).c_str();
          FileUtils::createTrainingDir(output_dir);

          write_pairs(pairs_3d_2d, common_descriptors, output_dir + "/" + filename);
          write_pairs_xml(pairs_3d_2d, common_descriptors, output_dir + "/" + filename);

          //delete and remove everything
          iren->TerminateApp();
          pairs_3d_2d.clear();
        }



        //MAIN CALLBACK, take snapshot, find features for the data from current actor
        process(iren, actor, renderer, snap_count);

        //update data with the actor
        actor->RotateY(360 / NO_SNAPSHOTS);
        iren->GetRenderWindow()->Render();
      }

    private:
      int snap_count;
    public:

      string takeSnapshot(vtkRenderWindow *renderWindow, int snap_no);

      void write_pairs(vector<pair<cv::Point3f, cv::KeyPoint> > pairs, cv::Mat descriptors, string filename);

      void write_pairs_xml(vector<pair<cv::Point3f, cv::KeyPoint> > pairs, cv::Mat descriptors, string filename);

      void process_image(string imgname, vtkRenderer *ren, vtkActor *actor, int ino);

      void process(vtkRenderWindowInteractor *iren, vtkActor *actor, vtkRenderer *renderer, int ino);


      //some local variables used

      struct fcomp3d_euclidian
      {
        bool operator()(const cv::Point3f &lhs, const cv::Point3f &rhs) const
        { return lhs.x < rhs.x; }
      };

      vector<pair<cv::Point3f, cv::KeyPoint> > pairs_3d_2d;
      cv::Mat common_descriptors;
      map<cv::Point3f, cv::KeyPoint, fcomp3d_euclidian> map_3d_2d;
      string feature_type;
      string input_file, input_dir, output_dir, output_extension;


      vtkActor *actor;
      vtkRenderer *renderer;
      bool snap_mode;
    };

    int ODCADRecogTrainerSnapshotBased::train()
    {
      FileUtils::createTrainingDir(trained_data_location_);

      //get models in the directory
      std::vector<std::string> files;
      std::string start = "";
      std::string ext = std::string("obj");
      bf::path dir = training_input_location_;
      FileUtils::getFilesInDirectory(dir, start, files, ext);

      //for each models in the train_input_directory, train them and put them on the training_directory

      for(size_t i = 0; i < files.size(); i++)
      {
        trainSingleModel(training_input_location_ + "/" + files[i]);
      }

      return 1;
    }

    void ODCADRecogTrainerSnapshotBased::trainSingleModel(std::string objname)
    {
      ///////////setup object

      vtkSmartPointer<vtkTimerCallbackSnapshot> cb = vtkSmartPointer<vtkTimerCallbackSnapshot>::New();
      cb->input_file = objname;

      boost::filesystem::path p(objname);
      cb->input_dir = boost::filesystem::path(objname).parent_path().c_str();
      cb->output_dir = getSpecificTrainingDataLocation();
      cb->output_extension = TRAINED_DATA_ID_;

      vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
      reader->SetFileName(objname.c_str());
      reader->Update();

      //texture
      // Read texture file
      string texfilename = getTexfileinObj(objname);
      vtkSmartPointer<vtkImageReader2Factory> readerFactory = vtkSmartPointer<vtkImageReader2Factory>::New();
      vtkImageReader2 *imageReader = readerFactory->CreateImageReader2(texfilename.c_str());
      imageReader->SetFileName(texfilename.c_str());
      imageReader->Update();

      // Create texture
      vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
      texture->SetInputConnection(imageReader->GetOutputPort());
      texture->InterpolateOn();

      // Create a mapper and actor
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(reader->GetOutputPort());
      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->SetTexture(texture);
      ////////////////

      ////////////// Create a renderer, render window, and interactor, good camera, view and actors
      vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
      vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
      renderWindow->AddRenderer(renderer);
      renderWindow->SetSize(1024, 800);
      vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
      renderWindowInteractor->SetRenderWindow(renderWindow);

      // Add the actor to the scene
      renderer->AddActor(actor);
      renderer->SetBackground(1, 1, 1); // Background color white

      //Set up the camera at a very proper location and angle (about 30)
      renderer->ResetCamera();
      vtkCamera *camera = renderer->GetActiveCamera();
      camera->Elevation(VIEW_ANGLE);
      ////////////////////



      //Interactor style for custom handpicking and troubleshooting
      /*vtkSmartPointer<MouseInteractorStyle2> style = vtkSmartPointer<MouseInteractorStyle2>::New();
  style->SetDefaultRenderer(renderer);
  renderWindowInteractor->SetInteractorStyle(style);*/
      // Render and interact
      renderWindow->Render();
      // Initialize must be called prior to creating timer events.
      renderWindowInteractor->Initialize();


      //Callback timer to view the steps of taking snapshots
      cb->actor = actor;
      cb->renderer = renderer;
      renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
      int timerId = renderWindowInteractor->CreateRepeatingTimer(10);
      std::cout << "timerId: " << timerId << std::endl;

      // Start the interaction and timer
      renderWindowInteractor->Start();
    }


    std::string vtkTimerCallbackSnapshot::takeSnapshot(vtkRenderWindow *renderWindow, int snap_no)
    {
      // Screenshot
      vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
      windowToImageFilter->SetInput(renderWindow);
      //windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
      //windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
      //windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
      windowToImageFilter->Update();

      vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkJPEGWriter>::New();
      std::string filename;

      filename = input_dir + "/" + string("snapshot") + toString(snap_no) + ".jpg";

      writer->SetFileName(filename.c_str());
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      renderWindow->Render();
      writer->Write();
      return filename;
    }


    void vtkTimerCallbackSnapshot::write_pairs(vector<pair<cv::Point3f, cv::KeyPoint> > pairs, cv::Mat descriptors,
                                               string filename)
    {
      ofstream fout;
      fout.open(filename.c_str());

      //headers
      fout << pairs.size() << endl;
      fout << 3 << " " << descriptors.cols << endl;
      for(int i = 0; i < pairs.size(); i++)
      {
        fout << pairs[i].first.x << " " << pairs[i].first.y << " " << pairs[i].first.z << endl;
        fout << pairs[i].second.pt.x << " " << pairs[i].second.pt.y << " " << pairs[i].second.octave << " " << pairs[i].second.angle << " " << pairs[i].second.response << " " << pairs[i].second.size << endl;
        for(int j = 0; j < descriptors.cols; j++)
        {
          fout << descriptors.at<float>(i, j) << " ";
        }
        fout << endl;
      }
      fout.close();
    }

    void vtkTimerCallbackSnapshot::write_pairs_xml(vector<pair<cv::Point3f, cv::KeyPoint> > pairs, cv::Mat descriptors,
                                                   string filename)
    {

      pugi::xml_document doc;
      pugi::xml_node root = doc.append_child("Model");
      root.append_attribute("name") = filename.c_str();
      //root.append_attribute("objid") = "1";

      pugi::xml_node points_node = root.append_child("Points");
      for(int i = 0; i < pairs.size(); i++)
      {
        pugi::xml_node p_node = points_node.append_child("Point");

        ostringstream ss;
        ss << pairs[i].first.x << " " << pairs[i].first.y << " " << pairs[i].first.z;
        p_node.append_attribute("p3d") = ss.str().c_str();
        p_node.append_attribute("nviews") = "1";
        p_node.append_attribute("color") = "0 0 0";

        ss.str("");
        ss << pairs[i].second.pt.x << " " << pairs[i].second.pt.y << " " << pairs[i].second.octave << " " << pairs[i].second.angle << " " << pairs[i].second.response << " " << pairs[i].second.size;
        //cout << pairs[i].second.pt.x << " " << pairs[i].second.pt.y<< " " << pairs[i].second.octave << " " << pairs[i].second.angle << " " << pairs[i].second.response << " " << pairs[i].second.size << endl;
        //cout << ss.str() <<endl;

        p_node.append_attribute("p2d_prop") = ss.str().c_str();


        p_node.append_attribute("desc_type") = feature_type.c_str();
        ss.str("");
        for(int j = 0; j < descriptors.cols; j++)
        {
          ss << descriptors.at<float>(i, j) << " ";
        }

        p_node.append_attribute("desc") = ss.str().c_str();
      }
      //save
      doc.save_file(filename.c_str());
    }

    bool valid_3D_point(double *pt)
    {
      if((pt[0] == 0) && (pt[1] == 0) && (pt[2] == 0)) return false;
      return true;
    }

//1. find interesting 2d points, 2. find their corresponding 3d points corresponding to the transformation defined by the "actor"
//3. transform the 3d points to the original by using the inverse of the "actor"
    void vtkTimerCallbackSnapshot::process_image(string imgname, vtkRenderer *ren, vtkActor *actor, int ino)
    {
      //1. find interesting 2d location
      cv::Mat img = cv::imread(imgname);
      vector<cv::KeyPoint> kpts;
      cv::Mat descriptors_local, descriptors_local_good;
      cv::Ptr<cv::FeatureDetector> fdetector = cv::xfeatures2d::SIFT::create();
      fdetector->detect(img, kpts);
      fdetector->compute(img, kpts, descriptors_local);

      //2. find the corresponding 3d points for each 2d location
      vector<pair<cv::Point3f, cv::KeyPoint> > pairs_local;
      for(int i = 0; i < kpts.size(); i++)
      {
        //find 2d
        vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
        picker->Pick(kpts[i].pt.x, img.rows - kpts[i].pt.y, 0, ren); //IMPORTANT: note that the y coordinate is converted from pixel coordinates to picking coordinates

        //find transformed 3d
        double *pos = picker->GetPickPosition();

        //3 find original 3d
        vtkMatrix4x4 *curr_transform_inv = vtkMatrix4x4::New();
        vtkMatrix4x4::Invert(actor->GetMatrix(), curr_transform_inv);
        double *pos_model = curr_transform_inv->MultiplyDoublePoint(pos);

        //std::cout << "2D position is: " << kpts[i].pt.x << " " << kpts[i].pt.x << std::endl;
        //std::cout << "3d position (world coordinates) is: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl << endl;

        if(valid_3D_point(pos))
        {
          pairs_local.push_back(make_pair(cv::Point3f(pos_model[0], pos_model[1], pos_model[2]), kpts[i]));
          descriptors_local_good.push_back(descriptors_local.row(i));

          //updating extra globals!
          map_3d_2d[cv::Point3f(pos_model[0], pos_model[1], pos_model[2])] = kpts[i];
        }
      }
      //updating globals
      pairs_3d_2d.insert(pairs_3d_2d.end(), pairs_local.begin(), pairs_local.end());
      common_descriptors.push_back(descriptors_local_good);

      //writing a local set
      write_pairs(pairs_local, descriptors_local_good, input_dir + "/" + "local" + toString(ino) + ".pairs");
      write_pairs_xml(pairs_local, descriptors_local_good, input_dir + "/" + "local" + toString(ino) + ".xml");
      cout << "Processed view " << ino << endl;
    }

    void vtkTimerCallbackSnapshot::process(vtkRenderWindowInteractor *iren, vtkActor *actor, vtkRenderer *renderer, int ino)
    {

      vtkRenderWindow *win = iren->GetRenderWindow();

      //snap
      string snapname = takeSnapshot(win, ino);
      //save
      process_image(snapname, renderer, actor, ino);

    }


    class MouseInteractorStyle2 : public vtkInteractorStyleTrackballCamera
    {
    public:
      static MouseInteractorStyle2 *New();

      vtkTypeMacro(MouseInteractorStyle2, vtkInteractorStyleTrackballCamera);

      virtual void OnLeftButtonDown()
      {
        int *clickPos = this->GetInteractor()->GetEventPosition();

        // Pick from this location.
        /*vtkSmartPointer<vtkPointPicker>  picker =
    vtkSmartPointer<vtkPointPicker>::New();
  picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
*/
        vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
        picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

        double *pos = picker->GetPickPosition();
        std::cout << "Click position is: " << clickPos[0] << " " << clickPos[1] << std::endl;
        std::cout << "Pick position (world coordinates) is: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

        //std::cout << "Picked actor: " << picker->GetActor() << std::endl;
        //Create a sphere
        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(pos[0], pos[1], pos[2]);
        sphereSource->SetRadius(0.01);

        //Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphereSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);


        //this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetDefaultRenderer()->AddActor(actor);
        this->GetDefaultRenderer()->AddActor(actor);
        // Forward events
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
      }

    private:

    };

    vtkStandardNewMacro(MouseInteractorStyle2)


  }
}